"""
================================================================================
FUERZAS Y MOMENTOS AERODINÁMICOS -- BODY FRAME
================================================================================
Calcula F = [Fx, Fy, Fz] [N] y M = [L, M, N] [N*m] en ejes cuerpo como
función del estado de vuelo y las deflexiones de mando.

Modelo físico:
  * Ala:          CL(alpha), CD(alpha) de tablas XFLR5 + deflexión de alerón
  * Cola horiz.:  contribución a Cm con downwash y elevador
  * Cola vert.:   contribución a CY, Cn con sideslip y rudder (H-tail)
  * Gravedad:     transformada al body frame por DCM (ángulos de Euler)
  * Propulsión:   empuje del motor pusher en +X body; rotores en -Z body

Convención de signos (body frame):
  +X adelante (morro)   +Y ala derecha   +Z abajo
  Momento L (roll)+X    M (pitch)+Y       N (yaw)+Z

Referencias:
  Bryan (1911) "Stability in Aviation"
  Corda (2017) §6.5-6.8
  Nelson (1998) Ch.2-3
================================================================================
"""

import numpy as np
from typing import Tuple
from pathlib import Path

from parameters import AircraftConfig, get_aircraft_config
from xflr5_loader import load_all_data, AeroTableWing, AeroTableTail


# ============================================================================
# ÁNGULOS AERODINÁMICOS
# ============================================================================

def airspeed_angles(u: float, v: float, w: float
                    ) -> Tuple[float, float, float]:
    """
    Calcula velocidad aerodinámica, AoA y sideslip desde velocidades body.

    alpha = arctan(w/u)        [rad]  -- ángulo de ataque
    beta = arcsin(v/V)        [rad]  -- ángulo de resbalamiento lateral
    V = sqrt(u2 + v2 + w2) [m/s]

    Ref: Corda (2017) Eq. 6.1-6.3
    """
    V = np.sqrt(u**2 + v**2 + w**2)
    V = max(V, 0.5)   # protección hover / velocidad casi nula
    alpha = np.arctan2(w, u)
    beta  = np.arcsin(np.clip(v / V, -1.0, 1.0))
    return alpha, beta, V


def body_to_wind_dcm(alpha: float, beta: float) -> np.ndarray:
    """
    DCM de body frame a wind (stability) frame.
    Ref: Etkin & Reid (1996) §2.4
    """
    ca, sa = np.cos(alpha), np.sin(alpha)
    cb, sb = np.cos(beta),  np.sin(beta)
    return np.array([
        [ ca*cb, sb,  sa*cb],
        [-ca*sb, cb, -sa*sb],
        [-sa,    0.,   ca  ],
    ])


# ============================================================================
# MODELO AERODINÁMICO COMPLETO
# ============================================================================

class AeroModel:
    """
    Modelo aerodinámico del VTOL en modo crucero (ala fija).

    Usa tablas XFLR5 para el ala y la cola. Los momentos se calculan
    acumulando las contribuciones individuales referenciadas al CG.

    Args:
        cfg:        AircraftConfig con todos los parámetros
        data_dir:   Path al directorio Data/ (opcional; usa default si None)
    """

    def __init__(self, cfg: AircraftConfig, data_dir=None):
        self.cfg = cfg
        tables   = load_all_data() if data_dir is None else load_all_data(data_dir)
        self._tw: AeroTableWing = tables["table_wing"]
        self._tt: AeroTableTail = tables["table_tail"]

    # ------------------------------------------------------------------ #
    # COEFICIENTES GLOBALES                                                #
    # ------------------------------------------------------------------ #

    def coefficients(
        self,
        alpha: float, beta: float, V: float,
        p: float, q: float, r: float,
        delta_e: float = 0.0,
        delta_a: float = 0.0,
        delta_r: float = 0.0,
    ) -> dict:
        """
        Devuelve dict con CL, CD, CY, Cl, Cm, Cn totales del avión.

        Inputs en rad, rad/s.
        """
        cfg  = self.cfg
        w    = cfg.wing
        ht   = cfg.htail
        vt   = cfg.vtail
        a    = cfg.aero
        m    = cfg.mass

        c_bar = w.c_bar
        b     = w.b
        S     = w.S
        S_t   = ht.S_t
        S_v   = vt.S_v
        eta_t = ht.eta_t
        eta_v = vt.eta_v
        lt    = cfg.lt
        lv    = vt.x_ac_v - m.x_cg

        # Velocidades angulares adimensionales
        p_hat = p * b   / (2.0 * V)
        q_hat = q * c_bar / (2.0 * V)
        r_hat = r * b   / (2.0 * V)

        # ---- ALA ----
        CL_w = self._tw.CL(alpha)
        CD_w = self._tw.CD(alpha)
        Cm_w = self._tw.Cm(alpha)   # referenciado al origen XFLR5

        # Momento del ala respecto al CG [Corda Eq. 6.37]:
        # Cm_w_CG = Cm_w_ac + CL_w*(x_ac_w - x_cg)/c
        Cm_w_CG = Cm_w + CL_w * (w.x_ac - m.x_cg) / c_bar

        # ---- COLA HORIZONTAL ----
        # Ángulo de ataque efectivo en la cola con downwash [Corda Eq. 6.70]:
        # alpha_t = alpha - eps + i_t;  eps = deps/dalpha * alpha
        epsilon = a.deps_da * alpha
        alpha_t = alpha - epsilon + ht.i_t

        # Corrección por elevador: Deltaalpha_t = tau_e * delta_e
        alpha_t_eff = alpha_t + ht.tau_e * delta_e

        CL_t = eta_t * (S_t / S) * self._tt.CL(alpha_t_eff)
        CD_t = eta_t * (S_t / S) * self._tt.CD(alpha_t_eff)

        # Momento de cola respecto al CG [Corda Eq. 6.71]:
        # Cm_t = -CL_t * lt/c   (cola por detrás del CG -> signo negativo)
        Cm_t = -CL_t * (lt / c_bar)

        # Derivada de control elevador
        CLde_contrib = a.CLde * delta_e
        CMde_contrib = a.CMde * delta_e

        # ---- COLA VERTICAL (H-tail: 2 aletas) ----
        # Fuerza lateral por sideslip + rudder [Corda Eq. 6.108]:
        # CY = -eta_v*(Sv/S)*av*beta + CY_deltar*deltar
        av  = a.CLa_t * 0.85   # eficiencia 3D de aleta vertical [PROVISIONAL]
        CY_v   = -eta_v * (S_v / S) * av * beta + a.CYdr * delta_r
        # Yawing moment: Cn = VV*eta_v*av*beta  [Sadraey Eq. 6.55]
        Cn_v   =  (vt.n_fins * av * eta_v * S_v * lv) / (S * b) * beta \
                  + a.Cndr * delta_r
        # Rolling moment due to vtail [small, Provisional]:
        Cl_v   = a.Cldr * delta_r

        # ---- TOTAL SUSTENTACIÓN Y ARRASTRE ----
        CL_total = CL_w + CL_t + CLde_contrib
        CD_total = CD_w + CD_t

        # ---- MOMENTO DE PITCH (Cm) total ----
        Cm_total = (Cm_w_CG + Cm_t + CMde_contrib
                    + a.Cmq * q_hat + a.Cma_dot * q_hat)
        # Nota: Cma_dot usa q_hat como proxy de alpha_dot en régimen cuasi-estático

        # ---- FUERZA LATERAL (CY) ----
        CY_total = CY_v

        # ---- MOMENTO DE ALABEO (Cl) ----
        Cl_total = (a.Clb * beta + a.Clp * p_hat + a.Clr * r_hat
                    + a.Clda * delta_a + Cl_v)

        # ---- MOMENTO DE GUIÑADA (Cn) ----
        Cn_total = (a.Cnb * beta + a.Cnp * p_hat + a.Cnr * r_hat
                    + a.Cnda * delta_a + Cn_v)

        return {
            "CL": CL_total, "CD": CD_total, "CY": CY_total,
            "Cl": Cl_total, "Cm": Cm_total, "Cn": Cn_total,
            "CL_w": CL_w, "CD_w": CD_w, "CL_t": CL_t,
            "alpha_t_eff": alpha_t_eff,
        }

    # ------------------------------------------------------------------ #
    # FUERZAS Y MOMENTOS EN BODY FRAME                                    #
    # ------------------------------------------------------------------ #

    def forces_moments(
        self,
        u: float, v: float, w_vel: float,
        p: float, q: float, r: float,
        phi: float, theta: float,
        delta_e: float = 0.0,
        delta_a: float = 0.0,
        delta_r: float = 0.0,
        thrust:  float = 0.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calcula fuerzas [Fx,Fy,Fz] [N] y momentos [L,M,N] [N*m] en body frame.

        Args:
            u, v, w_vel : componentes de velocidad body [m/s]
            p, q, r     : velocidades angulares body [rad/s]
            phi, theta  : ángulos de Euler roll y pitch [rad]
            delta_e     : deflexión elevador [rad]  + = borde de fuga arriba
            delta_a     : deflexión alerón  [rad]  + = ala der. baja
            delta_r     : deflexión rudder  [rad]
            thrust      : empuje motor pusher [N]  (actúa en +X body)

        Returns:
            F = np.array([Fx, Fy, Fz])  [N]
            M = np.array([L,  M,  N])   [N*m]
        """
        cfg  = self.cfg
        w    = cfg.wing
        m    = cfg.mass
        atm  = cfg.atm

        alpha, beta, V = airspeed_angles(u, v, w_vel)
        q_dyn = 0.5 * atm.rho * V**2    # presión dinámica [Pa]
        S     = w.S
        c_bar = w.c_bar
        b     = w.b

        # Coeficientes totales
        coef = self.coefficients(
            alpha, beta, V, p, q, r, delta_e, delta_a, delta_r
        )
        CL = coef["CL"]
        CD = coef["CD"]
        CY = coef["CY"]
        Cl = coef["Cl"]
        Cm = coef["Cm"]
        Cn = coef["Cn"]

        # Fuerzas en ejes viento (L = lift _ V, D = drag _ V)
        L_force = q_dyn * S * CL   # [N]
        D_force = q_dyn * S * CD   # [N]
        Y_force = q_dyn * S * CY   # [N]

        # Transformar L, D al body frame
        # [Corda Eq. 6.12-6.14]  (beta ~= 0 para vuelo nominal)
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta),  np.sin(beta)

        Fx_aero = -D_force * ca * cb + Y_force * sa * sb - L_force * sa
        Fy_aero = -D_force * sb      + Y_force * cb
        Fz_aero =  D_force * sa * cb - Y_force * ca * sb - L_force * ca
        # Nota: Fz_aero es negativo en vuelo normal (lift hacia arriba = -Z body)

        # Gravedad en body frame [Corda Eq. 4.28]
        g     = atm.g
        mass  = m.m
        Fx_g  = -mass * g * np.sin(theta)
        Fy_g  =  mass * g * np.cos(theta) * np.sin(phi)
        Fz_g  =  mass * g * np.cos(theta) * np.cos(phi)

        # Empuje pusher (en +X body)
        Fx_T  = thrust

        # Fuerzas totales
        F = np.array([
            Fx_aero + Fx_g + Fx_T,
            Fy_aero + Fy_g,
            Fz_aero + Fz_g,
        ])

        # Momentos en body frame
        M_vec = np.array([
            q_dyn * S * b     * Cl,   # L -- roll   [N*m]
            q_dyn * S * c_bar * Cm,   # M -- pitch  [N*m]
            q_dyn * S * b     * Cn,   # N -- yaw    [N*m]
        ])

        return F, M_vec


# ============================================================================
# EMPUJE ROTORES VTOL
# ============================================================================

class PropModel:
    """
    Modelo de propulsión simple para modo VTOL y crucero.

    Modo HOVER:  4 rotores generan empuje en -Z body.
    Modo CRUISE: motor pusher genera empuje en +X body.
    """

    def __init__(self, cfg: AircraftConfig):
        self.cfg = cfg

    def thrust_pusher(self, throttle: float) -> float:
        """Empuje motor pusher [N]. throttle _ [0, 1]."""
        return np.clip(throttle, 0.0, 1.0) * self.cfg.prop.T_max_pusher

    def thrust_rotors(self, throttle: float) -> float:
        """Empuje total de 4 rotores [N] en +Z_NED (-Z body). throttle _ [0,1]."""
        T = np.clip(throttle, 0.0, 1.0)**2 * self.cfg.prop.T_max_rotor
        return 4.0 * T


# ============================================================================
# MAIN -- test básico
# ============================================================================

if __name__ == "__main__":
    cfg   = get_aircraft_config()
    model = AeroModel(cfg)
    prop  = PropModel(cfg)

    print("=" * 60)
    print(" FORCES_MOMENTS -- TEST BÁSICO")
    print("=" * 60)

    # Condición de crucero trimado (alpha pequeño, vuelo nivelado)
    V_c   = cfg.V_cruise
    alpha0 = np.radians(3.5)   # ángulo de ataque aproximado en trim
    u0    = V_c * np.cos(alpha0)
    w0    = V_c * np.sin(alpha0)

    F, M = model.forces_moments(
        u=u0, v=0.0, w_vel=w0,
        p=0.0, q=0.0, r=0.0,
        phi=0.0, theta=alpha0,
        thrust=cfg.mass.m * cfg.atm.g * 0.12,   # ~12% empuje para crucero
    )

    W = cfg.mass.m * cfg.atm.g
    print(f"Peso:     W  = {W:.2f} N")
    print(f"Fuerzas:  Fx = {F[0]:+.2f} N  Fy = {F[1]:+.2f} N  Fz = {F[2]:+.2f} N")
    print(f"Momentos: L  = {M[0]:+.4f} N*m  M = {M[1]:+.4f} N*m  N = {M[2]:+.4f} N*m")
    print(f"(Fz~=-W en trim -> Fz ~ {-W:.1f} N esperado)")

    # Coeficientes en esa condición
    alpha0b, beta0, V0 = airspeed_angles(u0, 0.0, w0)
    coef = model.coefficients(alpha0b, beta0, V0, 0, 0, 0)
    q_c = 0.5 * cfg.atm.rho * V0**2
    print(f"\nCoeficientes @ alpha={np.degrees(alpha0b):.2f}°:")
    print(f"  CL = {coef['CL']:.4f}  CD = {coef['CD']:.5f}  Cm = {coef['Cm']:.5f}")
    print(f"  L/D = {coef['CL']/coef['CD']:.2f}")
    print(f"\n[OK] forces_moments.py OK")
