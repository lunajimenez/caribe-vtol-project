"""
================================================================================
ANÁLISIS DE ESTABILIDAD -- ESTÁTICA Y DINÁMICA COMPLETA
================================================================================
Cubre:
  A) Estabilidad estática longitudinal y lateral-direccional
  B) Linealización alrededor del trim de crucero
  C) Análisis de eigenvalores -- los 5 modos dinámicos clásicos:
       Longitudinal: Phugoid, Short Period
       Lateral:      Dutch Roll, Roll Mode, Spiral Mode

Método de linealización:
  Sistema longitudinal  ->  A_lon (4x4)  estados: [Deltau, Deltaw, Deltaq, Deltatheta]
  Sistema lateral-direc ->  A_lat (4x4)  estados: [Deltabeta, Deltap, Deltar, Delta_]

  Las derivadas dimensionales se construyen desde las adimensionales usando
  la velocidad de trim V_, la masa m, las inercias Iyy / Ixx / Izz / Ixz
  y la presión dinámica q_ según Nelson (1998) Ch.4.

Referencias:
  Corda (2017) §6.5-6.9  -- estática y fórmulas analíticas de modos
  Nelson (1998) Ch.4-5   -- forma estado-espacio, derivadas dimensionales
  Sadraey (2012) Ch.6     -- parámetros geométricos de empenaje
================================================================================
"""

import numpy as np
from typing import Dict, Tuple, List
from pathlib import Path
import os

from parameters import AircraftConfig, get_aircraft_config
from dynamics_6dof import Dynamics6DOF, State, Controls, numerical_jacobian
from forces_moments import AeroModel, airspeed_angles


# ============================================================================
# A) ESTABILIDAD ESTÁTICA
# ============================================================================

class StaticStability:
    """Punto neutro, margen estático, trim y derivadas de control."""

    def __init__(self, cfg: AircraftConfig):
        self.cfg = cfg

    # ------------------------------------------------------------------ #
    # PUNTO NEUTRO Y MARGEN ESTÁTICO                                      #
    # ------------------------------------------------------------------ #

    def neutral_point(self) -> float:
        """
        Posición del punto neutro desde el morro [m].

        Fórmula de Corda (2017) Eq. 6.88:
          x_NP = x_acw + c*VH*eta_t*(CLa_t/CLa_w)*(1 - deps/dalpha)

        Returns:
            x_NP [m] desde el morro
        """
        cfg  = self.cfg
        w    = cfg.wing
        ht   = cfg.htail
        a    = cfg.aero

        x_NP = (w.x_ac
                + w.c_bar * a.VH * ht.eta_t
                * (a.CLa_t / a.CLa_w)
                * (1.0 - a.deps_da))
        return x_NP

    def static_margin(self) -> float:
        """
        Margen estático SM = (x_NP - x_cg) / c.

        SM > 0  ->  CG adelante del NP  ->  estable
        """
        x_NP = self.neutral_point()
        return (x_NP - self.cfg.mass.x_cg) / self.cfg.wing.c_bar

    def Cmalpha_total(self) -> float:
        """
        Cmalpha total respecto al CG [/rad].

        Cmalpha = CLalpha_total * (x_cg - x_NP)/c   (negativo = estable)
        Ref: Corda (2017) Eq. 6.89
        """
        cfg  = self.cfg
        w    = cfg.wing
        ht   = cfg.htail
        a    = cfg.aero

        CLa_total = a.CLa_w + ht.eta_t * (ht.S_t / w.S) * a.CLa_t * (1 - a.deps_da)
        x_NP = self.neutral_point()
        return CLa_total * (cfg.mass.x_cg - x_NP) / w.c_bar

    # ------------------------------------------------------------------ #
    # TRIM EN CRUCERO                                                      #
    # ------------------------------------------------------------------ #

    def trim_cruise(self) -> Dict:
        """
        Resuelve condiciones de trim para vuelo nivelado en crucero.

        Sistema de 2 ecuaciones:
          _Fz = 0  ->  CL_trim = W / (q*S)
          _M  = 0  ->  Cm_total(alpha_trim, deltae_trim) = 0

        Cm_total = CM0_w + CMalpha_total*alpha + CMde*deltae = 0
        CL_total = CL0_w + CLalpha_total*alpha + CLde*deltae = CL_trim

        Ref: Corda (2017) Eq. 6.96-6.97
        """
        cfg   = self.cfg
        a     = cfg.aero
        w     = cfg.wing
        ht    = cfg.htail
        m     = cfg.mass
        atm   = cfg.atm
        V0    = cfg.V_cruise
        q0    = cfg.q_cruise

        CL_trim = cfg.CL_trim

        CLa_total = (a.CLa_w
                     + ht.eta_t * (ht.S_t / w.S) * a.CLa_t * (1 - a.deps_da))
        CMa_total = self.Cmalpha_total()

        # CM0 (momento a alpha=0, deltae=0): del ala más contribución de incidencia de cola
        # Cm_wing_origin a alpha=0 (XFLR5): a.CM0_w
        # Contribución de incidencia de cola: -CLa_t*eta_t*(St/S)*it*(lt/c)
        lt    = cfg.lt
        Cm0   = (a.CM0_w
                 - a.CLa_t * ht.eta_t * (ht.S_t / w.S)
                   * ht.i_t * (lt / w.c_bar))

        # Resolvemos el sistema 2x2:
        # CLa_total*alpha + CLde*deltae = CL_trim - CL0_w
        # CMa_total*alpha + CMde*deltae = -Cm0
        A_mat = np.array([
            [CLa_total, a.CLde],
            [CMa_total, a.CMde],
        ])
        b_vec = np.array([
            CL_trim - a.CL0_w,
            -Cm0,
        ])

        try:
            sol = np.linalg.solve(A_mat, b_vec)
            alpha_trim, delta_e_trim = sol
        except np.linalg.LinAlgError:
            alpha_trim  = CL_trim / CLa_total
            delta_e_trim = 0.0

        # Empuje necesario para equilibrar arrastre (polar parabólica)
        CD_trim = a.CD0_cruise + (1.0 / (np.pi * a.e_oswald * w.AR)) * CL_trim**2
        D_trim  = q0 * w.S * CD_trim
        T_trim  = D_trim   # [N] en crucero nivelado

        return {
            "alpha_trim_rad": alpha_trim,
            "alpha_trim_deg": np.degrees(alpha_trim),
            "delta_e_trim_rad": delta_e_trim,
            "delta_e_trim_deg": np.degrees(delta_e_trim),
            "CL_trim":  CL_trim,
            "CD_trim":  CD_trim,
            "T_trim_N": T_trim,
            "LD_trim":  CL_trim / CD_trim,
        }

    # ------------------------------------------------------------------ #
    # REPORTE ESTÁTICO                                                     #
    # ------------------------------------------------------------------ #

    def report(self) -> Dict:
        x_NP = self.neutral_point()
        SM   = self.static_margin()
        CMa  = self.Cmalpha_total()
        trim = self.trim_cruise()
        return {
            "x_NP_m": x_NP,
            "SM_frac": SM,
            "SM_pct":  SM * 100.0,
            "Cmalpha": CMa,
            "trim":    trim,
            "stable_longitudinal": SM > 0.0,
        }


# ============================================================================
# B) LINEALIZACIÓN -- MATRICES A_lon Y A_lat
# ============================================================================

class DynamicLinearization:
    """
    Construye las matrices de estado linealizadas A_lon (4x4) y A_lat (4x4).

    Derivadas dimensionales a partir de adimensionales según Nelson (1998):
      Xu = -q*S*2*CD0 / (m*V0)
      Zu = -q*S*2*CL0 / (m*V0)
      Zw = -q*S*CLa   / (m*V0)
      Mw = q*S*c*CMa  / (Iyy*V0)
      Mq = q*S*c2*Cmq / (2*Iyy*V0)
      etc.

    Ref: Nelson (1998) Table 4.5; Corda (2017) §6.9
    """

    def __init__(self, cfg: AircraftConfig):
        self.cfg  = cfg
        self.stat = StaticStability(cfg)

    def _trim_state(self) -> Dict:
        return self.stat.trim_cruise()

    def longitudinal_matrix(self) -> Tuple[np.ndarray, np.ndarray, Dict]:
        """
        Matriz A_lon para sistema [Deltau, Deltaw, Deltaq, Deltatheta].

        B_lon para entrada [Deltadeltae].

        Returns:
            A_lon (4x4), B_lon (4x1), derivadas dimensionales dict
        """
        cfg   = self.cfg
        a     = cfg.aero
        w     = cfg.wing
        ht    = cfg.htail
        m     = cfg.mass
        atm   = cfg.atm

        V0    = cfg.V_cruise
        q0    = cfg.q_cruise
        S     = w.S
        c_bar = w.c_bar
        mass  = m.m
        Iyy   = m.Iyy
        g     = atm.g

        trim  = self._trim_state()
        CL0   = trim["CL_trim"]
        CD0   = a.CD0_cruise
        K     = cfg.K

        # ---- Derivadas adimensionales ----
        CLa_total = (a.CLa_w
                     + ht.eta_t * (ht.S_t / S) * a.CLa_t * (1.0 - a.deps_da))
        CMa_total = self.stat.Cmalpha_total()

        # ---- Derivadas dimensionales [Nelson 1998 Table 4.5] ----
        # Fuerza X (drag):
        Xu  = -(q0 * S * 2.0 * CD0)     / (mass * V0)
        Xw  =  (q0 * S * (CL0 - 2*K*CL0*CLa_total)) / (mass * V0)

        # Fuerza Z (lift):
        Zu  = -(q0 * S * 2.0 * CL0)     / (mass * V0)
        Zw  = -(q0 * S * CLa_total)      / (mass * V0)
        Zq  = -(q0 * S * c_bar * a.CLq) / (2.0 * mass * V0)
        Za_dot = -(q0 * S * c_bar * a.CLa_dot) / (2.0 * mass * V0)

        # Momento M (pitch):
        Mu  =  0.0    # pequeño, Mach bajo (incompresible)
        Mw  =  (q0 * S * c_bar * CMa_total) / (Iyy * V0)
        Mq  =  (q0 * S * c_bar**2 * a.Cmq) / (2.0 * Iyy * V0)
        Ma_dot = (q0 * S * c_bar**2 * a.Cma_dot) / (2.0 * Iyy * V0)

        # ---- Matriz A_lon [Deltau, Deltaw, Deltaq, Deltatheta] ----
        # Ref: Nelson (1998) Eq. 4.56 (con alpha_dot -> simplificación Za_dot en Zw*)
        # Zw* = Zw / (1 - Za_dot*c/2V)  (corrección por alpha_dot)
        denom = 1.0 - Za_dot * c_bar / (2.0 * V0)
        Zw_star = Zw  / denom
        Zq_star = (Zq + V0 * mass) / (mass * denom)
        Mw_star = Mw  + Ma_dot * Zw  / denom
        Mq_star = Mq  + Ma_dot * (Zq + V0 * mass) / (mass * denom * Iyy)

        A_lon = np.array([
            [Xu,   Xw,          0.0,          -g             ],
            [Zu/V0*V0,  Zw,     V0,           0.0            ],
            [Mu,   Mw,          Mq,           0.0            ],
            [0.0,  0.0,         1.0,          0.0            ],
        ])
        # Corrección fila Zw (incluye q*V0 para vuelo nivelado)
        A_lon[1, :] = [Zu, Zw, Zq / mass + V0, 0.0]
        A_lon[2, :] = [Mu, Mw, Mq, 0.0]

        # B_lon para deltae
        Zde = -(q0 * S * a.CLde) / (mass)
        Mde =  (q0 * S * c_bar * a.CMde) / Iyy
        B_lon = np.array([[0.0], [Zde], [Mde], [0.0]])

        derivs = {
            "Xu": Xu, "Xw": Xw,
            "Zu": Zu, "Zw": Zw, "Zq": Zq,
            "Mu": Mu, "Mw": Mw, "Mq": Mq,
            "Za_dot": Za_dot, "Ma_dot": Ma_dot,
            "Zde": Zde, "Mde": Mde,
        }
        return A_lon, B_lon, derivs

    def lateral_matrix(self) -> Tuple[np.ndarray, np.ndarray, Dict]:
        """
        Matriz A_lat para sistema [Deltabeta, Deltap, Deltar, Delta_].

        B_lat para entradas [Deltadeltaa, Deltadeltar].

        Ref: Nelson (1998) Eq. 5.41; Corda (2017) §6.9.3-6.9.5
        """
        cfg   = self.cfg
        a     = cfg.aero
        w     = cfg.wing
        m     = cfg.mass
        atm   = cfg.atm

        V0    = cfg.V_cruise
        q0    = cfg.q_cruise
        S     = w.S
        b     = w.b
        mass  = m.m
        Ixx   = m.Ixx
        Izz   = m.Izz
        Ixz   = m.Ixz
        g     = atm.g

        trim  = self._trim_state()
        CL0   = trim["CL_trim"]

        # ---- Derivadas dimensionales laterales [Nelson (1998) Table 5.2] ----
        Yb  = (q0 * S * a.CYb)           / (mass)
        Lp  = (q0 * S * b**2 * a.Clp)   / (2.0 * Ixx * V0)
        Lr  = (q0 * S * b**2 * a.Clr)   / (2.0 * Ixx * V0)
        Lb  = (q0 * S * b   * a.Clb)    / Ixx
        Np  = (q0 * S * b**2 * a.Cnp)   / (2.0 * Izz * V0)
        Nr  = (q0 * S * b**2 * a.Cnr)   / (2.0 * Izz * V0)
        Nb  = (q0 * S * b   * a.Cnb)    / Izz

        # Corrección por producto de inercia Ixz [Nelson Eq. 5.38]
        Gamma = Ixx * Izz - Ixz**2
        Lp_  = (Izz * Lp  + Ixz * Np)  / Gamma * Ixx
        Lr_  = (Izz * Lr  + Ixz * Nr)  / Gamma * Ixx
        Lb_  = (Izz * Lb  + Ixz * Nb)  / Gamma * Ixx
        Np_  = (Ixz * Lp  + Ixx * Np)  / Gamma * Izz
        Nr_  = (Ixz * Lr  + Ixx * Nr)  / Gamma * Izz
        Nb_  = (Ixz * Lb  + Ixx * Nb)  / Gamma * Izz

        # ---- Matriz A_lat [Deltabeta, Deltap, Deltar, Delta_] ----
        A_lat = np.array([
            [Yb / V0,  0.0,  -(1.0 - Yb / (V0 * mass)),  g / V0],
            [Lb_,      Lp_,   Lr_,                         0.0   ],
            [Nb_,      Np_,   Nr_,                         0.0   ],
            [0.0,      1.0,   0.0,                         0.0   ],
        ])
        # Corrección fila beta: Yb/V0, 0, -1+Yb/(m*V0), g/V0
        A_lat[0, :] = [Yb / V0, 0.0, -1.0, g / V0]

        # B_lat [Deltadeltaa, Deltadeltar]
        Lda  = (q0 * S * b * a.Clda)  / Ixx
        Ldr  = (q0 * S * b * a.Cldr)  / Ixx
        Nda  = (q0 * S * b * a.Cnda)  / Izz
        Ndr  = (q0 * S * b * a.Cndr)  / Izz
        Ydr  = (q0 * S     * a.CYdr)  / mass

        Lda_ = (Izz * Lda + Ixz * Nda) / Gamma * Ixx
        Ldr_ = (Izz * Ldr + Ixz * Ndr) / Gamma * Ixx
        Nda_ = (Ixz * Lda + Ixx * Nda) / Gamma * Izz
        Ndr_ = (Ixz * Ldr + Ixx * Ndr) / Gamma * Izz

        B_lat = np.array([
            [0.0,      Ydr / V0],
            [Lda_,     Ldr_   ],
            [Nda_,     Ndr_   ],
            [0.0,      0.0    ],
        ])

        derivs = {
            "Yb": Yb, "Lb": Lb_, "Nb": Nb_,
            "Lp": Lp_, "Lr": Lr_,
            "Np": Np_, "Nr": Nr_,
            "Lda": Lda_, "Ldr": Ldr_,
            "Nda": Nda_, "Ndr": Ndr_,
        }
        return A_lat, B_lat, derivs


# ============================================================================
# C) ANÁLISIS DE EIGENVALORES -- 5 MODOS
# ============================================================================

def _mode_params(lam: complex) -> Dict:
    """Extrae omegan, zeta, período, t_half/double de un eigenvalor."""
    sigma = lam.real
    omega = abs(lam.imag)
    wn    = abs(lam)

    zeta  = -sigma / wn if wn > 1e-8 else (1.0 if sigma < 0 else -1.0)
    is_oscillatory = omega > 1e-4

    period = 2.0 * np.pi / omega if is_oscillatory else np.inf

    if sigma < -1e-8:
        t_half = np.log(2.0) / abs(sigma)
        t_double = np.inf
        stable = True
    elif sigma > 1e-8:
        t_half   = np.inf
        t_double = np.log(2.0) / sigma
        stable   = False
    else:
        t_half = t_double = np.inf
        stable = True

    return {
        "lambda": lam,
        "sigma": sigma, "omega_d": omega,
        "omega_n": wn, "zeta": zeta,
        "period_s": period,
        "t_half_s": t_half,
        "t_double_s": t_double,
        "is_oscillatory": is_oscillatory,
        "stable": stable,
    }


def analyze_longitudinal(A_lon: np.ndarray) -> Dict:
    """
    Identifica modos Phugoid y Short Period desde A_lon.

    Criterio de identificación [Corda §6.9]:
      Phugoid:      |omegan| < 0.5 rad/s  (período largo ~30-100 s)
      Short Period: |omegan| > 0.5 rad/s  (período corto ~1-5 s)
    """
    eigs = np.linalg.eigvals(A_lon)
    modes_raw = [_mode_params(e) for e in eigs]

    # Separar oscilatorios de reales
    osc  = [m for m in modes_raw if m["is_oscillatory"]]
    real = [m for m in modes_raw if not m["is_oscillatory"]]

    # Ordenar oscilatorios por omegan ascendente
    osc_sorted = sorted(osc, key=lambda m: m["omega_n"])

    result = {}

    if len(osc_sorted) >= 2:
        result["phugoid"]      = {**osc_sorted[0],  "name": "Phugoid"}
        result["short_period"] = {**osc_sorted[-1], "name": "Short Period"}
    elif len(osc_sorted) == 1:
        result["phugoid"]      = {**osc_sorted[0], "name": "Phugoid"}
        result["short_period"] = None
    else:
        result["phugoid"]      = None
        result["short_period"] = None

    # Fórmulas analíticas de Corda (2017) Eq. 6.138-6.140
    cfg = None  # se inyecta desde afuera si se desea comparar
    result["eigenvalues"] = eigs
    return result


def analyze_lateral(A_lat: np.ndarray) -> Dict:
    """
    Identifica modos Dutch Roll, Roll Mode y Spiral desde A_lat.

    Criterio [Corda §6.9]:
      Dutch Roll: eigenvalor complejo conjugado (oscilatorio)
      Roll Mode:  eigenvalor real negativo grande (|lambda| > 0.5)
      Spiral:     eigenvalor real pequeño (|lambda| < 0.5), puede ser > 0
    """
    eigs = np.linalg.eigvals(A_lat)
    modes_raw = [_mode_params(e) for e in eigs]

    osc  = [m for m in modes_raw if m["is_oscillatory"]]
    real = sorted(
        [m for m in modes_raw if not m["is_oscillatory"]],
        key=lambda m: abs(m["sigma"]),
        reverse=True,
    )

    result = {}

    # Dutch Roll: único modo oscilatorio lateral
    if osc:
        result["dutch_roll"] = {**osc[0], "name": "Dutch Roll"}
    else:
        result["dutch_roll"] = None

    # Roll Mode: mayor |lambda| real
    if len(real) >= 1:
        result["roll_mode"] = {**real[0], "name": "Roll Mode",
                               "tau_s": 1.0 / abs(real[0]["sigma"]) if abs(real[0]["sigma"]) > 1e-6 else np.inf}
    else:
        result["roll_mode"] = None

    # Spiral: menor |lambda| real
    if len(real) >= 2:
        result["spiral"] = {**real[-1], "name": "Spiral Mode"}
    else:
        result["spiral"] = None

    result["eigenvalues"] = eigs
    return result


def phugoid_analytic(cfg: AircraftConfig) -> Dict:
    """
    Fórmulas analíticas para el Phugoid [Corda (2017) Eq. 6.138-6.140].

      zeta_ph  = 1 / (sqrt2 * L/D)
      omegan_ph = sqrt2 * g / V_
      T_ph  = 2_ / omegan_ph
    """
    trim  = StaticStability(cfg).trim_cruise()
    LD    = trim["LD_trim"]
    V0    = cfg.V_cruise
    g     = cfg.atm.g

    zeta  = 1.0 / (np.sqrt(2.0) * LD)
    wn    = np.sqrt(2.0) * g / V0
    T     = 2.0 * np.pi / wn
    sigma = -zeta * wn
    omega_d = wn * np.sqrt(max(1 - zeta**2, 0.0))

    return {
        "name": "Phugoid (analítico)",
        "zeta": zeta, "omega_n": wn,
        "omega_d": omega_d,
        "period_s": T,
        "sigma": sigma,
        "stable": zeta > 0,
    }


# ============================================================================
# D) ANÁLISIS COMPLETO
# ============================================================================

class StabilityAnalyzer:
    """Interfaz única para todo el análisis de estabilidad."""

    def __init__(self, cfg: AircraftConfig = None):
        self.cfg   = cfg or get_aircraft_config()
        self.stat  = StaticStability(self.cfg)
        self.lin   = DynamicLinearization(self.cfg)

    def run(self) -> Dict:
        """
        Ejecuta el análisis completo. Devuelve dict con todos los resultados.
        """
        # Estática
        static = self.stat.report()

        # Matrices linealizadas
        A_lon, B_lon, deriv_lon = self.lin.longitudinal_matrix()
        A_lat, B_lat, deriv_lat = self.lin.lateral_matrix()

        # Modos longitudinales
        lon_modes = analyze_longitudinal(A_lon)
        ph_an     = phugoid_analytic(self.cfg)

        # Modos laterales
        lat_modes = analyze_lateral(A_lat)

        return {
            "static":    static,
            "A_lon":     A_lon,
            "B_lon":     B_lon,
            "A_lat":     A_lat,
            "B_lat":     B_lat,
            "deriv_lon": deriv_lon,
            "deriv_lat": deriv_lat,
            "lon_modes": lon_modes,
            "lat_modes": lat_modes,
            "phugoid_analytic": ph_an,
        }


# ============================================================================
# E) REPORTE A PANTALLA
# ============================================================================

def _fmt_mode(m: Dict, label: str) -> List[str]:
    if m is None:
        return [f"  {label:16s}: No identificado"]
    zeta   = m.get("zeta", float("nan"))
    wn     = m.get("omega_n", float("nan"))
    T      = m.get("period_s", float("nan"))
    th     = m.get("t_half_s",   float("nan"))
    td     = m.get("t_double_s", float("nan"))
    stable = m.get("stable", False)
    status = "ESTABLE" if stable else "INESTABLE"

    lines = [f"  {label:16s}: zeta={zeta:+.4f}  omegan={wn:.4f} rad/s"]
    if np.isfinite(T):
        lines.append(f"  {'':16s}  T={T:.2f} s")
    if np.isfinite(th):
        lines.append(f"  {'':16s}  t_half={th:.2f} s  [{status}]")
    elif np.isfinite(td):
        lines.append(f"  {'':16s}  t_double={td:.2f} s  [{status}]")
    else:
        lines.append(f"  {'':16s}  [{status}]")

    if "tau_s" in m:
        lines.append(f"  {'':16s}  tau={m['tau_s']:.3f} s")
    return lines


def print_stability_report(results: Dict):
    """Imprime el reporte en el formato requerido por la especificación."""
    s  = results["static"]
    tr = s["trim"]
    lm = results["lon_modes"]
    lt = results["lat_modes"]
    ph = results["phugoid_analytic"]

    print("\n" + "=" * 64)
    print("=== STATIC STABILITY ===")
    print(f"  Neutral Point:  x_NP = {s['x_NP_m']:.3f} m from nose")
    print(f"  Static Margin:  SM   = {s['SM_pct']:.1f}% MAC  "
          f"[{'STABLE' if s['stable_longitudinal'] else 'UNSTABLE'}]")
    print(f"  Cm_alpha:           {s['Cmalpha']:.4f} /rad")
    print(f"  Trim at cruise: alpha = {tr['alpha_trim_deg']:.2f}°  "
          f"deltae = {tr['delta_e_trim_deg']:.2f}°")
    print(f"  CL_trim={tr['CL_trim']:.4f}  CD_trim={tr['CD_trim']:.5f}  "
          f"L/D={tr['LD_trim']:.2f}")

    print("\n=== DYNAMIC STABILITY -- LONGITUDINAL ===")
    ph_eig = lm.get("phugoid")
    sp_eig = lm.get("short_period")

    if ph_eig:
        lines = _fmt_mode(ph_eig, "Phugoid (eig)")
        for l in lines: print(l)
    # Analítico también
    print(f"  {'Phugoid (anal)':16s}: zeta={ph['zeta']:.4f}  omegan={ph['omega_n']:.4f} rad/s"
          f"  T={ph['period_s']:.2f} s  [{'ESTABLE' if ph['stable'] else 'INESTABLE'}]")

    if sp_eig:
        for l in _fmt_mode(sp_eig, "Short Period"): print(l)

    print("\n=== DYNAMIC STABILITY -- LATERAL-DIRECTIONAL ===")
    for key, label in [("dutch_roll","Dutch Roll"),("roll_mode","Roll Mode"),("spiral","Spiral Mode")]:
        for l in _fmt_mode(lt.get(key), label): print(l)

    print("=" * 64)


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    import sys
    from pathlib import Path

    # Crear carpeta plots/
    plots_dir = Path(__file__).parent / "plots"
    plots_dir.mkdir(exist_ok=True)

    cfg      = get_aircraft_config()
    analyzer = StabilityAnalyzer(cfg)
    results  = analyzer.run()

    print_stability_report(results)

    # Guardar matrices en texto para referencia
    np.savetxt(str(plots_dir / "A_lon.txt"), results["A_lon"], fmt="%+12.6f",
               header="A_lon [Du, Dw, Dq, Dtheta]")
    np.savetxt(str(plots_dir / "A_lat.txt"), results["A_lat"], fmt="%+12.6f",
               header="A_lat [Dbeta, Dp, Dr, Dphi]")

    print(f"\n  Matrices guardadas en {plots_dir}/")

    # Eigenvalores completos
    print("\nEigenvalores longitudinales:")
    for ev in results["lon_modes"]["eigenvalues"]:
        print(f"  lambda = {ev.real:+.5f}  {ev.imag:+.5f}j")

    print("\nEigenvalores laterales:")
    for ev in results["lat_modes"]["eigenvalues"]:
        print(f"  lambda = {ev.real:+.5f}  {ev.imag:+.5f}j")

    print("\n[OK] stability_analysis.py OK")
