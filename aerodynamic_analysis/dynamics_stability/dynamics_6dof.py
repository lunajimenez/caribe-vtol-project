"""
================================================================================
ECUACIONES DE MOVIMIENTO 6 DOF -- INTEGRADOR RK4
================================================================================
Implementa las ecuaciones de Bryan (1911) / Newton-Euler para un cuerpo rígido
en el espacio:

  TRASLACIÓN (body frame):
    u_dot = Fx/m + r*v - q*w
    v_dot = Fy/m - r*u + p*w
    _ = Fz/m + q*u - p*v

  ROTACIÓN (Euler, body frame):
    [Ixx  0  -Ixz] [_]   [L - (Iyy-Izz)*q*r - Ixz*p*q]
    [ 0  Iyy  0  ] [q_dot] = [M - (Izz-Ixx)*p*r - Ixz*(r2-p2)]
    [-Ixz 0   Izz] [_]   [N - (Ixx-Iyy)*p*q - Ixz*q*r]

  CINEMÁTICA (ángulos de Euler):
    __dot = p + (q*sin _ + r*cos _)*tan theta
    theta_dot = q*cos _ - r*sin _
    __dot = (q*sin _ + r*cos _) / cos theta

  NAVEGACIÓN (NED):
    [_, _, _]_NED = DCM(_,theta,_)_ * [u, v, w]_body

Vector de estado: y = [u, v, w, p, q, r, _, theta, _, x, y, z]  (12 estados)

Integrador: RK4 de paso fijo (robusto, sin overhead de control de paso).

Referencias:
  Bryan (1911) "Stability in Aviation"
  Etkin & Reid (1996) "Dynamics of Atmospheric Flight" Ch.4
  Nelson (1998) "Flight Stability and Automatic Control" Ch.3
================================================================================
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Callable, List, Tuple

from parameters import AircraftConfig, get_aircraft_config
from forces_moments import AeroModel, PropModel, airspeed_angles


# ============================================================================
# ESTADO Y MANDO
# ============================================================================

@dataclass
class State:
    """Vector de estado completo del VTOL (12 estados)."""
    # Velocidades translacionales en body frame [m/s]
    u: float = 0.0   # +X adelante
    v: float = 0.0   # +Y ala derecha
    w: float = 0.0   # +Z abajo

    # Velocidades angulares en body frame [rad/s]
    p: float = 0.0   # roll rate
    q: float = 0.0   # pitch rate
    r: float = 0.0   # yaw rate

    # Ángulos de Euler [rad]
    phi:   float = 0.0   # roll
    theta: float = 0.0   # pitch
    psi:   float = 0.0   # yaw

    # Posición en Earth frame NED [m]
    x_e: float = 0.0
    y_e: float = 0.0
    z_e: float = 0.0   # positivo hacia abajo

    def to_array(self) -> np.ndarray:
        return np.array([
            self.u, self.v, self.w,
            self.p, self.q, self.r,
            self.phi, self.theta, self.psi,
            self.x_e, self.y_e, self.z_e,
        ], dtype=float)

    @staticmethod
    def from_array(y: np.ndarray) -> "State":
        return State(
            u=y[0], v=y[1], w=y[2],
            p=y[3], q=y[4], r=y[5],
            phi=y[6], theta=y[7], psi=y[8],
            x_e=y[9], y_e=y[10], z_e=y[11],
        )

    @property
    def V(self) -> float:
        return float(np.sqrt(self.u**2 + self.v**2 + self.w**2))

    @property
    def alpha(self) -> float:
        return float(np.arctan2(self.w, self.u))

    @property
    def beta(self) -> float:
        V = max(self.V, 0.5)
        return float(np.arcsin(np.clip(self.v / V, -1.0, 1.0)))

    @property
    def altitude(self) -> float:
        return -self.z_e   # altitud = -z_NED


@dataclass
class Controls:
    """Mandos de vuelo."""
    delta_e:  float = 0.0    # elevador [rad]
    delta_a:  float = 0.0    # alerón   [rad]
    delta_r:  float = 0.0    # rudder   [rad]
    throttle: float = 0.0    # motor pusher [0-1]

    # Rotores VTOL (sólo en modos HOVER/TRANSICIÓN)
    throttle_vtol: float = 0.0  # [0-1]


# ============================================================================
# MATRIZ DCM Y CINEMÁTICA
# ============================================================================

def euler_dcm(phi: float, theta: float, psi: float) -> np.ndarray:
    """
    DCM body -> NED (R_bn).  NED_vec = DCM @ body_vec
    Ref: Etkin (1996) Eq. 4.1.3
    """
    cp, sp = np.cos(phi),   np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cs, ss = np.cos(psi),   np.sin(psi)
    return np.array([
        [ct*cs,   sp*st*cs - cp*ss,   cp*st*cs + sp*ss],
        [ct*ss,   sp*st*ss + cp*cs,   cp*st*ss - sp*cs],
        [-st,     sp*ct,              cp*ct            ],
    ])


# ============================================================================
# MOTOR DE DINÁMICAS 6 DOF
# ============================================================================

class Dynamics6DOF:
    """
    Motor de integración 6 DOF con RK4.

    Uso:
        dyn = Dynamics6DOF(cfg)
        states, times = dyn.simulate(x0, control_fn, t_end=30.0, dt=0.02)
    """

    def __init__(self, cfg: AircraftConfig):
        self.cfg   = cfg
        self.aero  = AeroModel(cfg)
        self.prop  = PropModel(cfg)

    # ------------------------------------------------------------------ #
    # DERIVADAS DE ESTADO dy/dt                                           #
    # ------------------------------------------------------------------ #

    def derivatives(self, y: np.ndarray, ctrl: Controls) -> np.ndarray:
        """
        Calcula _ = f(y, u) para el integrador RK4.

        Args:
            y:    vector de estado (12,)
            ctrl: Controls con mandos actuales

        Returns:
            dy/dt (12,)
        """
        s  = State.from_array(y)
        cfg = self.cfg
        m   = cfg.mass
        atm = cfg.atm

        u, v, w  = s.u, s.v, s.w
        p, q, r  = s.p, s.q, s.r
        phi, theta, psi = s.phi, s.theta, s.psi

        mass = m.m
        Ixx, Iyy, Izz, Ixz = m.Ixx, m.Iyy, m.Izz, m.Ixz

        # ---- Fuerzas y momentos aerodinámicos + gravedad ----
        T_pusher = self.prop.thrust_pusher(ctrl.throttle)

        F, M_vec = self.aero.forces_moments(
            u, v, w, p, q, r, phi, theta,
            delta_e=ctrl.delta_e,
            delta_a=ctrl.delta_a,
            delta_r=ctrl.delta_r,
            thrust=T_pusher,
        )

        # Añadir empuje VTOL (rotores en -Z body)
        T_vtol = self.prop.thrust_rotors(ctrl.throttle_vtol)
        F[2]  -= T_vtol    # -Z body = hacia arriba

        Fx, Fy, Fz = F
        L_mom, M_mom, N_mom = M_vec

        # ---- Ecuaciones traslacionales (Bryan 1911) ----
        u_dot = Fx / mass + r * v - q * w
        v_dot = Fy / mass - r * u + p * w
        w_dot = Fz / mass + q * u - p * v

        # ---- Ecuaciones rotacionales -- sistema 2x2 acoplado en (p,r) ----
        # [Ixx  -Ixz][_]   [L - (Iyy-Izz)*q*r - Ixz*p*q]
        # [-Ixz  Izz][_] = [N - (Ixx-Iyy)*p*q - Ixz*q*r]
        rhs1 = L_mom - (Iyy - Izz) * q * r - Ixz * p * q
        rhs3 = N_mom - (Ixx - Iyy) * p * q - Ixz * q * r
        det  = Ixx * Izz - Ixz**2
        p_dot = ( Izz * rhs1 + Ixz * rhs3) / det
        r_dot = ( Ixz * rhs1 + Ixx * rhs3) / det

        q_dot = (M_mom - (Izz - Ixx) * p * r - Ixz * (r**2 - p**2)) / Iyy

        # ---- Cinemática de Euler ----
        cos_theta = np.cos(theta)
        # Singularidad en theta = ±90°; proteger
        if abs(cos_theta) < 1e-4:
            cos_theta = np.sign(cos_theta) * 1e-4

        phi_dot   = p + (q * np.sin(phi) + r * np.cos(phi)) * np.tan(theta)
        theta_dot = q * np.cos(phi) - r * np.sin(phi)
        psi_dot   = (q * np.sin(phi) + r * np.cos(phi)) / cos_theta

        # ---- Posición NED ----
        DCM = euler_dcm(phi, theta, psi)
        vel_ned = DCM @ np.array([u, v, w])
        x_dot, y_dot, z_dot = vel_ned

        return np.array([
            u_dot, v_dot, w_dot,
            p_dot, q_dot, r_dot,
            phi_dot, theta_dot, psi_dot,
            x_dot, y_dot, z_dot,
        ])

    # ------------------------------------------------------------------ #
    # INTEGRADOR RK4 DE PASO FIJO                                         #
    # ------------------------------------------------------------------ #

    def _rk4_step(
        self,
        t:    float,
        y:    np.ndarray,
        ctrl: Controls,
        dt:   float,
        ctrl_fn: Callable,
    ) -> np.ndarray:
        """Un paso RK4. ctrl_fn(t, State) -> Controls."""
        k1 = self.derivatives(y,              ctrl_fn(t,        State.from_array(y)))
        k2 = self.derivatives(y + 0.5*dt*k1,  ctrl_fn(t+0.5*dt, State.from_array(y + 0.5*dt*k1)))
        k3 = self.derivatives(y + 0.5*dt*k2,  ctrl_fn(t+0.5*dt, State.from_array(y + 0.5*dt*k2)))
        k4 = self.derivatives(y + dt*k3,       ctrl_fn(t+dt,     State.from_array(y + dt*k3)))
        return y + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

    def simulate(
        self,
        x0:      State,
        ctrl_fn: Callable[[float, State], Controls],
        t_end:   float = 60.0,
        dt:      float = 0.02,
    ) -> Tuple[List[State], np.ndarray]:
        """
        Integra las ecuaciones 6 DOF con RK4 de paso fijo.

        Args:
            x0:      estado inicial
            ctrl_fn: función (t, State) -> Controls
            t_end:   tiempo final [s]
            dt:      paso de integración [s]

        Returns:
            (states_list, time_array)
        """
        times  = np.arange(0.0, t_end + dt, dt)
        y      = x0.to_array()
        states = [State.from_array(y)]

        for i, t in enumerate(times[:-1]):
            ctrl = ctrl_fn(t, State.from_array(y))
            y    = self._rk4_step(t, y, ctrl, dt, ctrl_fn)
            states.append(State.from_array(y))

        return states, times[:len(states)]


# ============================================================================
# LINEALIZACIÓN NUMÉRICA (para stability_analysis)
# ============================================================================

def numerical_jacobian(
    dyn:   Dynamics6DOF,
    y_eq:  np.ndarray,
    ctrl:  Controls,
    eps:   float = 1e-5,
) -> np.ndarray:
    """
    Jacobiano numérico A = _f/_y por diferencias centrales, evaluado en y_eq.

    A[i,j] = (f_i(y + eps*ej) - f_i(y - eps*ej)) / (2*eps)

    Returns:
        A (12x12)
    """
    n  = len(y_eq)
    f0 = dyn.derivatives(y_eq, ctrl)
    A  = np.zeros((n, n))
    for j in range(n):
        yp = y_eq.copy(); yp[j] += eps
        ym = y_eq.copy(); ym[j] -= eps
        A[:, j] = (dyn.derivatives(yp, ctrl) - dyn.derivatives(ym, ctrl)) / (2.0 * eps)
    return A


# ============================================================================
# MAIN -- prueba de integración básica
# ============================================================================

if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")

    cfg = get_aircraft_config()
    dyn = Dynamics6DOF(cfg)

    print("=" * 60)
    print(" DYNAMICS 6DOF -- TEST: VUELO RECTO Y NIVELADO 10 s")
    print("=" * 60)

    # Trim aproximado en crucero
    V0    = cfg.V_cruise
    alpha0 = np.radians(3.5)

    x0 = State(
        u=V0 * np.cos(alpha0),
        w=V0 * np.sin(alpha0),
        theta=alpha0,
    )

    def cruise_ctrl(t: float, s: State) -> Controls:
        # Proporciona empuje para equilibrar arrastre (~= 10-12% empuje)
        return Controls(throttle=0.45, delta_e=np.radians(-1.0))

    states, times = dyn.simulate(x0, cruise_ctrl, t_end=10.0, dt=0.02)

    s_final = states[-1]
    print(f"Estado final (t={times[-1]:.1f} s):")
    print(f"  u={s_final.u:.3f} m/s  w={s_final.w:.3f} m/s  V={s_final.V:.3f} m/s")
    print(f"  theta={np.degrees(s_final.theta):.2f}°  alpha={np.degrees(s_final.alpha):.2f}°")
    print(f"  Altitud: {s_final.altitude:.2f} m  (Deltaz desde inicio)")
    print("\n[OK] dynamics_6dof.py OK")
