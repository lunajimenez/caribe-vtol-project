"""
================================================================================
MOTOR DE DINÁMICAS 6 DOF - ECUACIONES DE MOVIMIENTO COMPLETAS
================================================================================
Implementa el sistema de ecuaciones diferenciales de un vehículo aéreo rígido:

ECUACIONES DE TRASLACIÓN (Newton's Second Law, Body Frame):
  m·du/dt = Fx - m·g·sin(θ) + q·m·w - r·m·v
  m·dv/dt = Fy + m·g·sin(φ)·cos(θ) - p·m·w + r·m·u
  m·dw/dt = Fz + m·g·cos(φ)·cos(θ) + p·m·v - q·m·u

donde (u, v, w) son velocidades en Body Frame, (Fx, Fy, Fz) fuerzas totales.

ECUACIONES DE ROTACIÓN (Euler's Equations, Body Frame):
  Ixx·dp/dt + Ixz·dr/dt = L - (Iyy-Izz)·q·r - Ixz·p·q
  Iyy·dq/dt = M - (Izz-Ixx)·p·r - Ixz·(p²-r²)
  Izz·dr/dt + Ixz·dp/dt = N - (Ixx-Iyy)·p·q - Ixz·q·r

donde (p, q, r) son velocidades angulares, (L, M, N) momentos externos.

CINEMÁTICA (Euler Angles):
  dφ/dt = p + q·sin(φ)·tan(θ) + r·cos(φ)·tan(θ)
  dθ/dt = q·cos(φ) - r·sin(φ)
  dψ/dt = q·sin(φ)/cos(θ) + r·cos(φ)/cos(θ)

NAVEGACIÓN (Position in Earth Frame):
  Transformación de velocidades: Body → Earth usando matriz de rotación.

================================================================================
Referencia: Etkin & Reid, "Dynamics of Atmospheric Flight", 1996
            Nelson, "Flight Stability and Automatic Control", 2nd Ed., 1998
================================================================================
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, List, Callable
from scipy.integrate import odeint, solve_ivp
from parameters import VTOLAircraftParameters
from forces_moments import AerodynamicCalculator, ThrustCalculator


@dataclass
class AircraftState:
    """
    Estado completo del VTOL en tiempo t.
    
    Posición y velocidad (Earth/Body Frame):
      - Posición (x, y, z) en Earth Frame NED
      - Velocidad (u, v, w) en Body Frame
    
    Actitud (Euler angles):
      - φ (phi): roll angle
      - θ (theta): pitch angle
      - ψ (psi): yaw angle
    
    Velocidades angulares (Body Frame):
      - p: roll rate (alrededor eje X)
      - q: pitch rate (alrededor eje Y)
      - r: yaw rate (alrededor eje Z)
    """
    
    # Posición en Earth Frame (metros)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    # Velocidad en Body Frame (m/s)
    u: float = 0.0
    v: float = 0.0
    w: float = 0.0
    
    # Actitud - Ángulos de Euler (radianes)
    phi: float = 0.0  # Roll
    theta: float = 0.0  # Pitch
    psi: float = 0.0  # Yaw
    
    # Velocidades angulares en Body Frame (rad/s)
    p: float = 0.0  # Roll rate
    q: float = 0.0  # Pitch rate
    r: float = 0.0  # Yaw rate
    
    # Tiempo de simulación
    t: float = 0.0
    
    def to_array(self) -> np.ndarray:
        """Convierte estado a array [x, y, z, u, v, w, φ, θ, ψ, p, q, r]."""
        return np.array([
            self.x, self.y, self.z,
            self.u, self.v, self.w,
            self.phi, self.theta, self.psi,
            self.p, self.q, self.r
        ], dtype=float)
    
    @staticmethod
    def from_array(state: np.ndarray, t: float = 0.0) -> 'AircraftState':
        """Crea AircraftState desde array."""
        return AircraftState(
            x=state[0], y=state[1], z=state[2],
            u=state[3], v=state[4], w=state[5],
            phi=state[6], theta=state[7], psi=state[8],
            p=state[9], q=state[10], r=state[11],
            t=t
        )


@dataclass
class ControlInput:
    """Comandos de control del piloto/autopiloto."""
    
    # Throttle (0.0 = idle, 1.0 = máximo)
    throttle_vertical: float = 0.5  # Comando para rotores verticales
    throttle_horizontal: float = 0.5  # Comando para motor horizontal
    
    # Control de superficies (radianes)
    delta_e: float = 0.0  # Elevador (pitch control)
    delta_a: float = 0.0  # Alerones (roll control)
    delta_r: float = 0.0  # Timón (yaw control)
    
    # Control diferencial de rotores (modo VTOL)
    roll_cmd: float = 0.0  # [-1, 1]
    pitch_cmd: float = 0.0  # [-1, 1]
    yaw_cmd: float = 0.0  # [-1, 1]
    
    # Modo de operación
    mode: str = "HOVER"  # "HOVER", "TRANSITION", "CRUISE"


class VTOLDynamicsEngine:
    """
    Motor principal de dinámicas VTOL.
    
    Integra ecuaciones de movimiento 6 DOF usando RK45 y proporciona
    análisis de estabilidad.
    
    Flujo principal:
    1. Leer estado (x, y, z, u, v, w, φ, θ, ψ, p, q, r)
    2. Calcular fuerzas/momentos aerodinámicos + propulsión
    3. Integrar ecuaciones diferenciales
    4. Retornar nuevo estado
    """
    
    def __init__(self, params: VTOLAircraftParameters):
        self.params = params
        self.geom = params.geometry
        self.inertia = params.inertia
        self.flight = params.flight
        
        # Calculadores
        self.aero_calc = AerodynamicCalculator(params)
        self.thrust_calc = ThrustCalculator(params)
        
        # Historia de simulación
        self.history: List[AircraftState] = []
        self.time_history: List[float] = []
    
    # ========================================================================
    # ECUACIONES DE MOVIMIENTO (TRASLACIÓN Y ROTACIÓN)
    # ========================================================================
    
    def _derivatives(self, t: float, y: np.ndarray, control: ControlInput) -> np.ndarray:
        """
        Calcula derivadas de estado: dy/dt
        
        Implementa el sistema completo de ecuaciones diferenciales:
          dx/dt, dy/dt, dz/dt: Integración de velocidades
          du/dt, dv/dt, dw/dt: Ecuaciones de traslación Newton
          dφ/dt, dθ/dt, dψ/dt: Cinemática de Euler
          dp/dt, dq/dt, dr/dt: Ecuaciones de rotación Euler
        
        Args:
          t: tiempo actual (s)
          y: estado [x, y, z, u, v, w, φ, θ, ψ, p, q, r]
          control: ControlInput con comandos
        
        Returns:
          dy/dt en Body Frame + transformaciones
        """
        # Desempacar estado
        state = AircraftState.from_array(y, t)
        x, y_pos, z = state.x, state.y, state.z
        u, v, w = state.u, state.v, state.w
        phi, theta, psi = state.phi, state.theta, state.psi
        p, q, r = state.p, state.q, state.r
        
        # Parámetros inerciales
        m = self.geom.mtow
        g = self.flight.g
        Ixx = self.inertia.Ixx
        Iyy = self.inertia.Iyy
        Izz = self.inertia.Izz
        Ixz = self.inertia.Ixz
        
        # ====================================================================
        # PASO 1: CALCULAR FUERZAS Y MOMENTOS
        # ====================================================================
        
        # Gravedad (Body Frame)
        Fx_g = -m * g * np.sin(theta)
        Fy_g = m * g * np.sin(phi) * np.cos(theta)
        Fz_g = m * g * np.cos(phi) * np.cos(theta)
        
        # Fuerzas aerodinámicas + propulsión según modo
        if control.mode == "HOVER":
            # Hover: Rotores verticales dominan
            Fx_h, Fy_h, Fz_h = self.thrust_calc.compute_hover_forces(
                control.throttle_vertical
            )
            Fx_a, Fy_a, Fz_a, alpha, beta, V = \
                self.aero_calc.compute_aerodynamic_forces(u, v, w, p, q, r)
            
            # Momentos en hover (control diferencial)
            _, _, _, L_h, M_h, N_h = \
                self.thrust_calc.compute_transition_forces_moments(
                    control.throttle_vertical, control.throttle_horizontal,
                    control.roll_cmd, control.pitch_cmd, control.yaw_cmd
                )
            L_a, M_a, N_a, _, _, _ = \
                self.aero_calc.compute_aerodynamic_moments(
                    u, v, w, p, q, r, control.delta_e,
                    control.delta_a, control.delta_r
                )
        
        elif control.mode == "TRANSITION":
            # Transición: Mezcla de empujes
            Fx_h, Fy_h, Fz_h, L_h, M_h, N_h = \
                self.thrust_calc.compute_transition_forces_moments(
                    control.throttle_vertical, control.throttle_horizontal,
                    control.roll_cmd, control.pitch_cmd, control.yaw_cmd
                )
            Fx_a, Fy_a, Fz_a, alpha, beta, V = \
                self.aero_calc.compute_aerodynamic_forces(
                    u, v, w, p, q, r, control.delta_e,
                    control.delta_a, control.delta_r
                )
            L_a, M_a, N_a, _, _, _ = \
                self.aero_calc.compute_aerodynamic_moments(
                    u, v, w, p, q, r, control.delta_e,
                    control.delta_a, control.delta_r
                )
        
        else:  # CRUISE
            # Crucero: Ala fija dominante
            Fx_h, _, Fz_h, _, _, _ = \
                self.thrust_calc.compute_cruise_forces_moments(
                    control.throttle_horizontal, control.throttle_vertical
                )
            Fx_a, Fy_a, Fz_a, alpha, beta, V = \
                self.aero_calc.compute_aerodynamic_forces(
                    u, v, w, p, q, r, control.delta_e,
                    control.delta_a, control.delta_r
                )
            L_a, M_a, N_a, _, _, _ = \
                self.aero_calc.compute_aerodynamic_moments(
                    u, v, w, p, q, r, control.delta_e,
                    control.delta_a, control.delta_r
                )
            Fy_h = 0.0
            L_h = M_h = N_h = 0.0
        
        # Fuerzas totales en Body Frame
        Fx_total = Fx_h + Fx_a + Fx_g
        Fy_total = Fy_h + Fy_a + Fy_g
        Fz_total = Fz_h + Fz_a + Fz_g
        
        # Momentos totales
        L_total = L_h + L_a
        M_total = M_h + M_a
        N_total = N_h + N_a
        
        # ====================================================================
        # PASO 2: ECUACIONES DE TRASLACIÓN (Newton, Body Frame)
        # ====================================================================
        # Aceleraciones lineales
        u_dot = (Fx_total / m) + r * v - q * w
        v_dot = (Fy_total / m) + p * w - r * u
        w_dot = (Fz_total / m) + q * u - p * v
        
        # ====================================================================
        # PASO 3: CINEMÁTICA DE EULER (Actitud)
        # ====================================================================
        # Transformar velocidades angulares a derivadas de Euler
        phi_dot = p + q * np.sin(phi) * np.tan(theta) + \
                  r * np.cos(phi) * np.tan(theta)
        
        theta_dot = q * np.cos(phi) - r * np.sin(phi)
        
        # Protección contra singularidad en theta = ±π/2
        if np.abs(np.cos(theta)) > 1e-6:
            psi_dot = (q * np.sin(phi) + r * np.cos(phi)) / np.cos(theta)
        else:
            psi_dot = 0.0
        
        # ====================================================================
        # PASO 4: NAVEGACIÓN (Posición en Earth Frame)
        # ====================================================================
        # Matriz de transformación Body → Earth (NED)
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)
        
        # Rotación Body → Earth
        x_dot = (c_theta * c_psi) * u + \
                (s_phi * s_theta * c_psi - c_phi * s_psi) * v + \
                (c_phi * s_theta * c_psi + s_phi * s_psi) * w
        
        y_dot = (c_theta * s_psi) * u + \
                (s_phi * s_theta * s_psi + c_phi * c_psi) * v + \
                (c_phi * s_theta * s_psi - s_phi * c_psi) * w
        
        z_dot = -s_theta * u + s_phi * c_theta * v + c_phi * c_theta * w
        
        # ====================================================================
        # PASO 5: ECUACIONES DE ROTACIÓN (Euler, Body Frame)
        # ====================================================================
        # Matrices para resolver sistema acoplado
        # [Ixx  0  -Ixz] [p_dot]   [L - (Iyy-Izz)qr - Ixz(pq)]
        # [ 0  Iyy  0  ] [q_dot] = [M - (Izz-Ixx)pr - Ixz(p²-r²)]
        # [-Ixz  0  Izz] [r_dot]   [N - (Ixx-Iyy)pq - Ixz(qr)]
        
        a11 = Ixx
        a13 = -Ixz
        a22 = Iyy
        a31 = -Ixz
        a33 = Izz
        
        # RHS
        b1 = L_total - (Iyy - Izz) * q * r - Ixz * p * q
        b2 = M_total - (Izz - Ixx) * p * r - Ixz * (p**2 - r**2)
        b3 = N_total - (Ixx - Iyy) * p * q - Ixz * q * r
        
        # Resolver sistema 3x3
        A_mat = np.array([
            [a11, 0, a13],
            [0, a22, 0],
            [a31, 0, a33]
        ])
        
        b_vec = np.array([b1, b2, b3])
        
        try:
            rates = np.linalg.solve(A_mat, b_vec)
            p_dot, q_dot, r_dot = rates[0], rates[1], rates[2]
        except np.linalg.LinAlgError:
            # Fallback si matriz singular (improbable)
            p_dot = (L_total - (Iyy - Izz) * q * r) / Ixx
            q_dot = M_total / Iyy
            r_dot = (N_total - (Ixx - Iyy) * p * q) / Izz
        
        # ====================================================================
        # RETORNAR DERIVADAS
        # ====================================================================
        dy_dt = np.array([
            x_dot, y_dot, z_dot,
            u_dot, v_dot, w_dot,
            phi_dot, theta_dot, psi_dot,
            p_dot, q_dot, r_dot
        ], dtype=float)
        
        return dy_dt
    
    # ========================================================================
    # INTEGRACIÓN Y SIMULACIÓN
    # ========================================================================
    
    def simulate(self, initial_state: AircraftState,
                control_func: Callable[[float, AircraftState], ControlInput],
                t_end: float = 60.0, dt: float = 0.01,
                method: str = "RK45") -> Tuple[List[AircraftState], np.ndarray]:
        """
        Simula dinámicas VTOL sobre horizonte de tiempo.
        
        Args:
          initial_state: AircraftState inicial
          control_func: Función que retorna ControlInput en función de (t, state)
          t_end: tiempo final de simulación (s)
          dt: paso de tiempo para salida (s)
          method: método de integración ("RK45" recomendado)
        
        Returns:
          (states_list, time_array)
        """
        y0 = initial_state.to_array()
        t_span = (0.0, t_end)
        t_eval = np.arange(0.0, t_end, dt)
        
        # Wrapper para incluir control
        def derivatives_wrapper(t, y):
            state = AircraftState.from_array(y, t)
            control = control_func(t, state)
            return self._derivatives(t, y, control)
        
        # Integrar
        sol = solve_ivp(
            derivatives_wrapper,
            t_span,
            y0,
            method=method,
            t_eval=t_eval,
            dense_output=True,
            max_step=dt * 10,
            rtol=1e-8,
            atol=1e-10
        )
        
        # Convertir a lista de estados
        states = []
        for i, t in enumerate(sol.t):
            state = AircraftState.from_array(sol.y[:, i], t)
            states.append(state)
        
        self.history = states
        self.time_history = list(sol.t)
        
        return states, sol.t
    
    def get_history(self):
        """Retorna historia de simulación."""
        return self.history, np.array(self.time_history)


if __name__ == "__main__":
    from parameters import get_default_vtol_parameters
    
    print("\n" + "="*70)
    print("TEST: MOTOR 6 DOF VTOL")
    print("="*70)
    
    params = get_default_vtol_parameters()
    engine = VTOLDynamicsEngine(params)
    
    # Estado inicial: hover a cierta altitud
    initial_state = AircraftState(
        z=0.0,  # 0 m sobre suelo
        theta=0.0,  # Trimado para hover
        u=0.0,  # Estacionario
        v=0.0,
        w=0.0
    )
    
    # Control: hover constante
    def hover_control(t: float, state: AircraftState) -> ControlInput:
        return ControlInput(
            throttle_vertical=0.75,  # Enough to hover + margen
            throttle_horizontal=0.0,
            mode="HOVER"
        )
    
    # Simular 10 segundos
    print("Simulando 10 segundos en HOVER...")
    states, times = engine.simulate(initial_state, hover_control, t_end=10.0, dt=0.1)
    
    # Estadísticas
    final_state = states[-1]
    print(f"\nResultados finales:")
    print(f"  Posición: z = {-final_state.z:.2f} m (profundidad)")
    print(f"  Velocidad: u={final_state.u:.3f}, v={final_state.v:.3f}, w={final_state.w:.3f} m/s")
    print(f"  Actitud: φ={np.degrees(final_state.phi):.2f}°, θ={np.degrees(final_state.theta):.2f}°, ψ={np.degrees(final_state.psi):.2f}°")
    print(f"  Vel. Angular: p={final_state.p:.4f}, q={final_state.q:.4f}, r={final_state.r:.4f} rad/s")
    
    print("\n✓ Simulación completada")
