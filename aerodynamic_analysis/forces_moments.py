"""
================================================================================
FUERZAS Y MOMENTOS AERODINÁMICOS - ECUACIONES DE ESTABILIDAD DINÁMICA
================================================================================
Implementación de las ecuaciones clásicas de:
  1. Bryan (1911) - "Stability in Aviation"
  2. Abzug & Larrabee (2005) - "Airplane Stability and Control"
  3. Etkin & Reid (1996) - "Dynamics of Atmospheric Flight"

Estado de entrada (Body Frame):
  - Posición: [x, y, z]  (m, Earth Frame)
  - Velocidad body: [u, v, w]  (m/s, Body Frame)
  - Actitud: [φ, θ, ψ]  (rad, Euler angles)
  - Velocidad angular: [p, q, r]  (rad/s, Body Frame)

Salidas:
  - Fuerzas: [Fx, Fy, Fz] (N, Body Frame)
  - Momentos: [Lx, Mx, Nz] (N·m, Body Frame)

================================================================================
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple
from parameters import VTOLAircraftParameters, FlightConditions


class AerodynamicCalculator:
    """
    Calcula coeficientes aerodinámicos y fuerzas/momentos.
    
    Basado en Small Perturbation Theory (SPT):
      CX = CX0 + CX_α·α + CX_q·q̄ + CX_δe·δe + ...
    
    donde q̄ = q·c̄/(2V) es la velocidad angular adimensional.
    """
    
    def __init__(self, params: VTOLAircraftParameters):
        self.params = params
        self.aero = params.aero
        self.geom = params.geometry
        self.flight = params.flight
    
    # ========================================================================
    # SECCIÓN 1: COEFICIENTES AERODINÁMICOS BASE
    # ========================================================================
    
    def compute_alpha_beta(self, u: float, v: float, w: float) -> Tuple[float, float, float]:
        """
        Calcula ángulos aerodinámicos a partir de velocidades en Body Frame.
        
        Siguiendo convención de estabilidad dinámica (Abzug Ch. 2):
          V_airspeed = sqrt(u² + v² + w²)
          α = arctan(w/u)  [ángulo de ataque, pitch]
          β = arcsin(v/V)  [ángulo de resbalamiento lateral, yaw]
        
        Args:
          u, v, w: componentes de velocidad (m/s) en Body Frame
        
        Returns:
          (alpha, beta, V_airspeed)
        """
        V_airspeed = np.sqrt(u**2 + v**2 + w**2)
        
        # Protección contra división por cero
        if V_airspeed < 0.1:
            V_airspeed = 0.1
        
        # Ángulo de ataque (α): rotación alrededor eje Y
        alpha = np.arctan2(w, u)
        
        # Ángulo de resbalamiento (β): rotación alrededor eje Z
        beta = np.arcsin(np.clip(v / V_airspeed, -1, 1))
        
        return alpha, beta, V_airspeed
    
    def compute_cl(self, alpha: float, q_bar: float, delta_e: float) -> float:
        """
        Coeficiente de sustentación (lift coefficient).
        
        Ecuación fundamental (Small Perturbation Theory):
          CL = CL0 + CL_α·α + CL_q·q̄ + CL_δe·δe
        
        donde:
          - CL_α = dCL/dα (sustentación por ángulo ataque)
          - CL_q = dCL/d(q·c̄/2V) (sustentación por pitch rate)
          - CL_δe = dCL/dδ_e (sustentación por control elevador)
        
        Nota: En configuración VTOL mode (hover), CL se calcula pero es
        dominado por thrust de rotores, no alas.
        """
        CL = (self.aero.CL0 + 
              self.aero.CLalpha * alpha +
              self.aero.Cm_q * q_bar +  # Acoplamiento pitch-lift (pequeño)
              self.aero.Cm_delta_e * delta_e)  # Control elevador
        
        # Limitar a rango físico
        CL = np.clip(CL, -1.0, self.aero.CL_max)
        return CL
    
    def compute_cd(self, CL: float) -> float:
        """
        Coeficiente de arrastre - Polar parabólico.
        
        Ecuación (Abzug, Ch. 3):
          CD = CD0 + K·CL²
        
        donde K = 1/(π·e·AR) es el factor de arrastre inducido.
        
        En modo HOVER:
          - CD se calcula pero es pequeño comparado con thrust de rotores
          - Arrastre de hélices estacionarias ~30% adicional (Westcott, 2023)
        """
        K = 1.0 / (np.pi * self.aero.oswald_efficiency * self.geom.aspect_ratio)
        CD = self.aero.CD0 + K * CL**2
        return CD
    
    def compute_cm(self, alpha: float, q_bar: float, alpha_dot: float, 
                   delta_e: float) -> float:
        """
        Coeficiente de momento de pitch - CRÍTICO PARA ESTABILIDAD.
        
        Ecuación completa (Bryan, 1911; Abzug, Ch. 4):
          Cm = Cm0 + Cm_α·α + Cm_q·q̄ + Cm_α̇·ᾱ + Cm_δe·δe
        
        donde:
          - Cm_α: estabilidad estática longitudinal (DEBE SER NEGATIVO)
          - Cm_q: amortiguamiento de pitch (DEBE SER NEGATIVO)
          - Cm_α̇: amortiguamiento inducido por cambio de α (negativo)
          - Cm_δe: control de elevador (negativo = pitch down para deflexión positiva)
        
        La estabilidad depende críticamente del signo de Cm_α.
        Si Cm_α > 0: INESTABLE (divergencia espiral)
        Si Cm_α < 0: ESTABLE (oscilación amortiguada)
        
        Nota: En modo HOVER, elevador tiene efecto limitado.
        """
        Cm = (self.aero.Cm0 + 
              self.aero.Cm_alpha * alpha +
              self.aero.Cm_q * q_bar +
              self.aero.Cm_alphadot * alpha_dot +
              self.aero.Cm_delta_e * delta_e)
        
        return Cm
    
    def compute_cl_lateral(self, beta: float, p_bar: float, r_bar: float,
                          delta_a: float, delta_r: float) -> float:
        """
        Coeficiente de momento de roll - estabilidad lateral.
        
        Ecuación (Abzug, Ch. 5):
          Cl = Cl_β·β + Cl_p·p̄ + Cl_r·r̄ + Cl_δa·δa + Cl_δr·δr
        
        donde:
          - Cl_β: estabilidad de diedro (DEBE SER NEGATIVO para estable)
          - Cl_p: amortiguamiento roll (DEBE SER NEGATIVO)
          - Cl_r: acoplamiento yaw→roll (positivo típicamente)
          - Cl_δa: control de alerones (positivo)
        
        Cl_β < 0 significa que un resbalamiento derecho (β > 0) genera
        un momento que reduce el resbalamiento (roll estable).
        """
        Cl = (self.aero.Cl_beta * beta +
              self.aero.Cl_p * p_bar +
              self.aero.Cl_r * r_bar +
              self.aero.Cl_delta_a * delta_a +
              0.0)  # delta_r tiene efecto pequeño en roll
        
        return Cl
    
    def compute_cn_lateral(self, beta: float, p_bar: float, r_bar: float,
                          delta_a: float, delta_r: float) -> float:
        """
        Coeficiente de momento de yaw - estabilidad direccional.
        
        Ecuación (Bryan, 1911):
          Cn = Cn_β·β + Cn_p·p̄ + Cn_r·r̄ + Cn_δa·δa + Cn_δr·δr
        
        donde:
          - Cn_β: estabilidad direccional (DEBE SER POSITIVO)
          - Cn_r: amortiguamiento yaw (DEBE SER NEGATIVO)
          - Cn_δr: control de timón (negativo)
        
        Cn_β > 0 significa que un resbalamiento derecho genera un momento
        de yaw que restaura alineación (weathercock effect = estable).
        """
        Cn = (self.aero.Cn_beta * beta +
              self.aero.Cn_p * p_bar +
              self.aero.Cn_r * r_bar +
              0.0 +  # delta_a
              self.aero.Cn_delta_r * delta_r)
        
        return Cn
    
    def compute_cy_lateral(self, beta: float, delta_a: float, 
                          delta_r: float) -> float:
        """
        Coeficiente de fuerza lateral.
        
        Ecuación:
          CY = CY_β·β + CY_p·p̄ + CY_r·r̄ + CY_δa·δa + CY_δr·δr
        
        Típicamente pequeño en alas altas.
        """
        CY = (self.aero.CY_beta * beta +
              0.0 +  # CY_p negligible
              0.0 +  # CY_r negligible
              self.aero.CY_delta_a * delta_a +
              self.aero.CY_delta_r * delta_r)
        
        return CY
    
    # ========================================================================
    # SECCIÓN 2: CÁLCULO DE FUERZAS AERODINÁMICAS (BODY FRAME)
    # ========================================================================
    
    def compute_aerodynamic_forces(self, u: float, v: float, w: float,
                                   p: float, q: float, r: float,
                                   delta_e: float = 0.0, 
                                   delta_a: float = 0.0,
                                   delta_r: float = 0.0) -> Tuple[float, float, float]:
        """
        Calcula fuerzas aerodinámicas completas en Body Frame.
        
        Retorna: (Fx_aero, Fy_aero, Fz_aero) en Newton
        
        Marco de referencia Body:
          +X: forward (nosecone)
          +Y: right wing
          +Z: down
        
        En VTOL con empuje vertical:
          Fx = -D·cos(α) - L·sin(α)  [arrastre + componente α de lift]
          Fy = fuerzas laterales (CY)
          Fz = -L·cos(α) - D·sin(α)  [sustentación vertical]
        """
        # Calcular ángulos aerodinámicos
        alpha, beta, V_airspeed = self.compute_alpha_beta(u, v, w)
        
        # Parámetros adimensionales para derivadas dinámicas
        c_bar = self.geom.wing_chord_mean
        b = self.geom.wing_span
        S = self.geom.wing_area
        
        # Velocidades angulares adimensionales
        q_bar = q * c_bar / (2.0 * V_airspeed) if V_airspeed > 0.1 else 0.0
        p_bar = p * b / (2.0 * V_airspeed) if V_airspeed > 0.1 else 0.0
        r_bar = r * b / (2.0 * V_airspeed) if V_airspeed > 0.1 else 0.0
        alpha_dot = np.arctan2(w, u) - np.arctan2(w, u)  # Simplificado
        
        # Presión dinámica
        q_dynamic = 0.5 * self.flight.rho * V_airspeed**2
        
        # Coeficientes aerodinámicos
        CL = self.compute_cl(alpha, q_bar, delta_e)
        CD = self.compute_cd(CL)
        CY = self.compute_cy_lateral(beta, delta_a, delta_r)
        
        # Fuerzas aerodinámicas
        # Nota: L = sustentación perpendicular a V, D = arrastre paralelo a V
        L = q_dynamic * S * CL
        D = q_dynamic * S * CD
        Y = q_dynamic * S * CY
        
        # Transformar al Body Frame
        # Sustentación y arrastre actúan en plano de simetría (plano vertical)
        Fx_aero = -D * np.cos(alpha) - L * np.sin(alpha)
        Fy_aero = Y
        Fz_aero = -D * np.sin(alpha) + L * np.cos(alpha)
        
        return Fx_aero, Fy_aero, Fz_aero, alpha, beta, V_airspeed
    
    # ========================================================================
    # SECCIÓN 3: CÁLCULO DE MOMENTOS AERODINÁMICOS
    # ========================================================================
    
    def compute_aerodynamic_moments(self, u: float, v: float, w: float,
                                    p: float, q: float, r: float,
                                    delta_e: float = 0.0,
                                    delta_a: float = 0.0,
                                    delta_r: float = 0.0) -> Tuple[float, float, float]:
        """
        Calcula momentos aerodinámicos completos (Body Frame).
        
        Retorna: (L_aero, M_aero, N_aero) en N·m
        
        Momentos en convención estándar:
          L (roll):  +X positivo = ala derecha abajo
          M (pitch): +Y positivo = morro arriba
          N (yaw):   +Z positivo = morro a la izquierda
        """
        # Ángulos aerodinámicos
        alpha, beta, V_airspeed = self.compute_alpha_beta(u, v, w)
        
        # Parámetros adimensionales
        c_bar = self.geom.wing_chord_mean
        b = self.geom.wing_span
        S = self.geom.wing_area
        
        q_bar = q * c_bar / (2.0 * V_airspeed) if V_airspeed > 0.1 else 0.0
        p_bar = p * b / (2.0 * V_airspeed) if V_airspeed > 0.1 else 0.0
        r_bar = r * b / (2.0 * V_airspeed) if V_airspeed > 0.1 else 0.0
        alpha_dot = q * np.sin(alpha)  # dα/dt aproximado
        
        # Presión dinámica
        q_dynamic = 0.5 * self.flight.rho * V_airspeed**2
        
        # Coeficientes de momento
        Cm = self.compute_cm(alpha, q_bar, alpha_dot, delta_e)
        Cl = self.compute_cl_lateral(beta, p_bar, r_bar, delta_a, delta_r)
        Cn = self.compute_cn_lateral(beta, p_bar, r_bar, delta_a, delta_r)
        
        # Momentos aerodinámicos
        L_aero = q_dynamic * S * b * Cl  # Roll moment
        M_aero = q_dynamic * S * c_bar * Cm  # Pitch moment
        N_aero = q_dynamic * S * b * Cn  # Yaw moment
        
        return L_aero, M_aero, N_aero, alpha, beta, V_airspeed


class ThrustCalculator:
    """
    Calcula empujes y momentos inducidos por hélices.
    
    Modos:
    1. HOVER: 4 rotores verticales balanceados, motor horizontal idle
    2. TRANSICIÓN: mezcla progresiva 0%-100%
    3. CRUCERO: motor horizontal activo, rotores en idle/propulsión mínima
    """
    
    def __init__(self, params: VTOLAircraftParameters):
        self.params = params
        self.geom = params.geometry
        self.prop = params.propulsion
    
    def compute_rotor_thrust(self, throttle: float) -> float:
        """
        Empuje de un rotor vertical individual.
        
        Args:
          throttle: 0.0 (apagado) a 1.0 (máximo)
        
        Returns:
          Thrust en Newtons
        """
        thrust_kg = self.prop.vertical_motor_max_thrust_kg * (throttle**2)  # Quadrático típico
        return thrust_kg * 9.81  # Convertir a N
    
    def compute_hover_forces(self, throttle: float) -> Tuple[float, float, float]:
        """
        Fuerzas en modo HOVER (multirrotor puro).
        
        4 rotores verticales en configuración X:
          - Motor 1: +X, +Y  (frente-derecha)
          - Motor 2: -X, +Y  (atrás-derecha)
          - Motor 3: -X, -Y  (atrás-izquierda)
          - Motor 4: +X, -Y  (frente-izquierda)
        
        En vuelo estacionario balanceado:
          ΣT_z = 4·T_rotor = W (para equilibrio)
        """
        T_rotor = self.compute_rotor_thrust(throttle)
        
        # Fuerzas en Body Frame (Z negativo = arriba en Body Frame)
        Fz = -4 * T_rotor  # 4 rotores generan empuje hacia arriba
        Fx = 0.0
        Fy = 0.0
        
        return Fx, Fy, Fz
    
    def compute_horizontal_thrust(self, throttle_h: float) -> Tuple[float, float, float]:
        """
        Empuje del motor horizontal (pusher configuration).
        
        En Body Frame pusher:
          +X = forward, motor empuja hacia atrás negativo (drag reduction)
          Configuración: motor en eje X, genera Fx negativo
        """
        T_h = self.prop.horizontal_motor_max_thrust_kg * (throttle_h**2)
        T_h_N = T_h * 9.81
        
        # Pusher: empuje en -X (hacia atrás reduce drag aparente)
        if self.geom.horizontal_motor_type == "pusher":
            Fx_h = -T_h_N  # Negativo = empuje hacia atrás
        else:  # tractor
            Fx_h = T_h_N
        
        return Fx_h, 0.0, 0.0
    
    def compute_transition_forces_moments(self, throttle_v: float, throttle_h: float,
                                         roll_cmd: float = 0.0,
                                         pitch_cmd: float = 0.0,
                                         yaw_cmd: float = 0.0) -> Tuple:
        """
        Fuerzas y momentos durante TRANSICIÓN.
        
        Control diferencial de rotores permite generar momentos:
          - ROLL: diferenciar T1, T4 vs T2, T3
          - PITCH: diferenciar T1, T2 vs T3, T4
          - YAW: diferenciar rotor CW vs CCW
        
        Args:
          throttle_v: comando de throttle vertical [0, 1]
          throttle_h: comando de throttle horizontal [0, 1]
          roll_cmd: comando de roll [-1, 1]
          pitch_cmd: comando de pitch [-1, 1]
          yaw_cmd: comando de yaw [-1, 1]
        
        Returns:
          (Fx, Fy, Fz, L, M, N)
        """
        # Empuje vertical base
        T_base = self.compute_rotor_thrust(throttle_v)
        
        # Diferenciales para control
        T_diff_roll = T_base * roll_cmd * 0.2  # 20% de variación
        T_diff_pitch = T_base * pitch_cmd * 0.2
        
        # Asignación a rotores (configuración X)
        T1 = T_base - T_diff_pitch - T_diff_roll  # frente-derecha
        T2 = T_base - T_diff_pitch + T_diff_roll  # atrás-derecha
        T3 = T_base + T_diff_pitch + T_diff_roll  # atrás-izquierda
        T4 = T_base + T_diff_pitch - T_diff_roll  # frente-izquierda
        
        # Fuerzas
        Fz = -(T1 + T2 + T3 + T4)
        Fx_h, _, _ = self.compute_horizontal_thrust(throttle_h)
        Fx = Fx_h
        Fy = 0.0
        
        # Momentos inducidos por rotores
        arm = self.geom.vertical_motor_arm  # Brazo desde CG
        
        # Roll moment (diferencia lado izquierdo vs derecho)
        L = arm * ((T1 + T4) - (T2 + T3))
        
        # Pitch moment (diferencia frente vs atrás)
        M = arm * ((T1 + T2) - (T3 + T4))
        
        # Yaw moment (reacción de par + control diferencial)
        # Aproximado: proporcional a yaw_cmd
        N = T_base * yaw_cmd * 0.1 * arm
        
        return Fx, Fy, Fz, L, M, N
    
    def compute_cruise_forces_moments(self, throttle_h: float, 
                                     throttle_v: float = 0.05) -> Tuple:
        """
        Fuerzas en modo CRUCERO (ala fija dominante).
        
        En crucero, aerodinámicas dominan. Rotores en idle (~5% throttle)
        para margen de control vertical.
        
        Args:
          throttle_h: throttle motor horizontal [0, 1]
          throttle_v: throttle vertical mínimo (margen de seguridad)
        
        Returns:
          (Fx, Fy, Fz, L, M, N)
        """
        # Empuje horizontal dominante
        Fx_h, _, _ = self.compute_horizontal_thrust(throttle_h)
        
        # Empuje vertical mínimo (margen)
        T_min = self.compute_rotor_thrust(throttle_v)
        Fz_min = -4 * T_min
        
        # En crucero, aerodinámicas generan mayor parte de fuerzas
        # Empuje vertical contribuye solo para control
        Fz = Fz_min
        
        return Fx_h, 0.0, Fz, 0.0, 0.0, 0.0


if __name__ == "__main__":
    from parameters import get_default_vtol_parameters
    
    params = get_default_vtol_parameters()
    calc_aero = AerodynamicCalculator(params)
    calc_thrust = ThrustCalculator(params)
    
    # Test 1: Fuerzas aerodinámicas en crucero
    print("\n" + "="*70)
    print("TEST 1: FUERZAS AERODINÁMICAS EN CRUCERO")
    print("="*70)
    V_cruise = params.flight.v_cruise_ms
    Fx, Fy, Fz, α, β, V = calc_aero.compute_aerodynamic_forces(
        u=V_cruise, v=0.0, w=0.0, p=0, q=0, r=0, delta_e=0.0
    )
    print(f"Velocidad: {V:.2f} m/s | α={np.degrees(α):.2f}° | β={np.degrees(β):.2f}°")
    print(f"Fuerzas aero: Fx={Fx:.2f} N, Fy={Fy:.2f} N, Fz={Fz:.2f} N")
    
    # Test 2: Momentos en hover
    print("\n" + "="*70)
    print("TEST 2: EMPUJE EN HOVER")
    print("="*70)
    Fx_h, Fy_h, Fz_h = calc_thrust.compute_hover_forces(throttle=0.75)
    W = params.geometry.mtow * 9.81
    print(f"Peso: {W:.2f} N")
    print(f"Empuje vertical: {-Fz_h:.2f} N")
    print(f"Margen: {(-Fz_h - W)/W * 100:.1f}%")
    
    print("\n✓ Tests completados")
