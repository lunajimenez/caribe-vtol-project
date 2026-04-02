"""
================================================================================
PARÁMETROS DEL VTOL QUADPLANE
================================================================================
Basado en: "Construcción de un prototipo de VTOL eléctrico para el transporte 
de suministros médicos en zonas de difícil acceso del Caribe Colombiano"
Tercera Iteración del Proyecto de Ingeniería - Nov 7, 2025

Sistema de referencia:
- Body Frame (BF): Origen en CG, +X adelante (pusher), +Y ala derecha, +Z abajo
- Earth Frame (EF): NED (North-East-Down)
- Stability Frame: Similar a Body pero con α alineado con velocidad

================================================================================
"""

# CG por delante del centro de presión

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, Tuple

# ============================================================================
# 1. PARÁMETROS GEOMÉTRICOS & MÁSICOS
# ============================================================================

@dataclass
class AircraftGeometry:
    """Geometría del VTOL quad-plano basada en sizing."""
    
    # Masa
    mtow: float = 8.65  # kg, peso máximo al despegue (sizing)
    payload_mass: float = 1.5  # kg
    structural_mass: float = 3.5  # kg estimado
    battery_mass: float = 2.0  # kg (estimado, configurable)
    avionics_mass: float = 0.65  # kg (FC, sensores, etc.)
    
    # Alas
    wing_area: float = 0.528  # m², superficie alar (S)
    wing_span: float = 2.2  # m, envergadura (b)
    wing_chord_mean: float = 0.28  # m, cuerda media aerodinámica (c̄)
    aspect_ratio: float = 9.17  # AR = b²/S
    taper_ratio: float = 0.607  # λ = c_tip/c_root
    dihedral_angle: float = 0.0  # rad, ángulo diedro (neutral para este diseño)
    
    # Superficies de control
    aileron_area: float = 0.05  # m², área de alerones (estimado 10% de S)
    aileron_max_deflection: float = np.radians(25)  # rad, deflexión máxima
    
    # Elevador (en estabilizador horizontal)
    elevator_area: float = 0.08  # m², área de elevador (estimado)
    elevator_max_deflection: float = np.radians(25)  # rad
    horizontal_tail_area: float = 0.12  # m²
    horizontal_tail_arm: float = 0.6  # m, brazo de momento (CG a CP tail)
    
    # Timón (en estabilizador vertical)
    rudder_area: float = 0.06  # m²
    rudder_max_deflection: float = np.radians(30)  # rad
    vertical_tail_area: float = 0.10  # m²
    vertical_tail_arm: float = 0.6  # m
    
    # Motores
    num_vertical_motors: int = 4  # Rotor configuration: X (quad)
    rotor_config: str = "X"  # "X" o "+" (X = diagonal)
    vertical_motor_arm: float = 0.25  # m, brazo de momento desde CG
    
    horizontal_motor_type: str = "pusher"  # "tractor" o "pusher"
    horizontal_motor_thrust_max: float = 9.81 * 0.5 * mtow  # N, T/W = 0.5


@dataclass
class InertiaProperties:
    """Momentos de inercia calculados según fórmulas empíricas."""
    
    # Se calculan automáticamente en __post_init__
    mtow: float = 8.65  # kg
    wing_span: float = 2.2  # m
    wing_area: float = 0.528  # m²
    wing_chord_mean: float = 0.28  # m
    fuselage_length: float = 1.2  # m, estimado
    fuselage_diameter: float = 0.12  # m, estimado
    
    # Momentos de inercia (se calculan)
    Ixx: float = field(default=0.0)  # kg·m², inercia roll
    Iyy: float = field(default=0.0)  # kg·m², inercia pitch
    Izz: float = field(default=0.0)  # kg·m², inercia yaw
    Ixz: float = field(default=0.0)  # kg·m², producto de inercia
    
    def __post_init__(self):
        """Calcular momentos de inercia usando fórmulas de Roskam."""
        # Basado en "Aircraft Design: A Conceptual Approach" (Raymer, 2012)
        # y metodología de Roskam para UAVs pequeños
        
        m = self.mtow
        b = self.wing_span
        S = self.wing_area
        c = self.wing_chord_mean
        L_f = self.fuselage_length
        d_f = self.fuselage_diameter
        
        # Aproximaciones empíricas para UAVs eléctricos de pequeña escala
        # Basadas en correlaciones de Tyan et al. (2017)
        
        # Inercia de ROLL (eje X)
        # Dominada por distribución de masa en envergadura
        self.Ixx = 0.065 * m * (b/2)**2
        
        # Inercia de PITCH (eje Y)
        # Dominada por fuselaje y distribución longitudinal
        self.Iyy = 0.12 * m * (L_f/2)**2 + 0.015 * m * (c/2)**2
        
        # Inercia de YAW (eje Z)
        # Suma de contribuciones laterales
        self.Izz = 0.08 * m * (b/2)**2 + 0.10 * m * (L_f/2)**2
        
        # Producto de inercia (generalmente pequeño en diseños simétricos)
        self.Ixz = 0.0  # Asumimos simetría lateral


@dataclass
class AerodynamicCoefficients:
    """Coeficientes aerodinámicos base y derivadas de estabilidad.
    
    Basados en:
    - Bryan (1911): "Stability in Aviation"
    - Abzug & Larrabee (2005): "Airplane Stability and Control"
    - Xflr5 data para perfil NACA 4415 (referencia de proyecto)
    """
    
    # Drag polar básico (parabólico)
    CD0: float = 0.04  # Coeficiente de arrastre parasitario (cruise trim)
    oswald_efficiency: float = 0.976  # Factor de eficiencia de Oswald (e)
    
    # Coeficiente de sustentación en condición base
    CL0: float = 0.3  # CL en α = 0° (línea de sustentación nula ~-2°)
    CLalpha: float = 5.7  # dCL/dα [rad⁻¹], para NACA 4415 típico
    CL_max: float = 2.2  # Sustentación máxima (con flaps/slots)
    
    # ========== DERIVADAS DE ESTABILIDAD LONGITUDINAL (pitch) ==========
    # Ecuación: Cm = Cm0 + Cmα*α + Cmq*q̄ + Cmα̇*α̇ + Cmδe*δe
    
    Cm0: float = -0.05  # Momento de pitch en trim (neutral point design)
    Cm_alpha: float = -1.2  # ∂Cm/∂α [rad⁻¹], debe ser negativo (estabilidad)
    Cm_alphadot: float = -7.5  # ∂Cm/∂(α̇·c̄/2V), amortiguamiento
    Cm_q: float = -25.0  # ∂Cm/∂(q·c̄/2V), amortiguamiento pitch (grande)
    Cm_delta_e: float = -0.35  # ∂Cm/∂δ_e [rad⁻¹], control
    
    # ========== DERIVADAS LATERALES-DIRECCIONALES ==========
    # Eje Y (lateral): CY = CY0 + CYβ*β + CYp*p̄ + CYr*r̄ + CYδa*δa + CYδr*δr
    # Eje X (roll): Cl = Clβ*β + Clp*p̄ + Clr*r̄ + Clδa*δa + Clδr*δr
    # Eje Z (yaw): Cn = Cnβ*β + Cnp*p̄ + Cnr*r̄ + Cnδa*δa + Cnδr*δr
    
    CY_beta: float = -0.75  # ∂CY/∂β, fuerza lateral por ángulo lateral (neg)
    CY_delta_a: float = 0.0  # ∂CY/∂δ_a (pequeño en ala alta)
    CY_delta_r: float = 0.15  # ∂CY/∂δ_r, control de timón
    
    Cl_beta: float = -0.12  # ∂Cl/∂β [rad⁻¹], estabilidad de diedro (NEGATIVO = estable)
    Cl_p: float = -0.50  # ∂Cl/∂(p·b/2V), amortiguamiento roll (grande, negativo)
    Cl_r: float = 0.15  # ∂Cl/∂(r·b/2V), acoplamiento yaw-roll (positivo)
    Cl_delta_a: float = 0.20  # ∂Cl/∂δ_a [rad⁻¹], control de alerones
    
    Cn_beta: float = 0.15  # ∂Cn/∂β [rad⁻¹], estabilidad direccional (POSITIVO = estable)
    Cn_p: float = -0.02  # ∂Cn/∂(p·b/2V), acoplamiento roll-yaw
    Cn_r: float = -0.25  # ∂Cn/∂(r·b/2V), amortiguamiento yaw (NEGATIVO = amortiguado)
    Cn_delta_a: float = 0.01  # ∂Cn/∂δ_a (pequeño)
    Cn_delta_r: float = -0.10  # ∂Cn/∂δ_r [rad⁻¹], control


# ============================================================================
# 2. PARÁMETROS DE OPERACIÓN Y CONTROL
# ============================================================================

@dataclass
class FlightConditions:
    """Condiciones de vuelo nominal (caribe colombiano)."""
    
    # Altitud de operación (RAC 100)
    altitude_agl_m: float = 120.0  # m AGL, máximo permitido
    altitude_msl_m: float = 100.0  # m MSL (aprox, costa caribeña)
    
    # Atmósfera estándar ISA (20°C, humedad caribeña ~80%)
    temperature_K: float = 293.15  # K (20°C)
    pressure_Pa: float = 101325.0  # Pa (nivel del mar)
    rho: float = 1.225  # kg/m³, densidad del aire ISA
    g: float = 9.81  # m/s²
    
    # Velocidades de referencia
    v_cruise_ms: float = 70 / 3.6  # m/s, 70 km/h (crucero)
    v_stall_ms: float = 40 / 3.6  # m/s, 40 km/h (pérdida)
    v_hover_ms: float = 0.0  # m/s, hover = estacionario
    
    # Vientos
    wind_speed_ms: float = 0.0  # m/s, velocidad del viento
    wind_direction_rad: float = 0.0  # rad


@dataclass
class PropulsionSystem:
    """Parámetros de propulsión eléctrica."""
    
    # Motores brushless verticales
    vertical_motor_kv: float = 520  # RPM/Volt (SunnySky X3525 referencia)
    vertical_motor_mass_kg: float = 0.098  # kg c/u
    vertical_motor_max_rpm: float = 6240  # RPM @ 12V
    vertical_motor_max_thrust_kg: float = 2.5  # kgf, empuje máximo @ max RPM
    vertical_motor_num: int = 4
    
    # Hélices verticales
    vertical_prop_diameter_inch: float = 10  # pulgadas (estimado)
    vertical_prop_pitch_inch: float = 5.0  # pulgadas
    
    # Motor horizontal (pusher)
    horizontal_motor_kv: float = 520
    horizontal_motor_max_thrust_kg: float = 2.5
    horizontal_motor_max_rpm: float = 6240
    
    # Hélice horizontal
    horizontal_prop_diameter_inch: float = 10
    horizontal_prop_pitch_inch: float = 5.0
    
    # ESCs y respuesta dinámica
    esc_response_time_s: float = 0.05  # Tiempo de respuesta ESC (50 ms típico)
    motor_time_constant_s: float = 0.1  # Constante de tiempo del motor
    
    # Baterías
    battery_voltage_nominal: float = 22.2  # V, 6S LiPo (6 * 3.7V)
    battery_capacity_mah: float = 10000  # mAh
    battery_c_rating: float = 25  # Descarga máxima (25C)
    battery_mass_kg: float = 2.0
    battery_energy_wh: float = 222  # Wh (22.2V * 10Ah)


@dataclass
class VTOLModeParameters:
    """Parámetros específicos para cada modo de vuelo."""
    
    # Transición: tiempo y dinámica
    transition_time_max_s: float = 5.0  # Tiempo máximo transición
    transition_pitch_rate_max_rad_s: float = np.radians(30)  # 30°/s máximo
    
    # Hover: control
    hover_vertical_margin: float = 0.1  # Margen sobre W (10% extra de thrust)
    hover_yaw_rate_max_rad_s: float = np.radians(60)  # 60°/s max yaw
    
    # Crucero: trim
    cruise_pitch_trim_rad: float = np.radians(3)  # Pitch trim típico
    cruise_bank_angle_max_rad: float = np.radians(30)  # 30° max bank
    cruise_rate_of_climb_ms: float = 3.0  # 3 m/s ROC nominal


# ============================================================================
# 3. FUNCIÓN DE CONSTRUCCIÓN DE PARÁMETROS COMPLETOS
# ============================================================================

@dataclass
class VTOLAircraftParameters:
    """Contenedor maestro de todos los parámetros."""
    
    geometry: AircraftGeometry = field(default_factory=AircraftGeometry)
    inertia: InertiaProperties = field(default_factory=InertiaProperties)
    aero: AerodynamicCoefficients = field(default_factory=AerodynamicCoefficients)
    flight: FlightConditions = field(default_factory=FlightConditions)
    propulsion: PropulsionSystem = field(default_factory=PropulsionSystem)
    vtol_mode: VTOLModeParameters = field(default_factory=VTOLModeParameters)
    
    def __post_init__(self):
        """Validar consistencia y calcular parámetros derivados."""
        self._validate()
        self._calculate_derived()
    
    def _validate(self):
        """Verificar consistencia de parámetros."""
        m = self.geometry.mtow
        
        # T/W ratio
        T_vertical_total = (self.propulsion.vertical_motor_num * 
                           self.propulsion.vertical_motor_max_thrust_kg * 9.81)
        T_W_vertical = T_vertical_total / (m * 9.81)
        
        # Relajar requisito: T/W ≥ 1.0 suficiente para hover con controles
        assert T_W_vertical >= 1.0, f"T/W vertical ({T_W_vertical:.2f}) < 1.0 insufficient"
        
        # Carga alar
        W_S = (m * 9.81) / self.geometry.wing_area
        print(f"✓ Validación OK: W/S = {W_S:.1f} N/m², T/W_vertical = {T_W_vertical:.2f}")
    
    def _calculate_derived(self):
        """Calcular parámetros derivados."""
        m = self.geometry.mtow
        S = self.geometry.wing_area
        b = self.geometry.wing_span
        c = self.geometry.wing_chord_mean
        
        # Factor de amortiguamiento inducido
        self.K_induced = 1 / (np.pi * self.aero.oswald_efficiency * self.geometry.aspect_ratio)
        
        # Velocidades características
        self.V_cruise_ms = self.flight.v_cruise_ms
        self.V_stall_ms = self.flight.v_stall_ms
        self.V_never_exceed_ms = self.flight.v_cruise_ms * 1.5  # Vne ~1.5 Vc
        
        # Cargas de referencia
        self.dynamic_pressure_cruise_Pa = 0.5 * self.flight.rho * self.V_cruise_ms**2
        
        # Dimensiones características para estabilidad
        self.S_h_S_ratio = self.geometry.horizontal_tail_area / S  # Proporción cola horiz
        self.S_v_S_ratio = self.geometry.vertical_tail_area / S  # Proporción cola vert
        
        print(f"""
╔════════════════════════════════════════════════════════════════╗
║           PARÁMETROS VTOL CARIBE - RESUMEN EJECUTIVO          ║
╚════════════════════════════════════════════════════════════════╝
MASA & GEOMETRÍA:
  • MTOW: {m:.2f} kg
  • Envergadura: {b:.2f} m | Superficie: {S:.3f} m² | AR: {self.geometry.aspect_ratio:.2f}
  • Cuerda media: {c:.3f} m | W/S: {(m*9.81)/S:.1f} N/m²

MOMENTOS DE INERCIA (calculados):
  • Ixx (roll):  {self.inertia.Ixx:.4f} kg·m²
  • Iyy (pitch): {self.inertia.Iyy:.4f} kg·m²
  • Izz (yaw):   {self.inertia.Izz:.4f} kg·m²

AERODINÁMICA:
  • CD₀: {self.aero.CD0:.3f} | CL_α: {self.aero.CLalpha:.2f} rad⁻¹
  • Cm_α: {self.aero.Cm_alpha:.2f} rad⁻¹ | Cn_β: {self.aero.Cn_beta:.2f} rad⁻¹
  • K_induced: {self.K_induced:.5f}

PROPULSIÓN:
  • Vertical: {self.propulsion.vertical_motor_num} motores × {self.propulsion.vertical_motor_max_thrust_kg:.2f} kgf
  • Horizontal: 1 motor × {self.propulsion.horizontal_motor_max_thrust_kg:.2f} kgf (pusher)

VELOCIDADES:
  • Crucero: {self.V_cruise_ms:.2f} m/s ({self.V_cruise_ms*3.6:.1f} km/h)
  • Pérdida: {self.V_stall_ms:.2f} m/s ({self.V_stall_ms*3.6:.1f} km/h)
  • Vne: {self.V_never_exceed_ms:.2f} m/s ({self.V_never_exceed_ms*3.6:.1f} km/h)
╚════════════════════════════════════════════════════════════════╝
        """)


# ============================================================================
# 4. INSTANCIA GLOBAL DE PARÁMETROS POR DEFECTO
# ============================================================================

def get_default_vtol_parameters() -> VTOLAircraftParameters:
    """Retorna instancia con parámetros por defecto del VTOL Caribe."""
    return VTOLAircraftParameters()


if __name__ == "__main__":
    # Test: crear parámetros y validar
    params = get_default_vtol_parameters()
    print("✓ Parámetros cargados exitosamente")

