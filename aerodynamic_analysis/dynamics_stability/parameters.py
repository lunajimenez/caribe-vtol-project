"""
================================================================================
PARÁMETROS CONSOLIDADOS DEL VTOL QUADPLANE (4+1 PUSHER, H-TAIL)
================================================================================
Proyecto: VTOL eléctrico para transporte de suministros médicos
          Caribe Colombiano - Universidad Tecnológica de Bolívar
Configuración: QuadPlane 4+1 pusher, twin-boom H-tail, NACA 4412 wing

Sistema de referencia:
  Body Frame (BF): Origen en CG, +X adelante, +Y ala derecha, +Z abajo
  Earth Frame: NED (North-East-Down)

Etiquetas de confianza:
  [CONFIRMED] -- valor tomado directamente de datos XFLR5 o medición física
  [DERIVED]   -- calculado de valores CONFIRMED mediante fórmula documentada
  [PROVISIONAL] -- estimado; fuente documentada en el comentario

Referencia: Corda (2017) Ch.6; Sadraey (2012) Ch.6; Nelson (1998)
================================================================================
"""

import numpy as np
from dataclasses import dataclass, field


# ============================================================================
# ATMÓSFERA Y CONDICIONES DE VUELO
# ============================================================================

@dataclass
class Atmosphere:
    """ISA a nivel del mar (operación costera caribeña)."""
    rho: float = 1.225      # [kg/m3] densidad ISA sea-level [CONFIRMED]
    g:   float = 9.81       # [m/s2] gravedad estándar [CONFIRMED]
    mu:  float = 1.789e-5   # [Pa*s] viscosidad dinámica ISA 15°C [CONFIRMED]
    a:   float = 340.3      # [m/s] velocidad del sonido ISA [CONFIRMED]


# ============================================================================
# MASAS E INERCIAS
# ============================================================================

@dataclass
class MassProperties:
    """Propiedades másicas del VTOL.

    Inercias marcadas [PROVISIONAL]: deben sustituirse con valores CAD NX
    una vez se cierre el diseño detallado de la estructura.
    """
    m:   float = 10.0   # [kg] MTOW [CONFIRMED -- especificación de diseño]
    x_cg: float = 0.41  # [m] posición CG desde el morro [CONFIRMED -- sizing]

    # Momentos de inercia [PROVISIONAL -- estimados por fórmulas Roskam para UAVs]
    # Fuente estimación: Tyan et al. (2017) UAV mass-inertia correlations
    Ixx: float = 0.35   # [kg*m2] inercia de roll  [PROVISIONAL]
    Iyy: float = 0.45   # [kg*m2] inercia de pitch [PROVISIONAL]
    Izz: float = 0.70   # [kg*m2] inercia de yaw   [PROVISIONAL]
    Ixz: float = 0.02   # [kg*m2] producto de inercia XZ [PROVISIONAL]


# ============================================================================
# GEOMETRÍA DEL ALA
# ============================================================================

@dataclass
class WingGeometry:
    """Geometría alar -- NACA 4412, ala recta sin flecha."""
    S:        float = 0.567   # [m2] superficie alar [CONFIRMED -- sizing final]
    b:        float = 2.300   # [m]  envergadura [DERIVED: b = sqrt(AR*S) = sqrt(9.33*0.567)]
    c_bar:    float = 0.252   # [m]  MAC (cuerda media aerodinámica) [CONFIRMED]
    AR:       float = 9.33    # [-]  aspect ratio [CONFIRMED]
    taper:    float = 1.0     # [-]  taper ratio (ala recta) [CONFIRMED -- straight wing]
    sweep:    float = 0.0     # [rad] flecha (sin flecha) [CONFIRMED]
    dihedral: float = 0.0     # [rad] diedro [CONFIRMED -- diseño plano]
    iw:       float = 0.0     # [rad] incidencia del ala [CONFIRMED]

    # Posición en el avión
    x_LE:    float = 0.40     # [m] borde de ataque del ala desde el morro [PROVISIONAL -- medir CAD]
    x_ac:    float = 0.463    # [m] AC del ala desde morro = x_LE + 0.25*c_bar [DERIVED]

    airfoil:  str  = "NACA4412"
    Cm_ac_2D: float = -0.10   # [-] Cm perfil alrededor del AC (2D) [CONFIRMED -- NACA 4412 ref]

    def __post_init__(self):
        self.x_ac = self.x_LE + 0.25 * self.c_bar  # [DERIVED]


# ============================================================================
# GEOMETRÍA COLA HORIZONTAL (H-tail: estabilizador único)
# ============================================================================

@dataclass
class HTailGeometry:
    """Cola horizontal -- NACA 0012."""
    c_t:  float = 0.16    # [m]  cuerda de la cola [CONFIRMED]
    b_t:  float = 0.86    # [m]  envergadura de la cola [CONFIRMED]
    S_t:  float = 0.1376  # [m2] superficie = c_t x b_t [DERIVED]
    i_t:  float = np.radians(-2.0)  # [rad] incidencia de la cola (-2°) [CONFIRMED]
    x_ac_t: float = 1.20  # [m]  AC de cola desde el morro [CONFIRMED]

    # Elevator [PROVISIONAL -- diseño preliminar]
    ce_ct:  float = 0.35  # [-]  fracción cuerda elevador/cola [PROVISIONAL]
    tau_e:  float = 0.45  # [-]  eficiencia del elevador (Schlichting) [PROVISIONAL]
    de_max: float = np.radians(25.0)  # [rad] deflexión máx ±25° [PROVISIONAL]

    eta_t:  float = 0.9   # [-]  eficiencia dinámica de la cola (q_t/q_inf) [PROVISIONAL]

    def __post_init__(self):
        self.S_t = self.c_t * self.b_t  # [DERIVED]


# ============================================================================
# GEOMETRÍA COLA VERTICAL (2 aletas -- H-tail con booms)
# ============================================================================

@dataclass
class VTailGeometry:
    """Cola vertical -- configuración H-tail: 2 aletas en extremos de cola horiz."""
    n_fins: int   = 2      # [-]  número de aletas verticales [CONFIRMED -- H-tail]

    # Dimensiones por aleta [PROVISIONAL -- medir de CAD NX]
    c_v:   float = 0.16   # [m]  cuerda aleta vertical [PROVISIONAL]
    h_v:   float = 0.20   # [m]  altura por aleta [PROVISIONAL]
    S_v:   float = 0.064  # [m2] área total = 2 x c_v x h_v [DERIVED]

    x_ac_v: float = 1.15  # [m]  AC de cola vertical desde morro [PROVISIONAL]
    eta_v:  float = 0.9   # [-]  eficiencia dinámica cola vert [PROVISIONAL]

    # Rudder [PROVISIONAL]
    cr_cv:  float = 0.35  # [-]  fracción cuerda rudder/aleta [PROVISIONAL]
    tau_r:  float = 0.40  # [-]  eficiencia del rudder [PROVISIONAL]
    dr_max: float = np.radians(25.0)  # [rad] deflexión máx ±25° [PROVISIONAL]

    def __post_init__(self):
        self.S_v = self.n_fins * self.c_v * self.h_v  # [DERIVED]


# ============================================================================
# GEOMETRÍA DE ALERONES
# ============================================================================

@dataclass
class AileronGeometry:
    """Alerones -- diseño preliminar."""
    ca_c:   float = 0.25   # [-]  fracción cuerda alerón/ala [PROVISIONAL]
    tau_a:  float = 0.45   # [-]  eficiencia del alerón [PROVISIONAL]
    da_max: float = np.radians(25.0)  # [rad] deflexión máx ±25° [PROVISIONAL]
    y_inner: float = 0.55  # [m]  inicio alerón desde plano simetría [PROVISIONAL]
    y_outer: float = 1.05  # [m]  fin alerón desde plano simetría [PROVISIONAL]


# ============================================================================
# DERIVADAS AERODINÁMICAS -- DATOS XFLR5 + DERIVADOS ANALÍTICOS
# ============================================================================

@dataclass
class AeroDerivatives:
    """Derivadas de estabilidad aerodinámicas.

    Fuente primaria: XFLR5 v6.61, análisis tipo 'Plane', modelo 'Modelo alas'.
    Regresión lineal en rango alpha = [-5°, +11°].

    Fórmulas de derivadas de tasa: Corda (2017) §6.9; Nelson (1998) Ch.4
    """

    # ---- Ala (3D XFLR5 invíscido) ----
    CLa_w:  float = 5.0596   # [/rad] dCL/dalpha del ala 3D [CONFIRMED -- XFLR5 regresión]
    CL0_w:  float = 0.3597   # [-]    CL del ala a alpha=0° [CONFIRMED -- XFLR5]
    CMa_w:  float = -1.839   # [/rad] dCm/dalpha del ala ref. origen XFLR5 [CONFIRMED]
    CM0_w:  float = -0.2229  # [-]    Cm del ala a alpha=0° ref. origen XFLR5 [CONFIRMED]

    # ---- Cola horizontal (3D XFLR5 invíscido) ----
    CLa_t:  float = 4.3930   # [/rad] dCL/dalpha de la cola 3D [CONFIRMED -- XFLR5]
    CL0_t:  float = 0.0      # [-]    CL cola a alpha=0° (NACA 0012 simétrico) [CONFIRMED]
    CMa_t:  float = -0.588   # [/rad] dCm/dalpha de la cola ref. origen XFLR5 [CONFIRMED]

    # ---- Perfiles 2D (XFOIL, Re=70 000) ----
    CLa_2D_wing: float = 6.109  # [/rad] pendiente perfil NACA 4412 2D [CONFIRMED -- XFOIL]
    CLa_2D_tail: float = 7.123  # [/rad] pendiente perfil NACA 0012 2D [CONFIRMED -- XFOIL]

    # ---- Parámetros derivados de geometría y XFLR5 ----
    # Downwash: deps/dalpha = 2*CLa_w / (_*AR)  [Corda (2017) Eq. 6.78]
    deps_da: float = 0.345   # [-]    gradiente de downwash [DERIVED]

    # Coeficiente de volumen horizontal: VH = lt*St/(S*c)
    VH:      float = 0.761   # [-]    tail volume coefficient [DERIVED]

    # Arrastre parasitario (polar viscosa, velocidad = alpha cero sustentación)
    CD0_cruise: float = 0.014   # [-]  CD0 a 19.34 m/s [DERIVED -- polar XFLR5 viscosa]
    CD0_stall:  float = 0.016   # [-]  CD0 a 11.11 m/s [DERIVED -- polar XFLR5 viscosa]

    # Eficiencia de Oswald (ajuste parabólico polar viscosa)
    e_oswald: float = 0.78   # [-]    [DERIVED -- ajuste cuadrático polar viscosa XFLR5]

    # L/D máximo y CL óptimo
    LD_max:   float = 29.6   # [-]    [DERIVED -- polar viscosa 19.34 m/s]
    CL_opt:   float = 0.54   # [-]    CL a L/D máximo [DERIVED]

    # ---- Derivadas de tasa (pitch) -- Corda (2017) Eq. 6.98-6.105 ----
    # CLq = 2*CLa_t*eta_t*VH
    # Cmq = -2*CLa_t*eta_t*VH*(lt/c)     <- pitch damping
    # CLa_dot = 2*CLa_t*eta_t*VH*(deps/dalpha)
    # Cma_dot = -2*CLa_t*eta_t*VH*(lt/c)*(deps/dalpha)
    # Se calculan en __post_init__ usando parámetros de cola/ala

    CLq:     float = field(default=0.0)   # [DERIVED -- calculado en __post_init__]
    Cmq:     float = field(default=0.0)   # [DERIVED -- calculado en __post_init__]
    CLa_dot: float = field(default=0.0)   # [DERIVED -- calculado en __post_init__]
    Cma_dot: float = field(default=0.0)   # [DERIVED -- calculado en __post_init__]

    # ---- Derivadas laterales-direccionales ----
    # Calculadas analíticamente; ver stability_analysis.py para detalle
    CYb:   float = -0.42   # [/rad] _CY/_beta -- fuerza lateral por sideslip [DERIVED]
    Cnb:   float = 0.065   # [/rad] _Cn/_beta -- weathercock stability (>0 estable) [DERIVED]
    Clb:   float = -0.045  # [/rad] _Cl/_beta -- diedro effect (<0 estable) [PROVISIONAL]
    Clp:   float = -0.48   # [/rad] _Cl/_p_ -- roll damping (<0) [DERIVED]
    Clr:   float = 0.135   # [/rad] _Cl/_r_ -- roll due to yaw [DERIVED]
    Cnp:   float = -0.067  # [/rad] _Cn/_p_ -- yaw due to roll [DERIVED]
    Cnr:   float = -0.052  # [/rad] _Cn/_r_ -- yaw damping (<0) [DERIVED]

    # ---- Derivadas de control ----
    # Elevador: CLde = tau_e*CLa_t*eta_t*(St/S); Cmde = -CLde*(lt/c)
    CLde:  float = field(default=0.0)   # [DERIVED -- calculado en __post_init__]
    CMde:  float = field(default=0.0)   # [DERIVED -- calculado en __post_init__]

    # Alerones [PROVISIONAL -- depende de diseño final de alerón]
    Clda:  float = 0.148   # [/rad] _Cl/_delta_a [PROVISIONAL]
    Cnda:  float = -0.012  # [/rad] _Cn/_delta_a adverse yaw [PROVISIONAL]

    # Rudder [PROVISIONAL]
    CYdr:  float = 0.095   # [/rad] _CY/_delta_r [PROVISIONAL]
    Cldr:  float = 0.020   # [/rad] _Cl/_delta_r [PROVISIONAL]
    Cndr:  float = -0.060  # [/rad] _Cn/_delta_r [PROVISIONAL]

    def __post_init__(self):
        # Parámetros geométricos necesarios
        lt   = 0.79    # [m] brazo de cola = x_ac_t - x_cg = 1.20 - 0.41 [DERIVED]
        c_bar = 0.252  # [m] MAC
        S    = 0.567   # [m2]
        S_t  = 0.1376  # [m2]
        eta_t = 0.9    # eficiencia dinámica cola

        # Derivadas de tasa (Corda §6.9)
        self.CLq     =  2.0 * self.CLa_t * eta_t * self.VH                          # [DERIVED]
        self.Cmq     = -2.0 * self.CLa_t * eta_t * self.VH * (lt / c_bar)           # [DERIVED]
        self.CLa_dot =  2.0 * self.CLa_t * eta_t * self.VH * self.deps_da           # [DERIVED]
        self.Cma_dot = -2.0 * self.CLa_t * eta_t * self.VH * (lt / c_bar) * self.deps_da  # [DERIVED]

        # Derivadas de control elevador (Corda Eq. 6.93)
        tau_e = 0.45   # eficiencia elevador [PROVISIONAL]
        self.CLde =  tau_e * self.CLa_t * eta_t * (S_t / S)                         # [DERIVED]
        self.CMde = -self.CLde * (lt / c_bar)                                        # [DERIVED]

        # Gradiente de downwash verificación
        # deps/dalpha = 2*CLa_w/(_*AR) -- Corda Eq. 6.78
        AR = 9.33
        deps_check = 2.0 * self.CLa_w / (np.pi * AR)
        assert abs(deps_check - self.deps_da) < 0.01, (
            f"Downwash check failed: {deps_check:.3f} vs {self.deps_da:.3f}"
        )


# ============================================================================
# PROPULSIÓN
# ============================================================================

@dataclass
class PropulsionSystem:
    """Sistema de propulsión eléctrica.

    Motor pusher: genera empuje en +X del body frame.
    Rotores VTOL: generan empuje en -Z del body frame (hacia arriba).
    """
    # Motor horizontal pusher
    T_max_pusher: float = 24.5   # [N] empuje máximo motor pusher [PROVISIONAL]
    x_pusher:     float = 1.25   # [m] posición motor pusher desde morro [PROVISIONAL]

    # Rotores VTOL (4 motores en configuración X)
    n_rotors:     int   = 4
    T_max_rotor:  float = 24.5   # [N] empuje máximo por rotor [PROVISIONAL]
    arm_rotor:    float = 0.25   # [m] brazo desde CG a cada rotor [PROVISIONAL]

    # Constante de tiempo del motor (dinámica primer orden)
    tau_motor:    float = 0.10   # [s] [PROVISIONAL]


# ============================================================================
# CONTENEDOR MAESTRO DE CONFIGURACIÓN
# ============================================================================

@dataclass
class AircraftConfig:
    """Contenedor maestro -- instancia única para todos los módulos."""
    atm:      Atmosphere     = field(default_factory=Atmosphere)
    mass:     MassProperties = field(default_factory=MassProperties)
    wing:     WingGeometry   = field(default_factory=WingGeometry)
    htail:    HTailGeometry  = field(default_factory=HTailGeometry)
    vtail:    VTailGeometry  = field(default_factory=VTailGeometry)
    aileron:  AileronGeometry = field(default_factory=AileronGeometry)
    aero:     AeroDerivatives = field(default_factory=AeroDerivatives)
    prop:     PropulsionSystem = field(default_factory=PropulsionSystem)

    def __post_init__(self):
        self._compute_derived()

    def _compute_derived(self):
        """Calcula y verifica parámetros derivados inter-módulos."""
        w = self.wing
        ht = self.htail
        m = self.mass

        # Brazo de cola
        self.lt = ht.x_ac_t - m.x_cg   # [m] [DERIVED]

        # VH verificación
        VH_check = self.lt * ht.S_t / (w.S * w.c_bar)
        assert abs(VH_check - self.aero.VH) < 0.01, (
            f"VH check: {VH_check:.3f} vs {self.aero.VH:.3f}"
        )

        # Velocidades de referencia
        rho = self.atm.rho
        g   = self.atm.g
        self.V_cruise = 19.34   # [m/s] velocidad crucero [CONFIRMED]
        self.V_stall  = 12.05   # [m/s] velocidad pérdida [DERIVED: V_stall*sqrt(10/8.5)]

        # Presión dinámica en crucero
        self.q_cruise = 0.5 * rho * self.V_cruise**2   # [Pa] [DERIVED]
        self.q_stall  = 0.5 * rho * self.V_stall**2    # [Pa] [DERIVED]

        # Factor de arrastre inducido
        self.K = 1.0 / (np.pi * self.aero.e_oswald * w.AR)   # [DERIVED]

        # CL de trim en crucero: L = W -> CL_trim = W/(q*S)
        W = m.m * g
        self.CL_trim = W / (self.q_cruise * w.S)   # [DERIVED]

        # Coeficiente de volumen vertical: VV = Sv*lv/(S*b)
        lv = self.vtail.x_ac_v - m.x_cg
        self.VV = self.vtail.S_v * lv / (w.S * w.b)   # [DERIVED]

    def summary(self) -> str:
        m = self.mass
        w = self.wing
        a = self.aero
        lines = [
            "=" * 64,
            " VTOL CARIBE - PARAMETROS CONSOLIDADOS",
            "=" * 64,
            f"MASA:      m = {m.m:.2f} kg   CG = {m.x_cg:.3f} m",
            f"ALA:       S = {w.S:.3f} m2  AR = {w.AR:.2f}  c_bar = {w.c_bar:.3f} m",
            f"           x_ac = {w.x_ac:.3f} m  x_LE = {w.x_LE:.3f} m [PROVISIONAL]",
            f"COLA H:    St = {self.htail.S_t:.4f} m2  lt = {self.lt:.3f} m",
            f"           x_ac_t = {self.htail.x_ac_t:.3f} m  it = {np.degrees(self.htail.i_t):.1f} deg",
            f"VH = {a.VH:.3f}  VV = {self.VV:.4f} [DERIVED]",
            f"AERO:      CLa_w = {a.CLa_w:.4f}/rad  CLa_t = {a.CLa_t:.4f}/rad",
            f"           CMa_w = {a.CMa_w:.3f}/rad  deps_da = {a.deps_da:.3f}",
            f"           CD0 = {a.CD0_cruise:.4f}  e = {a.e_oswald:.2f}  K = {self.K:.5f}",
            f"           CLq = {a.CLq:.3f}  Cmq = {a.Cmq:.3f}  [DERIVED]",
            f"           CLde = {a.CLde:.4f}  CMde = {a.CMde:.4f}  [DERIVED]",
            f"VUELO:     V_cruise = {self.V_cruise:.2f} m/s  V_stall = {self.V_stall:.2f} m/s",
            f"           q_cruise = {self.q_cruise:.2f} Pa  CL_trim = {self.CL_trim:.4f}",
            "=" * 64,
        ]
        return "\n".join(lines)


def get_aircraft_config() -> AircraftConfig:
    """Retorna la configuración consolidada del VTOL Caribe."""
    return AircraftConfig()


# Alias para compatibilidad con módulos existentes
def get_default_vtol_parameters() -> AircraftConfig:
    return get_aircraft_config()


if __name__ == "__main__":
    cfg = get_aircraft_config()
    print(cfg.summary())
    print(f"\nDerivadas de tasa calculadas:")
    a = cfg.aero
    print(f"  CLq     = {a.CLq:.4f} /rad")
    print(f"  Cmq     = {a.Cmq:.4f} /rad")
    print(f"  CLa_dot = {a.CLa_dot:.4f} /rad")
    print(f"  Cma_dot = {a.Cma_dot:.4f} /rad")
    print(f"  CLde    = {a.CLde:.4f} /rad")
    print(f"  CMde    = {a.CMde:.4f} /rad")
    print(f"\n[OK] parameters.py OK")
