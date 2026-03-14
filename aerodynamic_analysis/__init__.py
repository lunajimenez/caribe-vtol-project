"""
================================================================================
CARIBE VTOL - AERODYNAMIC ANALYSIS PACKAGE
================================================================================
Suite completa de análisis de dinámicas y estabilidad para el prototipo VTOL
de la Universidad Tecnológica de Bolívar.

Módulos:
  - parameters: Definición de parámetros geométricos, másicos y aerodinámicos
  - forces_moments: Cálculo de fuerzas y momentos (aerodinámicos + propulsión)
  - dynamics_6dof: Motor de dinámicas 6 DOF con integrador RK45
  - stability_analysis: Análisis de estabilidad (eigenanalysis, trim)
  - simulator: Simulador completo con escenarios de vuelo

Uso básico:
  >>> from aerodynamic_analysis import get_default_vtol_parameters
  >>> from aerodynamic_analysis import VTOLDynamicsEngine, StabilityAnalyzer
  >>> 
  >>> params = get_default_vtol_parameters()
  >>> engine = VTOLDynamicsEngine(params)
  >>> analyzer = StabilityAnalyzer(params)

Referencia:
  - Bryan, G.H. (1911). "Stability in Aviation"
  - Abzug & Larrabee (2005). "Airplane Stability and Control", 2nd Ed.
  - Nelson, R.C. (1998). "Flight Stability and Automatic Control", 2nd Ed.
  - Etkin & Reid (1996). "Dynamics of Atmospheric Flight"

================================================================================
"""

from parameters import (
    AircraftGeometry,
    InertiaProperties,
    AerodynamicCoefficients,
    FlightConditions,
    PropulsionSystem,
    VTOLModeParameters,
    VTOLAircraftParameters,
    get_default_vtol_parameters,
)

from forces_moments import (
    AerodynamicCalculator,
    ThrustCalculator,
)

from dynamics_6dof import (
    AircraftState,
    ControlInput,
    VTOLDynamicsEngine,
)

from stability_analysis import (
    StabilityAnalyzer,
    print_stability_report,
)

from simulator import (
    HoverScenario,
    TransitionScenario,
    CruiseScenario,
    GustRecoveryScenario,
    extract_time_series,
    plot_results,
)

__version__ = "1.0.0"
__author__ = "Universidad Tecnológica de Bolívar - Ingeniería Mecatrónica"
__date__ = "2025-03-13"

__all__ = [
    # Parameters
    "AircraftGeometry",
    "InertiaProperties",
    "AerodynamicCoefficients",
    "FlightConditions",
    "PropulsionSystem",
    "VTOLModeParameters",
    "VTOLAircraftParameters",
    "get_default_vtol_parameters",
    # Forces & Moments
    "AerodynamicCalculator",
    "ThrustCalculator",
    # Dynamics
    "AircraftState",
    "ControlInput",
    "VTOLDynamicsEngine",
    # Stability
    "StabilityAnalyzer",
    "print_stability_report",
    # Simulator
    "HoverScenario",
    "TransitionScenario",
    "CruiseScenario",
    "GustRecoveryScenario",
    "extract_time_series",
    "plot_results",
]
