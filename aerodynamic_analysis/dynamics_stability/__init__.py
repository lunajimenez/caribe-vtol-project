"""
================================================================================
CARIBE VTOL -- MÓDULO DE DINÁMICA Y ESTABILIDAD
================================================================================
Suite completa de análisis dinámico para el VTOL QuadPlane 4+1 pusher.
Universidad Tecnológica de Bolívar -- Ingeniería Mecatrónica.

Módulos:
  parameters      -- Parámetros geométricos, másicos y aerodinámicos (dataclasses)
  xflr5_loader    -- Carga y procesamiento de polares XFLR5 + regresión lineal
  forces_moments  -- Fuerzas y momentos en body frame (ala + cola + gravedad + empuje)
  dynamics_6dof   -- Ecuaciones 6DOF + integrador RK4
  stability_analysis -- Estabilidad estática y dinámica (5 modos)
  simulator       -- Escenarios de perturbación y gráficas de validación
  simulink_export -- Exporta parámetros y matrices a MATLAB/Simulink (.m)

Uso rápido:
  >>> from aerodynamic_analysis.dynamics_stability import get_aircraft_config
  >>> from aerodynamic_analysis.dynamics_stability import StabilityAnalyzer
  >>> cfg = get_aircraft_config()
  >>> results = StabilityAnalyzer(cfg).run()
================================================================================
"""

from .parameters import (
    Atmosphere,
    MassProperties,
    WingGeometry,
    HTailGeometry,
    VTailGeometry,
    AileronGeometry,
    AeroDerivatives,
    PropulsionSystem,
    AircraftConfig,
    get_aircraft_config,
    get_default_vtol_parameters,
)

from .xflr5_loader import (
    Polar3D,
    Polar2D,
    LinearDerivatives,
    DragPolarFit,
    AeroTableWing,
    AeroTableTail,
    load_all_data,
    compute_linear_derivatives,
    compute_drag_polar,
    verify_velocity_independence,
)

from .forces_moments import (
    AeroModel,
    PropModel,
    airspeed_angles,
)

from .dynamics_6dof import (
    State,
    Controls,
    Dynamics6DOF,
    euler_dcm,
    numerical_jacobian,
)

from .stability_analysis import (
    StaticStability,
    DynamicLinearization,
    StabilityAnalyzer,
    analyze_longitudinal,
    analyze_lateral,
    phugoid_analytic,
    print_stability_report,
)

from .simulator import (
    scenario_elevator_step,
    scenario_beta_perturbation,
    scenario_aileron_pulse,
    plot_elevator_step,
    plot_beta_perturbation,
    plot_aileron_pulse,
)

from .simulink_export import export_simulink

__version__ = "2.0.0"
__author__  = "Universidad Tecnológica de Bolívar -- Ingeniería Mecatrónica"
__date__    = "2026-05-29"

__all__ = [
    # parameters
    "Atmosphere", "MassProperties", "WingGeometry", "HTailGeometry",
    "VTailGeometry", "AileronGeometry", "AeroDerivatives",
    "PropulsionSystem", "AircraftConfig",
    "get_aircraft_config", "get_default_vtol_parameters",
    # xflr5_loader
    "Polar3D", "Polar2D", "LinearDerivatives", "DragPolarFit",
    "AeroTableWing", "AeroTableTail",
    "load_all_data", "compute_linear_derivatives",
    "compute_drag_polar", "verify_velocity_independence",
    # forces_moments
    "AeroModel", "PropModel", "airspeed_angles",
    # dynamics_6dof
    "State", "Controls", "Dynamics6DOF", "euler_dcm", "numerical_jacobian",
    # stability_analysis
    "StaticStability", "DynamicLinearization", "StabilityAnalyzer",
    "analyze_longitudinal", "analyze_lateral",
    "phugoid_analytic", "print_stability_report",
    # simulator
    "scenario_elevator_step", "scenario_beta_perturbation",
    "scenario_aileron_pulse",
    "plot_elevator_step", "plot_beta_perturbation", "plot_aileron_pulse",
    # simulink_export
    "export_simulink",
]
