"""
================================================================================
SIMULADOR PRINCIPAL VTOL - ESCENARIOS DE VUELO
================================================================================
Ejecuta simulaciones completas del VTOL bajo diferentes escenarios:

1. HOVER: Vuelo estacionario con perturbaciones
2. TRANSICIÓN: Cambio de hover a crucero
3. CRUCERO: Vuelo de ala fija
4. RECUPERACIÓN: Respuesta ante perturbaciones (gust, control input)

Genera reportes con:
- Trayectorias de estado
- Análisis de estabilidad
- Métricas de desempeño
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Callable, Tuple, List
from parameters import get_default_vtol_parameters, AircraftState, ControlInput
from dynamics_6dof import VTOLDynamicsEngine
from stability_analysis import StabilityAnalyzer, print_stability_report
from forces_moments import AerodynamicCalculator


# ============================================================================
# ESCENARIOS DE SIMULACIÓN
# ============================================================================

class HoverScenario:
    """Simulación de hover: estacionario con perturbación."""
    
    def __init__(self, engine: VTOLDynamicsEngine):
        self.engine = engine
    
    def control_law(self, t: float, state: AircraftState) -> ControlInput:
        """
        Control simple PID para mantener hover.
        
        En tiempo real (si existiera FC), esto sería un controlador embarcado.
        """
        # Feedback simplificado
        throttle = 0.75  # Aproximado para equilibrio
        
        # Control de actitud (pitch/roll)
        delta_e = -state.q * 0.1  # Amortiguamiento pitch
        delta_a = -state.p * 0.1  # Amortiguamiento roll
        
        return ControlInput(
            throttle_vertical=throttle,
            throttle_horizontal=0.0,
            delta_e=delta_e,
            delta_a=delta_a,
            mode="HOVER"
        )
    
    def run(self, t_end: float = 30.0) -> Tuple[List[AircraftState], np.ndarray]:
        """Ejecuta simulación."""
        # Perturbación inicial: pequeño pitch
        initial_state = AircraftState(
            z=0.0,
            theta=np.radians(5.0),  # 5° pitch
            u=0.1,  # Pequeña velocidad forward
        )
        
        states, times = self.engine.simulate(
            initial_state,
            self.control_law,
            t_end=t_end,
            dt=0.05
        )
        
        return states, times


class TransitionScenario:
    """Simulación de transición: hover → crucero."""
    
    def __init__(self, engine: VTOLDynamicsEngine):
        self.engine = engine
        self.params = engine.params
    
    def control_law(self, t: float, state: AircraftState) -> ControlInput:
        """
        Control de transición: programa de empujes en función del tiempo.
        
        t ∈ [0, 1]: Fases de transición
          - [0, 0.5): Acelerar manteniendo hover
          - [0.5, 1.0): Pitch up progresivo
          - [1.0, ∞): Crucero
        """
        t_trans = 5.0  # Duración transición (s)
        phase = np.clip(t / t_trans, 0, 1)
        
        if phase < 0.5:
            # Aceleración horizontal inicial
            throttle_h = phase * 0.6
            throttle_v = 0.75
            theta_cmd = 0.0
        else:
            # Pitch up y transición a ala fija
            throttle_h = 0.3 + (phase - 0.5) * 0.4
            throttle_v = 0.75 * (1 - (phase - 0.5))
            theta_cmd = (phase - 0.5) * np.radians(15)
        
        # Control
        delta_e = theta_cmd - state.theta
        
        return ControlInput(
            throttle_vertical=throttle_v,
            throttle_horizontal=throttle_h,
            delta_e=delta_e,
            mode="TRANSITION"
        )
    
    def run(self, t_end: float = 15.0) -> Tuple[List[AircraftState], np.ndarray]:
        initial_state = AircraftState(z=0.0)
        
        states, times = self.engine.simulate(
            initial_state,
            self.control_law,
            t_end=t_end,
            dt=0.05
        )
        
        return states, times


class CruiseScenario:
    """Simulación en crucero estable."""
    
    def __init__(self, engine: VTOLDynamicsEngine):
        self.engine = engine
        self.params = engine.params
    
    def control_law(self, t: float, state: AircraftState) -> ControlInput:
        """Control simple de crucero."""
        V_cruise = self.params.flight.v_cruise_ms
        
        # Trim aproximado
        theta_trim = np.radians(3.0)
        throttle_trim = 0.6
        
        # Control PID
        error_v = (V_cruise - state.u)
        throttle = throttle_trim + error_v * 0.1
        
        delta_e = (theta_trim - state.theta) - state.q * 0.05
        
        return ControlInput(
            throttle_vertical=0.1,
            throttle_horizontal=np.clip(throttle, 0, 1),
            delta_e=delta_e,
            mode="CRUISE"
        )
    
    def run(self, t_end: float = 30.0) -> Tuple[List[AircraftState], np.ndarray]:
        # Estado inicial: en crucero
        V_cruise = self.params.flight.v_cruise_ms
        
        initial_state = AircraftState(
            u=V_cruise,
            theta=np.radians(3.0),
            w=V_cruise * np.tan(np.radians(3.0))
        )
        
        states, times = self.engine.simulate(
            initial_state,
            self.control_law,
            t_end=t_end,
            dt=0.05
        )
        
        return states, times


class GustRecoveryScenario:
    """Respuesta ante ráfaga de viento (perturbación aerodinámicos)."""
    
    def __init__(self, engine: VTOLDynamicsEngine):
        self.engine = engine
    
    def control_law(self, t: float, state: AircraftState) -> ControlInput:
        """Control con autopiloto básico."""
        V_cruise = self.engine.params.flight.v_cruise_ms
        
        # Ráfaga en t=5s
        if 5.0 < t < 5.5:
            # Simular como cambio en velocidad (simplificado)
            pass
        
        throttle = 0.6 + (V_cruise - state.u) * 0.1
        delta_e = -state.theta - state.q * 0.1
        
        return ControlInput(
            throttle_vertical=0.1,
            throttle_horizontal=np.clip(throttle, 0, 1),
            delta_e=delta_e,
            mode="CRUISE"
        )
    
    def run(self, t_end: float = 20.0) -> Tuple[List[AircraftState], np.ndarray]:
        V_cruise = self.engine.params.flight.v_cruise_ms
        
        initial_state = AircraftState(
            u=V_cruise,
            theta=np.radians(3.0),
            w=V_cruise * np.tan(np.radians(3.0))
        )
        
        states, times = self.engine.simulate(
            initial_state,
            self.control_law,
            t_end=t_end,
            dt=0.05
        )
        
        return states, times


# ============================================================================
# ANÁLISIS Y REPORTE
# ============================================================================

def extract_time_series(states: List[AircraftState], times: np.ndarray) -> dict:
    """Extrae series temporales de todas las variables."""
    
    n = len(states)
    
    data = {
        'time': times,
        # Posición
        'x': np.array([s.x for s in states]),
        'y': np.array([s.y for s in states]),
        'z': np.array([s.z for s in states]),
        # Velocidad body
        'u': np.array([s.u for s in states]),
        'v': np.array([s.v for s in states]),
        'w': np.array([s.w for s in states]),
        # Actitud
        'phi': np.array([s.phi for s in states]),
        'theta': np.array([s.theta for s in states]),
        'psi': np.array([s.psi for s in states]),
        # Vel. Angular
        'p': np.array([s.p for s in states]),
        'q': np.array([s.q for s in states]),
        'r': np.array([s.r for s in states]),
    }
    
    # Velocidad aerodinámica
    data['V_airspeed'] = np.sqrt(data['u']**2 + data['v']**2 + data['w']**2)
    
    # Ángulos aerodinámicos (aproximado)
    data['alpha'] = np.arctan2(data['w'], data['u'])
    data['beta'] = np.arcsin(np.clip(data['v'] / data['V_airspeed'], -1, 1))
    
    return data


def plot_results(data: dict, scenario_name: str = ""):
    """Grafica resultados de simulación."""
    
    fig, axes = plt.subplots(4, 3, figsize=(14, 12))
    fig.suptitle(f"VTOL Simulation - {scenario_name}", fontsize=14, fontweight='bold')
    
    time = data['time']
    
    # Fila 1: Posición
    axes[0, 0].plot(time, data['x'], 'b-')
    axes[0, 0].set_ylabel('X (m)')
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(time, data['y'], 'g-')
    axes[0, 1].set_ylabel('Y (m)')
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[0, 2].plot(time, -data['z'], 'r-')  # Altura = -z
    axes[0, 2].set_ylabel('Altura (m)')
    axes[0, 2].grid(True, alpha=0.3)
    
    # Fila 2: Velocidad
    axes[1, 0].plot(time, data['u'], 'b-', label='u')
    axes[1, 0].plot(time, data['v'], 'g-', label='v')
    axes[1, 0].plot(time, data['w'], 'r-', label='w')
    axes[1, 0].set_ylabel('Velocidad (m/s)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].plot(time, data['V_airspeed'], 'k-')
    axes[1, 1].set_ylabel('V airspeed (m/s)')
    axes[1, 1].grid(True, alpha=0.3)
    
    axes[1, 2].plot(time, np.degrees(data['alpha']), 'b-', label='α')
    axes[1, 2].plot(time, np.degrees(data['beta']), 'g-', label='β')
    axes[1, 2].set_ylabel('Ángulos aero (°)')
    axes[1, 2].legend()
    axes[1, 2].grid(True, alpha=0.3)
    
    # Fila 3: Actitud
    axes[2, 0].plot(time, np.degrees(data['phi']), 'b-')
    axes[2, 0].set_ylabel('Roll φ (°)')
    axes[2, 0].grid(True, alpha=0.3)
    
    axes[2, 1].plot(time, np.degrees(data['theta']), 'g-')
    axes[2, 1].set_ylabel('Pitch θ (°)')
    axes[2, 1].grid(True, alpha=0.3)
    
    axes[2, 2].plot(time, np.degrees(data['psi']), 'r-')
    axes[2, 2].set_ylabel('Yaw ψ (°)')
    axes[2, 2].grid(True, alpha=0.3)
    
    # Fila 4: Vel. Angular
    axes[3, 0].plot(time, data['p'], 'b-')
    axes[3, 0].set_ylabel('p (rad/s)')
    axes[3, 0].set_xlabel('Time (s)')
    axes[3, 0].grid(True, alpha=0.3)
    
    axes[3, 1].plot(time, data['q'], 'g-')
    axes[3, 1].set_ylabel('q (rad/s)')
    axes[3, 1].set_xlabel('Time (s)')
    axes[3, 1].grid(True, alpha=0.3)
    
    axes[3, 2].plot(time, data['r'], 'r-')
    axes[3, 2].set_ylabel('r (rad/s)')
    axes[3, 2].set_xlabel('Time (s)')
    axes[3, 2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


# ============================================================================
# MAIN
# ============================================================================

def main():
    """Ejecuta suite completa de simulaciones."""
    
    print("\n" + "="*80)
    print("CARIBE VTOL - SIMULADOR DE DINÁMICAS")
    print("="*80)
    
    # Inicializar
    params = get_default_vtol_parameters()
    engine = VTOLDynamicsEngine(params)
    analyzer = StabilityAnalyzer(params)
    
    # ========================================================================
    # ANÁLISIS DE ESTABILIDAD INICIAL
    # ========================================================================
    print("\n1. ANÁLISIS DE ESTABILIDAD EN TRIM CONDITIONS")
    print("-" * 80)
    
    eq_hover, ctrl_hover = analyzer.find_trim_hover()
    analysis_hover = analyzer.analyze_stability(eq_hover, ctrl_hover)
    print_stability_report(analysis_hover, "HOVER STABILITY")
    
    eq_cruise, ctrl_cruise = analyzer.find_trim_cruise()
    analysis_cruise = analyzer.analyze_stability(eq_cruise, ctrl_cruise)
    print_stability_report(analysis_cruise, "CRUISE STABILITY")
    
    # ========================================================================
    # SIMULACIONES
    # ========================================================================
    print("\n2. EJECUTANDO SIMULACIONES DE VUELO")
    print("-" * 80)
    
    scenarios = [
        ("HOVER", HoverScenario(engine), 30.0),
        ("TRANSITION", TransitionScenario(engine), 15.0),
        ("CRUISE", CruiseScenario(engine), 30.0),
    ]
    
    results = {}
    
    for name, scenario, t_end in scenarios:
        print(f"\n  Simulando: {name} ({t_end}s)...", end=' ', flush=True)
        states, times = scenario.run(t_end=t_end)
        data = extract_time_series(states, times)
        results[name] = data
        print("✓")
    
    # ========================================================================
    # GRÁFICAS
    # ========================================================================
    print("\n3. GENERANDO GRÁFICAS")
    print("-" * 80)
    
    for name, data in results.items():
        print(f"  {name}...", end=' ', flush=True)
        fig = plot_results(data, scenario_name=name)
        plt.savefig(f"/home/claude/caribe-vtol-aerodynamic-analysis/{name.lower()}_results.png",
                   dpi=100, bbox_inches='tight')
        plt.close(fig)
        print("✓")
    
    # ========================================================================
    # MÉTRICAS DE DESEMPEÑO
    # ========================================================================
    print("\n4. MÉTRICAS DE DESEMPEÑO")
    print("-" * 80)
    
    for name, data in results.items():
        print(f"\n  {name}:")
        print(f"    Altura máxima: {-np.min(data['z']):.2f} m")
        print(f"    Velocidad máx: {np.max(data['V_airspeed']):.2f} m/s")
        print(f"    Ángulo pitch máx: {np.max(np.abs(np.degrees(data['theta']))):.2f}°")
        print(f"    Oscilación residual (final 5s):")
        
        t_final = data['time'][-1]
        idx_final = data['time'] >= (t_final - 5.0)
        
        theta_std = np.std(data['theta'][idx_final])
        q_std = np.std(data['q'][idx_final])
        
        print(f"      θ std: {np.degrees(theta_std):.3f}°")
        print(f"      q std: {q_std:.6f} rad/s")
    
    print("\n" + "="*80)
    print("✓ SIMULACIÓN COMPLETADA")
    print("="*80)
    print("\nArchivos generados:")
    print("  - hover_results.png")
    print("  - transition_results.png")
    print("  - cruise_results.png")
    print("\n")


if __name__ == "__main__":
    main()
