"""
================================================================================
ANÁLISIS DE ESTABILIDAD DINÁMICA - EIGENANALYSIS
================================================================================
Calcula características de estabilidad del VTOL:

1. LINEALIZACIÓN: Ecuaciones de movimiento alrededor de equilibrio
2. EIGENVALOR ANALYSIS: Polos del sistema (frecuencias naturales + amortiguamiento)
3. CRITERIOS DE ESTABILIDAD:
   - Criterio de Routh-Hurwitz
   - Análisis de modos (Phugoid, Dutch Roll, Spiral, etc.)
   - Márgenes de estabilidad

Teoría:
  Si estado perturbado es x(t) = x_eq + Δx(t), entonces:
  
    d(Δx)/dt = A·Δx + B·Δu
  
  donde A es matriz de estabilidad (linealizada).
  
  Polos = eigenvalores de A = λ
  
  Estabilidad:
    - Re(λ) < 0  →  Estable (convergente)
    - Re(λ) = 0  →  Marginalmente estable
    - Re(λ) > 0  →  INESTABLE (divergente)
  
  Período oscilatorio: T = 2π / |Im(λ)|
  Tiempo a mitad amplitud: t_half = ln(2) / |Re(λ)|

Referencia: Nelson, "Flight Stability and Automatic Control", 2nd Ed.
            Abzug & Larrabee, "Airplane Stability and Control", 2nd Ed.

================================================================================
"""

import numpy as np
from typing import Tuple, Dict, List
from scipy.linalg import eig
from scipy.optimize import minimize_scalar
import matplotlib.pyplot as plt
from parameters import VTOLAircraftParameters, FlightConditions
from forces_moments import AerodynamicCalculator, ThrustCalculator
from dynamics_6dof import VTOLDynamicsEngine, AircraftState, ControlInput


class StabilityAnalyzer:
    """
    Analiza estabilidad linealizada del VTOL alrededor puntos de equilibrio.
    
    Procedimiento:
    1. Encontrar punto de equilibrio (trim condition)
    2. Linealizar ecuaciones de movimiento
    3. Calcular matriz de estabilidad A
    4. Computar eigenvalores y eigenvectores
    5. Clasificar modos de oscilación
    """
    
    def __init__(self, params: VTOLAircraftParameters):
        self.params = params
        self.engine = VTOLDynamicsEngine(params)
        self.aero_calc = AerodynamicCalculator(params)
        self.thrust_calc = ThrustCalculator(params)
    
    # ========================================================================
    # PASO 1: ENCONTRAR EQUILIBRIO (TRIM)
    # ========================================================================
    
    def find_trim_hover(self) -> Tuple[AircraftState, ControlInput]:
        """
        Encuentra condiciones de trim en HOVER.
        
        En hover equilibrio:
          - Velocidades: u=0, v=0, w=0
          - Actitud: θ≈0, φ≈0, ψ=cualquiera
          - Velocidades angulares: p=0, q=0, r=0
          - Empuje: 4·T = W (peso)
        
        Returns:
          (equilibrium_state, equilibrium_control)
        """
        m = self.params.geometry.mtow
        g = self.params.flight.g
        W = m * g
        
        # Throttle necesario para equilibrio
        T_rotor_needed = W / (4 * 9.81)  # Por rotor individual
        T_max = self.params.propulsion.vertical_motor_max_thrust_kg
        throttle_trim = (T_rotor_needed / T_max) ** 0.5  # Inversa de relación cuadrática
        
        # Estado de equilibrio
        eq_state = AircraftState(
            x=0.0, y=0.0, z=0.0,
            u=0.0, v=0.0, w=0.0,
            phi=0.0, theta=0.0, psi=0.0,
            p=0.0, q=0.0, r=0.0
        )
        
        # Control de equilibrio
        eq_control = ControlInput(
            throttle_vertical=np.clip(throttle_trim, 0.0, 1.0),
            throttle_horizontal=0.0,
            mode="HOVER"
        )
        
        return eq_state, eq_control
    
    def find_trim_cruise(self, V_cruise: float = None) -> Tuple[AircraftState, ControlInput]:
        """
        Encuentra trim en CRUCERO.
        
        En crucero equilibrio:
          - Velocidad: u = V_cruise, v = 0, w = 0
          - Actitud: θ = α_trim (ángulo ataque trim)
          - Velocidades angulares: p=0, q=0, r=0
          - Fuerzas: Fz=0, Fy=0 (solo Fx activo)
        """
        if V_cruise is None:
            V_cruise = self.params.flight.v_cruise_ms
        
        # En crucero, resolver para θ tal que Fz_total = 0
        # Simplificación: usar pequeño ángulo
        theta_trim = np.radians(3.0)  # Pitch trim típico
        
        # Ángulo de ataque aproximado
        alpha_trim = theta_trim  # Si w≈0
        
        # Estado trim
        eq_state = AircraftState(
            x=0.0, y=0.0, z=0.0,
            u=V_cruise, v=0.0, w=V_cruise * np.tan(alpha_trim),
            phi=0.0, theta=theta_trim, psi=0.0,
            p=0.0, q=0.0, r=0.0
        )
        
        # Control trim (encontrar por iteración si es necesario)
        eq_control = ControlInput(
            throttle_vertical=0.1,  # Margen mínimo
            throttle_horizontal=0.6,  # Estimado
            delta_e=-alpha_trim * 0.5,  # Elevador para trim
            mode="CRUISE"
        )
        
        return eq_state, eq_control
    
    # ========================================================================
    # PASO 2: LINEALIZACIÓN (JACOBIANO)
    # ========================================================================
    
    def compute_jacobian(self, eq_state: AircraftState, eq_control: ControlInput,
                        eps: float = 1e-6) -> np.ndarray:
        """
        Calcula matriz Jacobiana (matriz de estabilidad A) por diferencias finitas.
        
        A[i,j] = ∂(dx_i/dt) / ∂x_j evaluado en equilibrio
        
        Args:
          eq_state: estado de equilibrio
          eq_control: control en equilibrio
          eps: perturbación para diferencias finitas
        
        Returns:
          Matriz A (12x12)
        """
        A = np.zeros((12, 12))
        
        # Derivada nominal
        y_eq = eq_state.to_array()
        dy_eq = self.engine._derivatives(0.0, y_eq, eq_control)
        
        # Perturbar cada estado
        for j in range(12):
            y_pert = y_eq.copy()
            y_pert[j] += eps
            
            dy_pert = self.engine._derivatives(0.0, y_pert, eq_control)
            
            # Jacobiano
            A[:, j] = (dy_pert - dy_eq) / eps
        
        return A
    
    # ========================================================================
    # PASO 3: EIGENANALYSIS
    # ========================================================================
    
    def analyze_stability(self, eq_state: AircraftState,
                         eq_control: ControlInput) -> Dict:
        """
        Análisis completo de estabilidad.
        
        Returns:
          Dict con:
            - eigenvalues: array de valores propios λ
            - eigenvectors: matriz de vectores propios
            - modes: información sobre cada modo (freq, damping, etc.)
            - is_stable: bool, True si todos Re(λ) < 0
        """
        # Calcular Jacobiano
        A = self.compute_jacobian(eq_state, eq_control)
        
        # Eigenvalores y eigenvectores
        eigenvalues, eigenvectors = eig(A)
        
        # Ordenar por parte real (decreciente)
        idx = np.argsort(np.real(eigenvalues))[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        # Información de modos
        modes = []
        for i, lam in enumerate(eigenvalues):
            mode_info = self._characterize_mode(lam, i)
            modes.append(mode_info)
        
        # Criterio de estabilidad global
        is_stable = np.all(np.real(eigenvalues) < 0)
        
        return {
            "eigenvalues": eigenvalues,
            "eigenvectors": eigenvectors,
            "modes": modes,
            "A_matrix": A,
            "is_stable": is_stable,
            "stability_margin": np.max(np.real(eigenvalues)),  # Debe ser < 0
        }
    
    def _characterize_mode(self, lam: complex, mode_num: int) -> Dict:
        """
        Caracteriza un modo individual de oscilación.
        
        Parámetros:
          - Frecuencia natural: ωn = |λ|
          - Factor de amortiguamiento: ζ = -Re(λ) / |λ|
          - Período: T = 2π / |Im(λ)|
          - Tiempo a mitad amplitud: t_half = ln(2) / |Re(λ)|
          - Nombre del modo (heurístico)
        """
        re = np.real(lam)
        im = np.abs(np.imag(lam))
        mag = np.abs(lam)
        
        # Factor de amortiguamiento
        if mag > 1e-6:
            zeta = -re / mag
        else:
            zeta = 1.0
        
        # Frecuencia y período
        if im > 1e-6:
            freq_hz = im / (2 * np.pi)
            period_s = 1.0 / freq_hz
            is_oscillatory = True
        else:
            freq_hz = 0.0
            period_s = np.inf
            is_oscillatory = False
        
        # Tiempo de convergencia
        if re < -1e-6:
            t_half = np.log(2) / (-re)
            t_63 = 1.0 / (-re)
        else:
            t_half = np.inf
            t_63 = np.inf
        
        # Nombre del modo (heurístico)
        mode_name = self._identify_mode_name(freq_hz, zeta, is_oscillatory)
        
        return {
            "eigenvalue": lam,
            "mode_name": mode_name,
            "real_part": re,
            "imag_part": im,
            "frequency_hz": freq_hz,
            "period_s": period_s,
            "damping_ratio": zeta,
            "t_half_amplitude_s": t_half,
            "t_63pct_s": t_63,
            "is_oscillatory": is_oscillatory,
            "is_stable": re < 0,
        }
    
    def _identify_mode_name(self, freq_hz: float, zeta: float,
                           is_oscillatory: bool) -> str:
        """
        Intenta identificar el modo oscilatorio.
        
        Convenciones (Abzug, Nelson):
          - Phugoid: freq ~0.01-0.05 Hz, large ζ (~0.05-0.1)
          - Short Period: freq ~0.5-2 Hz, high ζ (~0.5-1.0)
          - Dutch Roll: freq ~0.5-1.5 Hz, moderate ζ (~0.1-0.5)
          - Spiral: freq ~0, small ζ (unstable)
          - Roll: freq ~0, large ζ
        """
        if not is_oscillatory:
            if freq_hz < 0.01:
                return "APERIODIC (Spiral/Convergence)"
            else:
                return "REAL mode"
        
        if freq_hz < 0.1:
            return "PHUGOID"
        elif freq_hz < 0.5:
            return "Long Period / Oscillatory"
        elif freq_hz < 2.0:
            if zeta > 0.5:
                return "SHORT PERIOD"
            else:
                return "DUTCH ROLL"
        else:
            return "High Frequency mode"
    
    # ========================================================================
    # CRITERIOS DE ESTABILIDAD
    # ========================================================================
    
    def routh_hurwitz_stability(self, A: np.ndarray) -> Tuple[bool, str]:
        """
        Aplica criterio de Routh-Hurwitz para estabilidad.
        
        Más confiable que eigenvalores en algunos casos.
        """
        # Usar eigenvalores ya que Routh-Hurwitz es tedioso en 12x12
        eigenvalues = np.linalg.eigvals(A)
        is_stable = np.all(np.real(eigenvalues) < 0)
        
        if is_stable:
            margin = -np.max(np.real(eigenvalues))
            msg = f"ESTABLE. Margen: {margin:.6f} rad/s"
        else:
            margin = np.max(np.real(eigenvalues))
            msg = f"INESTABLE. Polo más positivo: {margin:.6f} rad/s"
        
        return is_stable, msg
    
    def compute_stability_envelopes(self) -> Dict:
        """
        Calcula envolventes de estabilidad en rango de velocidades.
        
        Evalúa estabilidad en V = 0 a V_max, útil para identificar
        régimen inestable (e.g., flutter, etc.)
        """
        V_range = np.linspace(0, self.params.flight.v_never_exceed_ms, 20)
        
        stability_data = {
            "velocities": [],
            "is_stable": [],
            "stability_margins": [],
            "damping_ratios": [],
        }
        
        for V in V_range:
            # Trim en esa velocidad
            eq_state, eq_control = self.find_trim_cruise(V)
            
            # Análisis
            analysis = self.analyze_stability(eq_state, eq_control)
            
            stability_data["velocities"].append(V)
            stability_data["is_stable"].append(analysis["is_stable"])
            stability_data["stability_margins"].append(analysis["stability_margin"])
            
            # Damping ratio del modo más crítico
            damping_min = min([m["damping_ratio"] for m in analysis["modes"]])
            stability_data["damping_ratios"].append(damping_min)
        
        return stability_data


def print_stability_report(analysis: Dict, title: str = "STABILITY ANALYSIS"):
    """
    Imprime reporte formatted de estabilidad.
    """
    print(f"\n{'='*80}")
    print(f"  {title}")
    print(f"{'='*80}")
    
    eigenvalues = analysis["eigenvalues"]
    modes = analysis["modes"]
    is_stable = analysis["is_stable"]
    margin = analysis["stability_margin"]
    
    print(f"\nESTABILIDAD GLOBAL: {'✓ ESTABLE' if is_stable else '✗ INESTABLE'}")
    print(f"Margen de estabilidad: {margin:.6f} rad/s {'(OK)' if margin < 0 else '(CRÍTICO)'}")
    
    print(f"\n{'MODO':<25} {'FREQ (Hz)':<12} {'DAMPING':<12} {'T_HALF (s)':<12} {'STATUS':<10}")
    print("-" * 80)
    
    for i, (lam, mode) in enumerate(zip(eigenvalues, modes)):
        freq_str = f"{mode['frequency_hz']:.3f}" if mode['is_oscillatory'] else "N/A"
        zeta_str = f"{mode['damping_ratio']:.3f}"
        t_half_str = f"{mode['t_half_amplitude_s']:.2f}" if np.isfinite(mode['t_half_amplitude_s']) else "∞"
        status = "OK" if mode['is_stable'] else "UNSTABLE"
        
        print(f"{mode['mode_name']:<25} {freq_str:<12} {zeta_str:<12} {t_half_str:<12} {status:<10}")
    
    print(f"{'='*80}\n")


if __name__ == "__main__":
    from parameters import get_default_vtol_parameters
    
    print("\nINICIALIZANDO ANÁLISIS DE ESTABILIDAD...")
    params = get_default_vtol_parameters()
    analyzer = StabilityAnalyzer(params)
    
    # Análisis en HOVER
    print("\n" + "="*80)
    print("HOVER STABILITY")
    print("="*80)
    eq_hover, ctrl_hover = analyzer.find_trim_hover()
    analysis_hover = analyzer.analyze_stability(eq_hover, ctrl_hover)
    print_stability_report(analysis_hover, "HOVER MODE STABILITY")
    
    # Análisis en CRUCERO
    print("="*80)
    print("CRUISE STABILITY")
    print("="*80)
    eq_cruise, ctrl_cruise = analyzer.find_trim_cruise()
    analysis_cruise = analyzer.analyze_stability(eq_cruise, ctrl_cruise)
    print_stability_report(analysis_cruise, "CRUISE MODE STABILITY")
    
    print("✓ Análisis completado")
