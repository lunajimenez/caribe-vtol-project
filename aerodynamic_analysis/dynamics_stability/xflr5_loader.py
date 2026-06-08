"""
================================================================================
CARGADOR Y PROCESADOR DE DATOS XFLR5
================================================================================
Lee los archivos .txt exportados de XFLR5 v6.61 (análisis tipo 'Plane')
y computa derivadas aerodinámicas por regresión lineal.

Archivos soportados:
  3D invíscidos (ala y cola por separado):
    Invisid 19_34 - 3D - ala.txt   |  alpha -15deg a +15deg, Deltaalpha=1deg
    Invisid 19_34 - 3D - cola.txt  |  alpha -15deg a +15deg, Deltaalpha=1deg
    Invisid 11_11 - 3D - ala.txt
    Invisid 11_11 - 3D - cola.txt

  3D viscosos (avión completo):
    Viscosidad 19_34 - 3D.txt      |  alpha -5deg a +11deg, Deltaalpha=1deg
    Viscosidad 11_11 - 3D.txt

  Perfiles 2D (XFOIL, Re=70 000):
    NACA 4412-RE70.txt   |  alpha -5deg a +20deg
    NACA 0012-RE70.txt
    SD7032-RE70.txt

Columnas 3D (13 cols):
  alpha, Beta, CL, CDi, CDv, CD, CY, Cl, Cm, Cn, Cni, QInf, XCP

Columnas 2D (12 cols):
  alpha, CL, CD, CDp, Cm, Top_Xtr, Bot_Xtr, Cpmin, Chinge, XCp
  (con línea de separador '---')

Referencia: XFLR5 v6.61 User Manual; Corda (2017) §6.5
================================================================================
"""

import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import Tuple, Dict, Optional

# Directorio de datos relativo a este archivo
_DATA_DIR = Path(__file__).parent / "Data"

# Rango lineal para regresión (zona pre-stall bien comportada)
_ALPHA_LIN_MIN = -5.0   # [deg]
_ALPHA_LIN_MAX = 11.0   # [deg]


# ============================================================================
# ESTRUCTURAS DE DATOS
# ============================================================================

@dataclass
class Polar3D:
    """Polar 3D XFLR5 (ala o cola, invíscida o viscosa)."""
    source:   str            # nombre del archivo
    V_inf:    float          # [m/s] velocidad de referencia
    alpha:    np.ndarray     # [deg]
    CL:       np.ndarray
    CDi:      np.ndarray
    CDv:      np.ndarray
    CD:       np.ndarray
    Cm:       np.ndarray
    CY:       np.ndarray     # fuerza lateral (=0 en análisis simétrico)
    Cl_roll:  np.ndarray     # momento de alabeo
    Cn:       np.ndarray     # momento de guiñada


@dataclass
class Polar2D:
    """Polar 2D XFOIL."""
    source:  str
    alpha:   np.ndarray   # [deg]
    CL:      np.ndarray
    CD:      np.ndarray
    CDp:     np.ndarray
    Cm:      np.ndarray


@dataclass
class LinearDerivatives:
    """Derivadas de la zona lineal obtenidas por regresión lineal."""
    source:    str
    alpha_min: float    # [deg] rango de regresión
    alpha_max: float    # [deg]
    CLa:       float    # [/rad]  dCL/dalpha
    CL0:       float    # [-]     CL a alpha=0deg
    CMa:       float    # [/rad]  dCm/dalpha
    CM0:       float    # [-]     Cm a alpha=0deg
    R2_CL:     float    # coeficiente de determinación de CL vs alpha
    R2_CM:     float    # coeficiente de determinación de Cm vs alpha


@dataclass
class DragPolarFit:
    """Ajuste parabólico de la polar de arrastre: CD = CD0 + K*CL2."""
    source:    str
    V_inf:     float    # [m/s]
    CD0:       float    # [-]
    K:         float    # [-]  = 1/(_*e*AR)
    e_oswald:  float    # [-]  eficiencia de Oswald
    AR:        float    # [-]  aspect ratio usado en el ajuste
    R2:        float    # coeficiente de determinación


# ============================================================================
# PARSERS DE ARCHIVO
# ============================================================================

def _parse_xflr5_3d(filepath: Path) -> Polar3D:
    """
    Lee un archivo XFLR5 3D.

    Formato: cabecera con nombres de columna, luego datos separados por
    espacios. Las líneas en blanco y las que no empiezan con número se ignoran.
    """
    rows = []
    with open(filepath, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            # Saltar cabeceras (empiezan con letra o '-')
            try:
                vals = [float(v) for v in line.split()]
                if len(vals) >= 12:
                    rows.append(vals)
            except ValueError:
                continue

    if not rows:
        raise ValueError(f"No se encontraron datos numéricos en {filepath}")

    data = np.array(rows)
    # Columnas: alpha Beta CL CDi CDv CD CY Cl Cm Cn Cni QInf XCP
    return Polar3D(
        source   = filepath.name,
        V_inf    = float(data[0, 11]),   # QInf de la primera fila
        alpha    = data[:, 0],
        CL       = data[:, 2],
        CDi      = data[:, 3],
        CDv      = data[:, 4],
        CD       = data[:, 5],
        CY       = data[:, 6],
        Cl_roll  = data[:, 7],
        Cm       = data[:, 8],
        Cn       = data[:, 9],
    )


def _parse_xfoil_2d(filepath: Path) -> Polar2D:
    """
    Lee un archivo XFOIL 2D.

    Formato: línea de separador '---', luego datos numéricos.
    Columnas: alpha CL CD CDp Cm Top_Xtr Bot_Xtr Cpmin Chinge XCp
    """
    rows = []
    header_passed = False
    with open(filepath, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            if "---" in line:
                header_passed = True
                continue
            if not header_passed:
                continue
            try:
                vals = [float(v) for v in line.split()]
                if len(vals) >= 5:
                    rows.append(vals[:5])   # alpha, CL, CD, CDp, Cm
            except ValueError:
                continue

    if not rows:
        raise ValueError(f"No se encontraron datos en {filepath}")

    data = np.array(rows)
    return Polar2D(
        source = filepath.name,
        alpha  = data[:, 0],
        CL     = data[:, 1],
        CD     = data[:, 2],
        CDp    = data[:, 3],
        Cm     = data[:, 4],
    )


# ============================================================================
# REGRESIÓN LINEAL
# ============================================================================

def _linear_regression(x: np.ndarray, y: np.ndarray) -> Tuple[float, float, float]:
    """
    Regresión lineal y = a*x + b.

    Returns: (slope, intercept, R2)
    """
    n = len(x)
    if n < 2:
        raise ValueError("Se necesitan al menos 2 puntos para regresión.")
    x_mean = np.mean(x)
    y_mean = np.mean(y)
    ss_xy = np.sum((x - x_mean) * (y - y_mean))
    ss_xx = np.sum((x - x_mean)**2)
    slope     = ss_xy / ss_xx
    intercept = y_mean - slope * x_mean
    y_pred    = slope * x + intercept
    ss_res    = np.sum((y - y_pred)**2)
    ss_tot    = np.sum((y - y_mean)**2)
    R2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0
    return float(slope), float(intercept), float(R2)


def compute_linear_derivatives(
    polar: Polar3D,
    alpha_min: float = _ALPHA_LIN_MIN,
    alpha_max: float = _ALPHA_LIN_MAX,
) -> LinearDerivatives:
    """
    Calcula CLalpha, CL0, CMalpha, CM0 por regresión lineal en la zona lineal.

    La pendiente en [/deg] se convierte a [/rad] multiplicando por 180/_.

    Args:
        polar:      datos XFLR5 3D
        alpha_min:  límite inferior del rango lineal [deg]
        alpha_max:  límite superior del rango lineal [deg]

    Returns:
        LinearDerivatives con pendientes en [/rad]
    """
    mask = (polar.alpha >= alpha_min) & (polar.alpha <= alpha_max)
    alpha_rad = np.radians(polar.alpha[mask])
    CL        = polar.CL[mask]
    Cm        = polar.Cm[mask]

    if mask.sum() < 3:
        raise ValueError(
            f"Menos de 3 puntos en rango [{alpha_min}deg, {alpha_max}deg] "
            f"para {polar.source}"
        )

    slope_CL, CL0, R2_CL = _linear_regression(alpha_rad, CL)
    slope_Cm, CM0, R2_Cm = _linear_regression(alpha_rad, Cm)

    return LinearDerivatives(
        source    = polar.source,
        alpha_min = alpha_min,
        alpha_max = alpha_max,
        CLa       = slope_CL,   # ya está en /rad
        CL0       = CL0,
        CMa       = slope_Cm,   # ya está en /rad
        CM0       = CM0,
        R2_CL     = R2_CL,
        R2_CM     = R2_Cm,
    )


def compute_drag_polar(
    polar: Polar3D,
    AR: float = 9.33,
    alpha_min: float = _ALPHA_LIN_MIN,
    alpha_max: float = _ALPHA_LIN_MAX,
) -> DragPolarFit:
    """
    Ajusta polar parabólica CD = CD0 + K*CL2 en la zona lineal.

    El ajuste se hace mediante regresión lineal de CD vs CL2
    (Corda §6.5; Nelson Ch.2).

    Args:
        polar:   datos XFLR5 3D (viscosa para obtener CDv)
        AR:      aspect ratio del ala
        alpha_min, alpha_max: rango de ajuste [deg]

    Returns:
        DragPolarFit con CD0, K, e_oswald
    """
    mask = (polar.alpha >= alpha_min) & (polar.alpha <= alpha_max)
    CL2 = polar.CL[mask]**2
    CD  = polar.CD[mask]

    # Regresión: CD = CD0 + K*CL2
    K, CD0, R2 = _linear_regression(CL2, CD)

    # Oswald efficiency: K = 1/(_*e*AR)  ->  e = 1/(_*AR*K)
    e_oswald = 1.0 / (np.pi * AR * K) if K > 1e-6 else np.nan

    return DragPolarFit(
        source   = polar.source,
        V_inf    = polar.V_inf,
        CD0      = CD0,
        K        = K,
        e_oswald = e_oswald,
        AR       = AR,
        R2       = R2,
    )


# ============================================================================
# VERIFICACIÓN DE INDEPENDENCIA DE VELOCIDAD (invíscido)
# ============================================================================

def verify_velocity_independence(
    polar_low:  Polar3D,
    polar_high: Polar3D,
    tol: float = 0.001,
) -> Tuple[bool, float]:
    """
    Verifica que los coeficientes invíscidos sean independientes de la velocidad.

    En régimen incompresible, CL, CD, Cm no dependen de V (ni de Re para
    análisis invíscido). Compara los arrays interpolados en alfa común.

    Returns:
        (is_independent, max_difference)
    """
    # Interpolar en el rango común de alfa
    alpha_common = np.intersect1d(
        np.round(polar_low.alpha, 1),
        np.round(polar_high.alpha, 1),
    )
    if len(alpha_common) == 0:
        return False, np.nan

    def interp(p: Polar3D, alphas):
        CL = np.interp(alphas, p.alpha, p.CL)
        Cm = np.interp(alphas, p.alpha, p.Cm)
        return CL, Cm

    CL_low,  Cm_low  = interp(polar_low,  alpha_common)
    CL_high, Cm_high = interp(polar_high, alpha_common)

    max_diff_CL = float(np.max(np.abs(CL_low - CL_high)))
    max_diff_Cm = float(np.max(np.abs(Cm_low - Cm_high)))
    max_diff    = max(max_diff_CL, max_diff_Cm)

    return max_diff < tol, max_diff


# ============================================================================
# INTERPOLADORES PARA USO EN TIEMPO REAL
# ============================================================================

class AeroTableWing:
    """
    Tabla de lookup para CL(alpha) y CD(alpha) del ala.

    Usa datos invíscidos para CL(alpha) y Cm(alpha), datos viscosos para CD(alpha).
    Fuera del rango de tabla usa extrapolación lineal con la pendiente
    del extremo más cercano.
    """

    def __init__(
        self,
        polar_inv:  Polar3D,    # invíscido (CL, Cm)
        polar_visc: Polar3D,    # viscoso (CD)
    ):
        self._alpha = polar_inv.alpha          # [deg]
        self._CL    = polar_inv.CL
        self._Cm    = polar_inv.Cm
        # CD del viscoso, interpolado en mismos alfas
        self._CD = np.interp(
            polar_inv.alpha,
            polar_visc.alpha,
            polar_visc.CD,
        )

    def CL(self, alpha_rad: float) -> float:
        """CL interpolado. alpha en [rad]."""
        return float(np.interp(
            np.degrees(alpha_rad), self._alpha, self._CL,
        ))

    def CD(self, alpha_rad: float) -> float:
        """CD interpolado. alpha en [rad]."""
        return float(np.interp(
            np.degrees(alpha_rad), self._alpha, self._CD,
        ))

    def Cm(self, alpha_rad: float) -> float:
        """Cm interpolado. alpha en [rad]."""
        return float(np.interp(
            np.degrees(alpha_rad), self._alpha, self._Cm,
        ))


class AeroTableTail:
    """Tabla de lookup para CL(alpha) y Cm(alpha) de la cola (sólo invíscido)."""

    def __init__(self, polar_inv: Polar3D):
        self._alpha = polar_inv.alpha
        self._CL    = polar_inv.CL
        self._Cm    = polar_inv.Cm
        self._CDi   = polar_inv.CDi

    def CL(self, alpha_eff_rad: float) -> float:
        return float(np.interp(
            np.degrees(alpha_eff_rad), self._alpha, self._CL,
        ))

    def CD(self, alpha_eff_rad: float) -> float:
        return float(np.interp(
            np.degrees(alpha_eff_rad), self._alpha, self._CDi,
        ))

    def Cm(self, alpha_eff_rad: float) -> float:
        return float(np.interp(
            np.degrees(alpha_eff_rad), self._alpha, self._Cm,
        ))


# ============================================================================
# API PÚBLICA PRINCIPAL
# ============================================================================

def load_all_data(data_dir: Path = _DATA_DIR) -> Dict:
    """
    Carga todos los archivos XFLR5 y XFOIL del directorio Data/.

    Returns:
        dict con claves:
          'wing_inv_hi'   : Polar3D ala invíscida 19.34 m/s
          'wing_inv_lo'   : Polar3D ala invíscida 11.11 m/s
          'tail_inv_hi'   : Polar3D cola invíscida 19.34 m/s
          'tail_inv_lo'   : Polar3D cola invíscida 11.11 m/s
          'visc_hi'       : Polar3D viscosa 19.34 m/s
          'visc_lo'       : Polar3D viscosa 11.11 m/s
          'naca4412_2d'   : Polar2D NACA 4412
          'naca0012_2d'   : Polar2D NACA 0012
          'deriv_wing'    : LinearDerivatives del ala
          'deriv_tail'    : LinearDerivatives de la cola
          'drag_polar_hi' : DragPolarFit 19.34 m/s
          'drag_polar_lo' : DragPolarFit 11.11 m/s
          'table_wing'    : AeroTableWing
          'table_tail'    : AeroTableTail
    """
    d = {}

    # 3D invíscidos
    d["wing_inv_hi"] = _parse_xflr5_3d(data_dir / "Invisid 19_34 - 3D - ala.txt")
    d["wing_inv_lo"] = _parse_xflr5_3d(data_dir / "Invisid 11_11 - 3D - ala.txt")
    d["tail_inv_hi"] = _parse_xflr5_3d(data_dir / "Invisid 19_34 - 3D - cola.txt")
    d["tail_inv_lo"] = _parse_xflr5_3d(data_dir / "Invisid 11_11 - 3D - cola.txt")

    # 3D viscosos
    d["visc_hi"] = _parse_xflr5_3d(data_dir / "Viscosidad 19_34 - 3D.txt")
    d["visc_lo"] = _parse_xflr5_3d(data_dir / "Viscosidad 11_11 - 3D.txt")

    # 2D perfiles
    d["naca4412_2d"] = _parse_xfoil_2d(data_dir / "NACA 4412-RE70.txt")
    d["naca0012_2d"] = _parse_xfoil_2d(data_dir / "NACA 0012-RE70.txt")

    # Derivadas lineales
    d["deriv_wing"] = compute_linear_derivatives(d["wing_inv_hi"])
    d["deriv_tail"] = compute_linear_derivatives(d["tail_inv_hi"])

    # Polar de arrastre (usando viscosa de crucero)
    d["drag_polar_hi"] = compute_drag_polar(d["visc_hi"], AR=9.33)
    d["drag_polar_lo"] = compute_drag_polar(d["visc_lo"], AR=9.33)

    # Tablas de interpolación
    d["table_wing"] = AeroTableWing(d["wing_inv_hi"], d["visc_hi"])
    d["table_tail"] = AeroTableTail(d["tail_inv_hi"])

    return d


# ============================================================================
# MAIN -- verificación y reporte
# ============================================================================

if __name__ == "__main__":
    print("=" * 64)
    print(" XFLR5 LOADER -- VERIFICACIÓN DE DATOS")
    print("=" * 64)

    data = load_all_data()

    # 1) Derivadas del ala
    dw = data["deriv_wing"]
    print(f"\n[ALA -- NACA 4412, 3D, invíscido, rango {dw.alpha_min}deg a {dw.alpha_max}deg]")
    print(f"  CLalpha  = {dw.CLa:8.4f} /rad   R2 = {dw.R2_CL:.6f}")
    print(f"  CL0  = {dw.CL0:8.4f}        (esperado ~= 0.3597)")
    print(f"  CMalpha  = {dw.CMa:8.4f} /rad   R2 = {dw.R2_CM:.6f}")
    print(f"  CM0  = {dw.CM0:8.4f}        (esperado ~= -0.2229)")

    # 2) Derivadas de la cola
    dt = data["deriv_tail"]
    print(f"\n[COLA -- NACA 0012, 3D, invíscido, rango {dt.alpha_min}deg a {dt.alpha_max}deg]")
    print(f"  CLalpha  = {dt.CLa:8.4f} /rad   R2 = {dt.R2_CL:.6f}")
    print(f"  CL0  = {dt.CL0:8.4f}        (esperado ~= 0)")
    print(f"  CMalpha  = {dt.CMa:8.4f} /rad   R2 = {dt.R2_CM:.6f}")

    # 3) Independencia de velocidad (invíscido)
    ok_w, diff_w = verify_velocity_independence(
        data["wing_inv_lo"], data["wing_inv_hi"]
    )
    ok_t, diff_t = verify_velocity_independence(
        data["tail_inv_lo"], data["tail_inv_hi"]
    )
    print(f"\n[INDEPENDENCIA DE VELOCIDAD -- invíscido]")
    print(f"  Ala:  max |DeltaCL, DeltaCm| = {diff_w:.5f}  {'[OK] OK' if ok_w else '[FAIL] FALLA'}")
    print(f"  Cola: max |DeltaCL, DeltaCm| = {diff_t:.5f}  {'[OK] OK' if ok_t else '[FAIL] FALLA'}")

    # 4) Polar de arrastre
    dp_hi = data["drag_polar_hi"]
    dp_lo = data["drag_polar_lo"]
    print(f"\n[POLAR DE ARRASTRE -- ajuste CD = CD0 + K*CL2]")
    print(f"  19.34 m/s: CD0 = {dp_hi.CD0:.5f}  K = {dp_hi.K:.5f}"
          f"  e = {dp_hi.e_oswald:.3f}  R2 = {dp_hi.R2:.6f}")
    print(f"  11.11 m/s: CD0 = {dp_lo.CD0:.5f}  K = {dp_lo.K:.5f}"
          f"  e = {dp_lo.e_oswald:.3f}  R2 = {dp_lo.R2:.6f}")

    # 5) Test de interpolación de tablas
    alpha_test = np.radians(3.0)
    tw = data["table_wing"]
    print(f"\n[TEST TABLA ALA @ alpha=3deg]")
    print(f"  CL = {tw.CL(alpha_test):.5f}  CD = {tw.CD(alpha_test):.5f}"
          f"  Cm = {tw.Cm(alpha_test):.5f}")

    print("\n[OK] xflr5_loader.py OK")
