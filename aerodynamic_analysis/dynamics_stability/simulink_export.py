"""
================================================================================
EXPORTADOR DE PARÁMETROS PARA MATLAB / SIMULINK
================================================================================
Genera un script .m con:
  * Matrices A_lon, B_lon, A_lat, B_lat
  * Parámetros de masa, inercias y geometría para bloque 6DOF (Euler Angles)
  * Tablas lookup CL(alpha), CD(alpha), Cm(alpha) del ala y la cola
  * Condiciones iniciales para simulación de crucero

Uso desde MATLAB:
  >> run('simulink_params.m')
  >> % Luego cargar en el bloque 6DOF de Simulink

El archivo se guarda en el mismo directorio que este script.
================================================================================
"""

import numpy as np
from pathlib import Path
from datetime import datetime

from parameters import AircraftConfig, get_aircraft_config
from stability_analysis import StabilityAnalyzer
from xflr5_loader import load_all_data


_OUT_FILE = Path(__file__).parent / "simulink_params.m"


# ============================================================================
# FORMATEADORES MATLAB
# ============================================================================

def _mat(arr: np.ndarray, name: str, indent: int = 0) -> str:
    """Convierte un array numpy a una asignación de matriz MATLAB."""
    sp = " " * indent
    if arr.ndim == 1:
        vals = "  ".join(f"{v:+.8f}" for v in arr)
        return f"{sp}{name} = [{vals}]';\n"
    rows = []
    for row in arr:
        rows.append("  ".join(f"{v:+.8f}" for v in row))
    body = ";\n  ".join(rows)
    return f"{sp}{name} = [{body}];\n"


def _scalar(val: float, name: str, comment: str = "", indent: int = 0) -> str:
    sp  = " " * indent
    cmt = f"  % {comment}" if comment else ""
    return f"{sp}{name} = {val:.8g};{cmt}\n"


def _vec(arr: np.ndarray, name: str, comment: str = "") -> str:
    cmt = f"  % {comment}" if comment else ""
    vals = "  ".join(f"{v:.8g}" for v in arr)
    return f"{name} = [{vals}];{cmt}\n"


# ============================================================================
# GENERADOR PRINCIPAL
# ============================================================================

def export_simulink(cfg: AircraftConfig = None, out_path: Path = _OUT_FILE) -> Path:
    """
    Genera simulink_params.m con todos los parámetros necesarios.

    Args:
        cfg:      AircraftConfig (usa default si None)
        out_path: ruta de salida del archivo .m

    Returns:
        Path del archivo generado
    """
    cfg      = cfg or get_aircraft_config()
    analyzer = StabilityAnalyzer(cfg)
    results  = analyzer.run()
    data     = load_all_data()

    A_lon = results["A_lon"]
    B_lon = results["B_lon"]
    A_lat = results["A_lat"]
    B_lat = results["B_lat"]

    trim  = results["static"]["trim"]
    stat  = results["static"]
    m     = cfg.mass
    w     = cfg.wing
    ht    = cfg.htail
    vt    = cfg.vtail
    a     = cfg.aero
    atm   = cfg.atm

    # Tablas de lookup (usar datos invíscidos del ala)
    tw   = data["table_wing"]
    tt   = data["table_tail"]
    alpha_table_deg = np.linspace(-15.0, 15.0, 31)
    alpha_table_rad = np.radians(alpha_table_deg)
    CL_table  = np.array([tw.CL(ar) for ar in alpha_table_rad])
    CD_table  = np.array([tw.CD(ar) for ar in alpha_table_rad])
    Cm_table  = np.array([tw.Cm(ar) for ar in alpha_table_rad])
    CLt_table = np.array([tt.CL(ar) for ar in alpha_table_rad])

    now = datetime.now().strftime("%Y-%m-%d %H:%M")

    lines = []
    lines.append(f"""%% simulink_params.m
%% Generado automáticamente por simulink_export.py
%% Fecha: {now}
%% Proyecto: VTOL Caribe -- Universidad Tecnológica de Bolívar
%%
%% Uso:
%%   run('simulink_params.m')
%%   % Luego asignar al bloque 6DOF (Euler Angles) de Simulink:
%%   %   Masa       ->  mass_kg
%%   %   Inercias   ->  I_body
%%   %   IC velocid ->  [u0 v0 w0]
%%   %   IC actitud ->  [phi0 theta0 psi0]
%%
%% Nota: valores [PROVISIONAL] marcados con comentario P
%%       valores [CONFIRMED/DERIVED] sin marca

clear; clc;
fprintf('Cargando parámetros VTOL Caribe...\\n');

%% =========================================================================
%% 1. MASA E INERCIAS
%% =========================================================================
""")
    lines.append(_scalar(m.m,   "mass_kg",   "[kg] MTOW [CONFIRMED]"))
    lines.append(_scalar(m.Ixx, "Ixx",       "[kg*m2] inercia roll [PROVISIONAL]"))
    lines.append(_scalar(m.Iyy, "Iyy",       "[kg*m2] inercia pitch [PROVISIONAL]"))
    lines.append(_scalar(m.Izz, "Izz",       "[kg*m2] inercia yaw [PROVISIONAL]"))
    lines.append(_scalar(m.Ixz, "Ixz",       "[kg*m2] producto inercia [PROVISIONAL]"))
    lines.append("\nI_body = [Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz];  % tensor inercia body\n")

    lines.append("""
%% =========================================================================
%% 2. GEOMETRÍA
%% =========================================================================
""")
    lines.append(_scalar(w.S,      "S_wing",    "[m2] superficie alar [CONFIRMED]"))
    lines.append(_scalar(w.b,      "b_wing",    "[m] envergadura [DERIVED]"))
    lines.append(_scalar(w.c_bar,  "c_bar",     "[m] MAC [CONFIRMED]"))
    lines.append(_scalar(w.AR,     "AR",        "[-] aspect ratio [CONFIRMED]"))
    lines.append(_scalar(ht.S_t,   "S_tail",    "[m2] superficie cola horiz [DERIVED]"))
    lines.append(_scalar(cfg.lt,   "l_t",       "[m] brazo de cola [DERIVED]"))
    lines.append(_scalar(m.x_cg,   "x_cg",      "[m] CG desde morro [CONFIRMED]"))
    lines.append(_scalar(ht.x_ac_t,"x_act",     "[m] AC cola desde morro [CONFIRMED]"))
    lines.append(_scalar(np.degrees(ht.i_t), "i_t_deg", "[°] incidencia cola [CONFIRMED]"))

    lines.append("""
%% =========================================================================
%% 3. COEFICIENTES AERODINÁMICOS
%% =========================================================================
""")
    lines.append(_scalar(a.CLa_w,       "CLa_wing",    "[/rad] CL_alpha ala 3D [CONFIRMED]"))
    lines.append(_scalar(a.CL0_w,       "CL0_wing",    "[-] CL_ ala 3D [CONFIRMED]"))
    lines.append(_scalar(a.CLa_t,       "CLa_tail",    "[/rad] CL_alpha cola 3D [CONFIRMED]"))
    lines.append(_scalar(a.CMa_w,       "CMa_wing",    "[/rad] Cm_alpha ala ref XFLR5 [CONFIRMED]"))
    lines.append(_scalar(a.CD0_cruise,  "CD0",         "[-] arrastre parasitario crucero [DERIVED]"))
    lines.append(_scalar(a.e_oswald,    "e_oswald",    "[-] eficiencia Oswald [DERIVED]"))
    lines.append(_scalar(cfg.K,         "K_induced",   "[-] factor arrastre inducido [DERIVED]"))
    lines.append(_scalar(a.VH,          "VH",          "[-] vol coef horiz [DERIVED]"))
    lines.append(_scalar(a.deps_da,     "deps_dalpha", "[-] deps/dalpha [DERIVED]"))
    lines.append(_scalar(a.Cmq,         "Cmq",         "[/rad] pitch damping [DERIVED]"))
    lines.append(_scalar(a.CLq,         "CLq",         "[/rad] lift due to q [DERIVED]"))

    lines.append("""
%% =========================================================================
%% 4. CONDICIONES DE TRIM EN CRUCERO
%% =========================================================================
""")
    lines.append(_scalar(cfg.V_cruise,           "V0",           "[m/s] velocidad crucero [CONFIRMED]"))
    lines.append(_scalar(cfg.atm.rho,            "rho",          "[kg/m3] densidad ISA [CONFIRMED]"))
    lines.append(_scalar(cfg.atm.g,              "g",            "[m/s2]"))
    lines.append(_scalar(trim["alpha_trim_deg"],  "alpha0_deg",   "[°] AoA trim [DERIVED]"))
    lines.append(_scalar(trim["alpha_trim_rad"],  "alpha0_rad",   "[rad] AoA trim [DERIVED]"))
    lines.append(_scalar(trim["delta_e_trim_deg"],"de0_deg",      "[°] elevador trim [DERIVED]"))
    lines.append(_scalar(trim["delta_e_trim_rad"],"de0_rad",      "[rad] elevador trim [DERIVED]"))
    lines.append(_scalar(trim["CL_trim"],         "CL0_trim",     "[-] CL en trim [DERIVED]"))
    lines.append(_scalar(trim["CD_trim"],         "CD0_trim",     "[-] CD en trim [DERIVED]"))
    lines.append(_scalar(trim["LD_trim"],         "LD_trim",      "[-] L/D en trim [DERIVED]"))
    lines.append(_scalar(trim["T_trim_N"],        "T_trim",       "[N] empuje de trim [DERIVED]"))
    lines.append(_scalar(stat["x_NP_m"],          "x_NP",         "[m] punto neutro [DERIVED]"))
    lines.append(_scalar(stat["SM_pct"],          "SM_pct",       "[%MAC] margen estático [DERIVED]"))

    lines.append("""
%% Condiciones iniciales (IC) para bloque Simulink 6DOF
alpha0 = alpha0_rad;
u0  = V0 * cos(alpha0);   % [m/s] vel adelante
v0  = 0.0;                 % [m/s] vel lateral
w0  = V0 * sin(alpha0);   % [m/s] vel vertical body
phi0   = 0.0;              % [rad] roll
theta0 = alpha0;           % [rad] pitch = AoA en vuelo nivelado
psi0   = 0.0;              % [rad] yaw
x0_pos = [0; 0; 0];       % [m] posición inicial NED
IC_vel_body = [u0; v0; w0];
IC_euler    = [phi0; theta0; psi0];
""")

    lines.append("""
%% =========================================================================
%% 5. MATRICES DE ESTADO LINEALIZADAS
%% =========================================================================
%% Sistema longitudinal:  x_lon = [Deltau, Deltaw, Deltaq, Deltatheta]
%%                        u_lon = [Deltadeltae]
""")
    lines.append(_mat(A_lon, "A_lon"))
    lines.append(_mat(B_lon, "B_lon"))

    lines.append("""
%% Sistema lateral-direccional:  x_lat = [Deltabeta, Deltap, Deltar, Delta_]
%%                               u_lat = [Deltadeltaa, Deltadeltar]
""")
    lines.append(_mat(A_lat, "A_lat"))
    lines.append(_mat(B_lat, "B_lat"))

    # Eigenvalores
    eig_lon = np.linalg.eigvals(A_lon)
    eig_lat = np.linalg.eigvals(A_lat)
    lines.append("\n%% Eigenvalores longitudinales\n")
    for i, ev in enumerate(eig_lon):
        lines.append(f"eig_lon({i+1}) = {ev.real:.6f} + {ev.imag:.6f}i;\n")
    lines.append("\n%% Eigenvalores laterales\n")
    for i, ev in enumerate(eig_lat):
        lines.append(f"eig_lat({i+1}) = {ev.real:.6f} + {ev.imag:.6f}i;\n")

    lines.append("""
%% =========================================================================
%% 6. TABLAS DE LOOKUP CL(alpha), CD(alpha), Cm(alpha)
%% =========================================================================
%% Rango alpha = -15° a +15°, Deltaalpha = 1°  (31 puntos)
%% Interpolación lineal en Simulink: bloque '1-D Lookup Table'
""")
    lines.append(_vec(alpha_table_deg, "alpha_table_deg", "[°] vector de ángulos de ataque"))
    lines.append(_vec(CL_table,        "CL_wing_table",   "[-] CL ala (XFLR5 invíscido)"))
    lines.append(_vec(CD_table,        "CD_wing_table",   "[-] CD ala (XFLR5 viscoso)"))
    lines.append(_vec(Cm_table,        "Cm_wing_table",   "[-] Cm ala ref XFLR5 origin"))
    lines.append(_vec(CLt_table,       "CL_tail_table",   "[-] CL cola (XFLR5 invíscido)"))

    lines.append("""
%% =========================================================================
%% 7. PARÁMETROS DE CONTROL (para bloque 6DOF Simulink)
%% =========================================================================
""")
    lines.append(_scalar(np.degrees(ht.tau_e * a.CLde), "CLde_per_deg",
                         "[/°] efectividad elevador total"))
    lines.append(_scalar(a.CLde,  "CLde",  "[/rad] lift por deflexión elevador [DERIVED]"))
    lines.append(_scalar(a.CMde,  "CMde",  "[/rad] pitch por deflexión elevador [DERIVED]"))
    lines.append(_scalar(a.Clda,  "Clda",  "[/rad] roll por deflexión alerón [PROVISIONAL]"))
    lines.append(_scalar(a.Cndr,  "Cndr",  "[/rad] yaw por deflexión rudder [PROVISIONAL]"))

    lines.append("""
%% =========================================================================
%% 8. RESUMEN EN PANTALLA
%% =========================================================================
fprintf('\\n=== VTOL CARIBE -- PARÁMETROS SIMULINK ===\\n');
fprintf('  Masa: %.2f kg  CG: %.3f m\\n', mass_kg, x_cg);
fprintf('  V_crucero: %.2f m/s  alpha_trim: %.2f°  deltae_trim: %.2f°\\n', V0, alpha0_deg, de0_deg);
fprintf('  NP: %.3f m  SM: %.1f%% MAC\\n', x_NP, SM_pct);
fprintf('  L/D trim: %.2f  CD0: %.5f  e: %.3f\\n', LD_trim, CD0, e_oswald);
fprintf('\\nEigenvalores longitudinales:\\n');
disp(eig_lon)
fprintf('Eigenvalores laterales:\\n');
disp(eig_lat)
fprintf('\\n[OK] simulink_params.m cargado correctamente.\\n');
""")

    content = "".join(lines)
    out_path.write_text(content, encoding="utf-8")
    return out_path


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    print("=" * 64)
    print(" SIMULINK EXPORT -- Generando simulink_params.m")
    print("=" * 64)

    cfg  = get_aircraft_config()
    path = export_simulink(cfg)

    print(f"\n[OK] Archivo generado: {path}")
    print(f"  Tamaño: {path.stat().st_size / 1024:.1f} KB")
    print("\nContenido incluido:")
    print("  * Masa, inercias, geometría")
    print("  * Condiciones de trim en crucero")
    print("  * Matrices A_lon, B_lon, A_lat, B_lat")
    print("  * Eigenvalores de los 5 modos dinámicos")
    print("  * Tablas CL(alpha), CD(alpha), Cm(alpha)  [31 puntos, -15° a +15°]")
    print("  * Parámetros de control (CLde, CMde, Clda, Cndr)")
    print("\n=== SIMULINK PARAMETERS EXPORTED ===")
    print(f"File: {path.name}")
