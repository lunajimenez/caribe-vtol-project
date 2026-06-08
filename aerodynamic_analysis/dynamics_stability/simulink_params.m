%% simulink_params.m
%% Generado automáticamente por simulink_export.py
%% Fecha: 2026-06-07 23:02
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
fprintf('Cargando parámetros VTOL Caribe...\n');

%% =========================================================================
%% 1. MASA E INERCIAS
%% =========================================================================
mass_kg = 8.5;  % [kg] MTOW [CONFIRMED]
Ixx = 0.35;  % [kg*m2] inercia roll [PROVISIONAL]
Iyy = 0.45;  % [kg*m2] inercia pitch [PROVISIONAL]
Izz = 0.7;  % [kg*m2] inercia yaw [PROVISIONAL]
Ixz = 0.02;  % [kg*m2] producto inercia [PROVISIONAL]

I_body = [Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz];  % tensor inercia body

%% =========================================================================
%% 2. GEOMETRÍA
%% =========================================================================
S_wing = 0.567;  % [m2] superficie alar [CONFIRMED]
b_wing = 2.3;  % [m] envergadura [DERIVED]
c_bar = 0.252;  % [m] MAC [CONFIRMED]
AR = 9.33;  % [-] aspect ratio [CONFIRMED]
S_tail = 0.1376;  % [m2] superficie cola horiz [DERIVED]
l_t = 0.79;  % [m] brazo de cola [DERIVED]
x_cg = 0.41;  % [m] CG desde morro [CONFIRMED]
x_act = 1.2;  % [m] AC cola desde morro [CONFIRMED]
i_t_deg = -2;  % [°] incidencia cola [CONFIRMED]

%% =========================================================================
%% 3. COEFICIENTES AERODINÁMICOS
%% =========================================================================
CLa_wing = 5.0596;  % [/rad] CL_alpha ala 3D [CONFIRMED]
CL0_wing = 0.3597;  % [-] CL_ ala 3D [CONFIRMED]
CLa_tail = 4.393;  % [/rad] CL_alpha cola 3D [CONFIRMED]
CMa_wing = -1.839;  % [/rad] Cm_alpha ala ref XFLR5 [CONFIRMED]
CD0 = 0.014;  % [-] arrastre parasitario crucero [DERIVED]
e_oswald = 0.78;  % [-] eficiencia Oswald [DERIVED]
K_induced = 0.043739507;  % [-] factor arrastre inducido [DERIVED]
VH = 0.761;  % [-] vol coef horiz [DERIVED]
deps_dalpha = 0.345;  % [-] deps/dalpha [DERIVED]
Cmq = -18.864483;  % [/rad] pitch damping [DERIVED]
CLq = 6.0175314;  % [/rad] lift due to q [DERIVED]

%% =========================================================================
%% 4. CONDICIONES DE TRIM EN CRUCERO
%% =========================================================================
V0 = 19.34;  % [m/s] velocidad crucero [CONFIRMED]
rho = 1.225;  % [kg/m3] densidad ISA [CONFIRMED]
g = 9.81;  % [m/s2]
alpha0_deg = 3.9839962;  % [°] AoA trim [DERIVED]
alpha0_rad = 0.069533851;  % [rad] AoA trim [DERIVED]
de0_deg = -15.033016;  % [°] elevador trim [DERIVED]
de0_rad = -0.26237563;  % [rad] elevador trim [DERIVED]
CL0_trim = 0.64192729;  % [-] CL en trim [DERIVED]
CD0_trim = 0.032023767;  % [-] CD en trim [DERIVED]
LD_trim = 20.04534;  % [-] L/D en trim [DERIVED]
T_trim = 4.1598197;  % [N] empuje de trim [DERIVED]
x_NP = 0.56115536;  % [m] punto neutro [DERIVED]
SM_pct = 59.982286;  % [%MAC] margen estático [DERIVED]

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

%% =========================================================================
%% 5. MATRICES DE ESTADO LINEALIZADAS
%% =========================================================================
%% Sistema longitudinal:  x_lon = [Deltau, Deltaw, Deltaq, Deltatheta]
%%                        u_lon = [Deltadeltae]
A_lon = [-0.02212507  +0.25484380  +0.00000000  -9.81000000;
  -1.01447777  -4.49460125  +19.26951500  +0.00000000;
  +0.00000000  -12.83279144  -8.94023860  +0.00000000;
  +0.00000000  +0.00000000  +1.00000000  +0.00000000];
B_lon = [+0.00000000;
  -6.59834117;
  -98.46191324;
  +0.00000000];

%% Sistema lateral-direccional:  x_lat = [Deltabeta, Deltap, Deltar, Delta_]
%%                               u_lat = [Deltadeltaa, Deltadeltar]
A_lat = [-0.33187611  +0.00000000  -1.00000000  +0.50723888;
  -37.68153996  -24.45227460  +6.82574856  +0.00000000;
  +25.58924675  -3.09766095  -0.92966143  +0.00000000;
  +0.00000000  +1.00000000  +0.00000000  +0.00000000];
B_lat = [+0.00000000  +0.07506721;
  +126.39500016  +16.36734637;
  +2.10088327  -24.67316393;
  +0.00000000  +0.00000000];

%% Eigenvalores longitudinales
eig_lon(1) = -6.723791 + 15.564075i;
eig_lon(2) = -6.723791 + -15.564075i;
eig_lon(3) = -0.004691 + 0.666537i;
eig_lon(4) = -0.004691 + -0.666537i;

%% Eigenvalores laterales
eig_lat(1) = -23.816946 + 0.000000i;
eig_lat(2) = -0.993641 + 5.648353i;
eig_lat(3) = -0.993641 + -5.648353i;
eig_lat(4) = 0.090415 + 0.000000i;

%% =========================================================================
%% 6. TABLAS DE LOOKUP CL(alpha), CD(alpha), Cm(alpha)
%% =========================================================================
%% Rango alpha = -15° a +15°, Deltaalpha = 1°  (31 puntos)
%% Interpolación lineal en Simulink: bloque '1-D Lookup Table'
alpha_table_deg = [-15  -14  -13  -12  -11  -10  -9  -8  -7  -6  -5  -4  -3  -2  -1  0  1  2  3  4  5  6  7  8  9  10  11  12  13  14  15];  % [°] vector de ángulos de ataque
CL_wing_table = [-0.970946  -0.884491  -0.797473  -0.709942  -0.621948  -0.533543  -0.444779  -0.355709  -0.266386  -0.176865  -0.087199  0.002557  0.092348  0.18212  0.271816  0.361383  0.450766  0.53991  0.628761  0.717267  0.805372  0.893026  0.980177  1.066773  1.152764  1.238101  1.322736  1.406622  1.489714  1.571966  1.653335];  % [-] CL ala (XFLR5 invíscido)
CD_wing_table = [0.01527  0.01527  0.01527  0.01527  0.01527  0.01527  0.01527  0.01527  0.01527  0.01527  0.01527  0.013385  0.012413  0.012276  0.012871  0.013999  0.015411  0.018215  0.021906  0.026278  0.031234  0.036683  0.042637  0.049064  0.055992  0.063515  0.072322  0.072322  0.072322  0.072322  0.072322];  % [-] CD ala (XFLR5 viscoso)
Cm_wing_table = [0.253042  0.223457  0.193395  0.162893  0.131986  0.100714  0.069114  0.037224  0.005084  -0.027267  -0.05979  -0.092446  -0.125194  -0.157995  -0.190808  -0.223594  -0.256312  -0.288924  -0.321388  -0.353667  -0.385719  -0.417507  -0.448991  -0.480133  -0.510895  -0.54124  -0.571131  -0.600531  -0.629404  -0.657716  -0.685432];  % [-] Cm ala ref XFLR5 origin
CL_tail_table = [-1.129062  -1.057414  -0.985033  -0.911965  -0.838259  -0.763963  -0.689127  -0.613803  -0.538042  -0.461897  -0.385422  -0.308671  -0.231697  -0.154558  -0.077307  -0  0.077307  0.154558  0.231697  0.308671  0.385422  0.461897  0.538042  0.613803  0.689127  0.763963  0.838259  0.911965  0.985033  1.057414  1.129062];  % [-] CL cola (XFLR5 invíscido)

%% =========================================================================
%% 7. PARÁMETROS DE CONTROL (para bloque 6DOF Simulink)
%% =========================================================================
CLde_per_deg = 11.132347;  % [/°] efectividad elevador total
CLde = 0.43176914;  % [/rad] lift por deflexión elevador [DERIVED]
CMde = -1.353562;  % [/rad] pitch por deflexión elevador [DERIVED]
Clda = 0.148;  % [/rad] roll por deflexión alerón [PROVISIONAL]
Cndr = -0.06;  % [/rad] yaw por deflexión rudder [PROVISIONAL]

%% =========================================================================
%% 8. RESUMEN EN PANTALLA
%% =========================================================================
fprintf('\n=== VTOL CARIBE -- PARÁMETROS SIMULINK ===\n');
fprintf('  Masa: %.2f kg  CG: %.3f m\n', mass_kg, x_cg);
fprintf('  V_crucero: %.2f m/s  alpha_trim: %.2f°  deltae_trim: %.2f°\n', V0, alpha0_deg, de0_deg);
fprintf('  NP: %.3f m  SM: %.1f%% MAC\n', x_NP, SM_pct);
fprintf('  L/D trim: %.2f  CD0: %.5f  e: %.3f\n', LD_trim, CD0, e_oswald);
fprintf('\nEigenvalores longitudinales:\n');
disp(eig_lon)
fprintf('Eigenvalores laterales:\n');
disp(eig_lat)
fprintf('\n[OK] simulink_params.m cargado correctamente.\n');
