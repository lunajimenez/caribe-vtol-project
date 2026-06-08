"""
================================================================================
SIMULADOR NUMÉRICO -- RESPUESTA A PERTURBACIONES
================================================================================
Integra las ecuaciones 6 DOF (RK4) para tres escenarios de validación:

  1. Escalón de elevador  ->  excita Short Period + Phugoid
  2. Perturbación en beta    ->  excita Dutch Roll + Spiral
  3. Pulso de alerón      ->  excita Roll Mode

Para cada escenario se genera una gráfica en plots/ que compara la
respuesta simulada con los modos analíticos (envolvente exponencial).

Dependencias: numpy, matplotlib
================================================================================
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")   # backend sin ventana -- compatible con scripts batch
import matplotlib.pyplot as plt
from pathlib import Path
from typing import List, Tuple

from parameters import AircraftConfig, get_aircraft_config
from dynamics_6dof import Dynamics6DOF, State, Controls
from stability_analysis import StabilityAnalyzer, phugoid_analytic

_PLOTS = Path(__file__).parent / "plots"


# ============================================================================
# FUNCIONES DE AYUDA
# ============================================================================

def _extract(states: List[State], times: np.ndarray) -> dict:
    """Extrae series temporales de la lista de estados."""
    return {
        "t":     times,
        "u":     np.array([s.u     for s in states]),
        "v":     np.array([s.v     for s in states]),
        "w":     np.array([s.w     for s in states]),
        "p":     np.array([s.p     for s in states]),
        "q":     np.array([s.q     for s in states]),
        "r":     np.array([s.r     for s in states]),
        "phi":   np.array([s.phi   for s in states]),
        "theta": np.array([s.theta for s in states]),
        "psi":   np.array([s.psi   for s in states]),
        "V":     np.array([s.V     for s in states]),
        "alpha": np.array([s.alpha for s in states]),
        "beta":  np.array([s.beta  for s in states]),
        "alt":   np.array([s.altitude for s in states]),
    }


def _trim_state(cfg: AircraftConfig) -> State:
    """Estado trimado en crucero."""
    from stability_analysis import StaticStability
    trim = StaticStability(cfg).trim_cruise()
    alpha_t = trim["alpha_trim_rad"]
    V0      = cfg.V_cruise
    return State(
        u     = V0 * np.cos(alpha_t),
        w     = V0 * np.sin(alpha_t),
        theta = alpha_t,
    )


def _trim_throttle(cfg: AircraftConfig) -> float:
    """Throttle de trim para equilibrar arrastre."""
    from stability_analysis import StaticStability
    trim = StaticStability(cfg).trim_throttle_frac() if hasattr(
        StaticStability(cfg), "trim_throttle_frac") else None
    if trim is None:
        st = StaticStability(cfg).trim_cruise()
        T  = st["T_trim_N"]
        return np.clip(T / cfg.prop.T_max_pusher, 0.0, 1.0)
    return trim


# ============================================================================
# ESCENARIO 1 -- ESCALÓN DE ELEVADOR (Short Period + Phugoid)
# ============================================================================

def scenario_elevator_step(
    cfg: AircraftConfig,
    dyn: Dynamics6DOF,
    de_step: float = np.radians(5.0),
    t_end:   float = 120.0,
    dt:      float = 0.05,
) -> dict:
    """
    Respuesta a escalón de elevador de +5° aplicado en t=2 s.

    Parámetros elegidos para ver ambos modos:
      Short Period: transitorio rápido en los primeros ~5 s
      Phugoid:      oscilación lenta visible en V y theta en t > 10 s
    """
    x0       = _trim_state(cfg)
    th_trim  = _trim_throttle(cfg)
    de_trim  = StaticStability_trim_de(cfg)

    def ctrl(t: float, s: State) -> Controls:
        de = de_trim + (de_step if t >= 2.0 else 0.0)
        return Controls(
            delta_e  = de,
            throttle = th_trim,
        )

    states, times = dyn.simulate(x0, ctrl, t_end=t_end, dt=dt)
    return _extract(states, times)


def StaticStability_trim_de(cfg):
    from stability_analysis import StaticStability
    return StaticStability(cfg).trim_cruise()["delta_e_trim_rad"]


# ============================================================================
# ESCENARIO 2 -- PERTURBACIÓN EN beta (Dutch Roll + Spiral)
# ============================================================================

def scenario_beta_perturbation(
    cfg: AircraftConfig,
    dyn: Dynamics6DOF,
    beta0:  float = np.radians(5.0),
    t_end:  float = 60.0,
    dt:     float = 0.02,
) -> dict:
    """
    Respuesta a perturbación inicial en ángulo de resbalamiento beta=+5°.

    Excita los modos laterales: Dutch Roll (oscilatorio en beta, _, r)
    y Spiral (divergencia lenta en _ si es inestable).
    """
    x0_trim = _trim_state(cfg)
    th_trim = _trim_throttle(cfg)
    de_trim = StaticStability_trim_de(cfg)

    # Perturbación: v != 0 -> beta = arcsin(v/V)
    V0  = cfg.V_cruise
    x0  = State(
        u     = x0_trim.u,
        v     = V0 * np.sin(beta0),
        w     = x0_trim.w,
        theta = x0_trim.theta,
    )

    def ctrl(t: float, s: State) -> Controls:
        return Controls(delta_e=de_trim, throttle=th_trim)

    states, times = dyn.simulate(x0, ctrl, t_end=t_end, dt=dt)
    return _extract(states, times)


# ============================================================================
# ESCENARIO 3 -- PULSO DE ALERÓN (Roll Mode)
# ============================================================================

def scenario_aileron_pulse(
    cfg: AircraftConfig,
    dyn: Dynamics6DOF,
    da_pulse: float = np.radians(10.0),
    t_pulse:  float = 0.5,
    t_end:    float = 20.0,
    dt:       float = 0.02,
) -> dict:
    """
    Pulso de alerón de +10° durante 0.5 s, luego neutro.

    Excita el Roll Mode (decaimiento exponencial de p).
    """
    x0      = _trim_state(cfg)
    th_trim = _trim_throttle(cfg)
    de_trim = StaticStability_trim_de(cfg)

    def ctrl(t: float, s: State) -> Controls:
        da = da_pulse if t <= t_pulse else 0.0
        return Controls(delta_e=de_trim, delta_a=da, throttle=th_trim)

    states, times = dyn.simulate(x0, ctrl, t_end=t_end, dt=dt)
    return _extract(states, times)


# ============================================================================
# GRÁFICAS DE VALIDACIÓN
# ============================================================================

def plot_elevator_step(data: dict, results: dict, filename: str = "elevator_step.png"):
    """Gráfica escalón elevador: V(t), alpha(t), theta(t), q(t)."""
    _PLOTS.mkdir(exist_ok=True)
    t = data["t"]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Respuesta Escalón Elevador (+5°)  --  Short Period + Phugoid",
                 fontweight="bold")

    axes[0, 0].plot(t, data["V"],               "b-", lw=1.5)
    axes[0, 0].axhline(data["V"][0], ls="--",  color="k", alpha=0.4)
    axes[0, 0].set_ylabel("V [m/s]")
    axes[0, 0].set_xlabel("t [s]")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_title("Velocidad aerodinámica")

    axes[0, 1].plot(t, np.degrees(data["alpha"]), "r-", lw=1.5)
    axes[0, 1].set_ylabel("alpha [°]")
    axes[0, 1].set_xlabel("t [s]")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_title("Ángulo de ataque")

    axes[1, 0].plot(t, np.degrees(data["theta"]), "g-", lw=1.5)
    axes[1, 0].set_ylabel("theta [°]")
    axes[1, 0].set_xlabel("t [s]")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_title("Ángulo de pitch")

    axes[1, 1].plot(t, np.degrees(data["q"]),    "m-", lw=1.5)
    axes[1, 1].set_ylabel("q [°/s]")
    axes[1, 1].set_xlabel("t [s]")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_title("Velocidad angular de pitch")

    # Superponer envolvente analítica del Phugoid si disponible
    ph = results.get("phugoid_analytic")
    if ph and np.isfinite(ph["period_s"]):
        zeta = ph["zeta"]; wn = ph["omega_n"]
        t_ph = np.linspace(0, t[-1], 500)
        A_ph = abs(data["V"][-1] - data["V"][0]) * 0.5
        env  = data["V"][0] + A_ph * np.exp(-zeta * wn * t_ph) * np.cos(
            wn * np.sqrt(max(1 - zeta**2, 0.0)) * t_ph)
        axes[0, 0].plot(t_ph, env, "k--", alpha=0.6, label=f"Phugoid analítico\nT={ph['period_s']:.1f}s")
        axes[0, 0].legend(fontsize=8)

    plt.tight_layout()
    fig.savefig(str(_PLOTS / filename), dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  Guardado: {_PLOTS / filename}")


def plot_beta_perturbation(data: dict, filename: str = "beta_perturbation.png"):
    """Gráfica perturbación beta: beta(t), _(t), r(t), p(t)."""
    _PLOTS.mkdir(exist_ok=True)
    t = data["t"]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Perturbación beta=+5°  --  Dutch Roll + Spiral", fontweight="bold")

    pairs = [
        (axes[0, 0], np.degrees(data["beta"]),  "beta [°]",    "Ángulo de resbalamiento"),
        (axes[0, 1], np.degrees(data["phi"]),   "_ [°]",    "Ángulo de roll"),
        (axes[1, 0], np.degrees(data["r"]),     "r [°/s]",  "Yaw rate"),
        (axes[1, 1], np.degrees(data["p"]),     "p [°/s]",  "Roll rate"),
    ]
    colors = ["b", "r", "g", "m"]
    for (ax, y, ylabel, title), c in zip(pairs, colors):
        ax.plot(t, y, color=c, lw=1.5)
        ax.axhline(0.0, ls="--", color="k", alpha=0.3)
        ax.set_ylabel(ylabel); ax.set_xlabel("t [s]")
        ax.grid(True, alpha=0.3); ax.set_title(title)

    plt.tight_layout()
    fig.savefig(str(_PLOTS / filename), dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  Guardado: {_PLOTS / filename}")


def plot_aileron_pulse(data: dict, results: dict, filename: str = "aileron_pulse.png"):
    """Gráfica pulso alerón: p(t), _(t) con envolvente del Roll Mode."""
    _PLOTS.mkdir(exist_ok=True)
    t = data["t"]

    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    fig.suptitle("Pulso Alerón +10° (0.5 s)  --  Roll Mode", fontweight="bold")

    axes[0].plot(t, np.degrees(data["p"]),   "b-", lw=1.5, label="p simulado")
    axes[1].plot(t, np.degrees(data["phi"]), "r-", lw=1.5, label="_ simulado")

    # Envolvente Roll Mode: p(t) ~ p_max * exp(lambda_roll * t)
    lm = results.get("lat_modes", {})
    rm = lm.get("roll_mode")
    if rm and np.isfinite(rm.get("sigma", np.nan)):
        sigma_r = rm["sigma"]
        tau_r   = rm.get("tau_s", 1.0 / abs(sigma_r) if abs(sigma_r) > 1e-6 else np.inf)
        t_env   = np.linspace(0, t[-1], 400)
        p_max   = np.max(np.abs(np.degrees(data["p"][:50])))  # pico inicial
        env_p   = p_max * np.exp(sigma_r * t_env)
        axes[0].plot(t_env, env_p,  "k--", alpha=0.6,
                     label=f"Roll mode tau={tau_r:.2f}s")
        axes[0].plot(t_env, -env_p, "k--", alpha=0.6)

    for ax, lbl in zip(axes, ["p [°/s]", "_ [°]"]):
        ax.axhline(0, ls="--", color="k", alpha=0.3)
        ax.set_xlabel("t [s]"); ax.set_ylabel(lbl)
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    plt.tight_layout()
    fig.savefig(str(_PLOTS / filename), dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  Guardado: {_PLOTS / filename}")


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    print("=" * 64)
    print(" SIMULADOR VTOL -- ESCENARIOS DE PERTURBACIÓN")
    print("=" * 64)

    cfg      = get_aircraft_config()
    dyn      = Dynamics6DOF(cfg)
    analyzer = StabilityAnalyzer(cfg)
    results  = analyzer.run()

    from stability_analysis import print_stability_report
    print_stability_report(results)

    print("\nEjecutando escenarios de simulación...")

    # 1) Escalón de elevador
    print("\n  [1/3] Escalón de elevador (+5°, 120 s)...", end=" ", flush=True)
    data_elev = scenario_elevator_step(cfg, dyn)
    plot_elevator_step(data_elev, results)
    print("OK")

    # 2) Perturbación beta
    print("  [2/3] Perturbación beta=+5° (60 s)...",  end=" ", flush=True)
    data_beta = scenario_beta_perturbation(cfg, dyn)
    plot_beta_perturbation(data_beta)
    print("OK")

    # 3) Pulso de alerón
    print("  [3/3] Pulso alerón +10° (20 s)...",   end=" ", flush=True)
    data_ail  = scenario_aileron_pulse(cfg, dyn)
    plot_aileron_pulse(data_ail, results)
    print("OK")

    print(f"\nGráficas generadas en: {_PLOTS}/")
    print("  - elevator_step.png")
    print("  - beta_perturbation.png")
    print("  - aileron_pulse.png")
    print("\n[OK] simulator.py OK")
