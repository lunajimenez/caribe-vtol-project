import { theme } from "../theme";
import type { TelemetryFrame, Role } from "../telemetry/types";

interface Props {
  frame: TelemetryFrame | null;
  role: Role;
}

export function TopBar({ frame, role }: Props) {
  const linked = frame !== null;
  const armed = frame?.armed ?? false;
  const mode = frame?.mode ?? "—";

  return (
    <header style={{
      background: theme.panel,
      borderBottom: `1px solid ${theme.border}`,
      display: "flex",
      alignItems: "center",
      gap: "1.5rem",
      padding: "0 1.25rem",
      height: "3rem",
      flexShrink: 0,
    }}>
      {/* Identity */}
      <span style={{ fontFamily: theme.fontMono, fontSize: "0.85rem", color: theme.accent, letterSpacing: "0.08em" }}>
        CARIBE VTOL
      </span>

      <div style={{ width: 1, height: "1.5rem", background: theme.border }} />

      {/* Link status */}
      <Pill color={linked ? theme.ok : theme.danger} label={linked ? "ENLACE OK" : "SIN ENLACE"} />

      {/* Flight mode */}
      <span style={{ fontFamily: theme.fontMono, fontSize: "0.78rem", color: theme.text }}>
        {mode}
      </span>

      {/* Armed state */}
      <Pill color={armed ? theme.warn : theme.textDim} label={armed ? "ARMADO" : "DESARMADO"} />

      <div style={{ flex: 1 }} />

      {/* Role badge */}
      <span style={{
        fontFamily: theme.fontSans,
        fontSize: "0.7rem",
        color: role === "operador" ? theme.accent : theme.textDim,
        background: theme.card,
        border: `1px solid ${role === "operador" ? theme.accent : theme.border}`,
        borderRadius: "0.25rem",
        padding: "0.15rem 0.55rem",
        textTransform: "uppercase",
        letterSpacing: "0.06em",
      }}>
        {role}
      </span>
    </header>
  );
}

function Pill({ color, label }: { color: string; label: string }) {
  return (
    <span style={{
      display: "inline-flex",
      alignItems: "center",
      gap: "0.35rem",
      fontFamily: theme.fontMono,
      fontSize: "0.72rem",
      color,
    }}>
      <span style={{ width: 6, height: 6, borderRadius: "50%", background: color, display: "inline-block" }} />
      {label}
    </span>
  );
}
