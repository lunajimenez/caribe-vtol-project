import { theme } from "../theme";
import type { TelemetryFrame } from "../telemetry/types";

interface Props {
  frame: TelemetryFrame | null;
}

function battColor(pct: number): string {
  if (pct < 20) return theme.danger;
  if (pct < 40) return theme.warn;
  return theme.ok;
}

function gpsColor(fix: TelemetryFrame["gpsFix"]): string {
  if (fix === "NO_FIX") return theme.danger;
  if (fix === "2D") return theme.warn;
  return theme.ok;
}

interface CardProps {
  label: string;
  value: string;
  unit?: string;
  valueColor?: string;
  sub?: string;
}

function Card({ label, value, unit, valueColor = theme.text, sub }: CardProps) {
  return (
    <div style={{
      background: theme.card,
      border: `1px solid ${theme.border}`,
      borderRadius: "0.35rem",
      padding: "0.6rem 0.75rem",
      display: "flex",
      flexDirection: "column",
      gap: "0.2rem",
      minWidth: 0,
    }}>
      <span style={{ fontFamily: theme.fontSans, fontSize: "0.65rem", color: theme.textDim, textTransform: "uppercase", letterSpacing: "0.05em" }}>
        {label}
      </span>
      <span style={{ fontFamily: theme.fontMono, fontSize: "1rem", color: valueColor, lineHeight: 1.1 }}>
        {value}
        {unit && <span style={{ fontSize: "0.7rem", color: theme.textDim, marginLeft: "0.2rem" }}>{unit}</span>}
      </span>
      {sub && (
        <span style={{ fontFamily: theme.fontMono, fontSize: "0.65rem", color: theme.textDim }}>{sub}</span>
      )}
    </div>
  );
}

export function TelemetryGrid({ frame }: Props) {
  const f = frame;

  const battPct = f?.battRemainingPct ?? 0;
  const gpsFix = f?.gpsFix ?? "NO_FIX";

  return (
    <div style={{
      display: "grid",
      gridTemplateColumns: "repeat(auto-fill, minmax(130px, 1fr))",
      gap: "0.5rem",
    }}>
      <Card
        label="Batería"
        value={f ? f.battRemainingPct.toFixed(0) : "—"}
        unit="%"
        valueColor={f ? battColor(battPct) : theme.textDim}
        sub={f ? `${f.battVoltage.toFixed(1)} V · ${f.battCurrent.toFixed(1)} A` : undefined}
      />
      <Card
        label="Altitud rel."
        value={f ? f.altRelM.toFixed(1) : "—"}
        unit="m"
        sub={f ? `AMSL ${f.altAmslM.toFixed(1)} m` : undefined}
      />
      <Card
        label="Vel. suelo"
        value={f ? f.groundSpeedMs.toFixed(1) : "—"}
        unit="m/s"
      />
      <Card
        label="Vel. vertical"
        value={f ? (f.verticalSpeedMs >= 0 ? "+" : "") + f.verticalSpeedMs.toFixed(2) : "—"}
        unit="m/s"
        valueColor={f && Math.abs(f.verticalSpeedMs) > 1.5 ? theme.warn : theme.text}
      />
      <Card
        label="GPS"
        value={f ? gpsFix : "—"}
        valueColor={f ? gpsColor(gpsFix) : theme.textDim}
        sub={f ? `${f.gpsSats} sats` : undefined}
      />
      <Card
        label="Próx. WP"
        value={f ? String(f.wpCurrent) : "—"}
        sub={f ? `${f.distToWpM.toFixed(0)} m` : undefined}
      />
      <Card
        label="Roll"
        value={f ? f.rollDeg.toFixed(1) : "—"}
        unit="°"
        valueColor={f && Math.abs(f.rollDeg) > 20 ? theme.warn : theme.text}
      />
      <Card
        label="Pitch"
        value={f ? f.pitchDeg.toFixed(1) : "—"}
        unit="°"
        valueColor={f && Math.abs(f.pitchDeg) > 15 ? theme.warn : theme.text}
      />
    </div>
  );
}
