import { theme } from "../theme";
import type { TelemetryFrame, Waypoint } from "../telemetry/types";

interface Props {
  frame: TelemetryFrame | null;
  mission: Waypoint[];
}

function fmt(n: number, dec = 0): string {
  return n.toFixed(dec);
}

export function MissionPanel({ frame, mission }: Props) {
  const wpCurrent = frame?.wpCurrent ?? 0;
  const wpTotal = frame ? frame.wpTotal - 1 : mission.length - 1; // exclude home
  const distToWpM = frame?.distToWpM ?? 0;
  const speed = frame?.groundSpeedMs ?? 0;
  const etaSec = speed > 0.5 ? distToWpM / speed : null;

  const progress = wpTotal > 0 ? Math.min(wpCurrent / wpTotal, 1) : 0;

  return (
    <div style={{ display: "flex", flexDirection: "column", gap: "0.75rem", height: "100%", overflow: "hidden" }}>
      <div style={{ fontFamily: theme.fontSans, fontSize: "0.7rem", color: theme.textDim, textTransform: "uppercase", letterSpacing: "0.06em" }}>
        Misión
      </div>

      {/* Progress bar */}
      <div>
        <div style={{ display: "flex", justifyContent: "space-between", marginBottom: "0.3rem" }}>
          <span style={{ fontFamily: theme.fontMono, fontSize: "0.75rem", color: theme.text }}>
            WP {wpCurrent} / {wpTotal}
          </span>
          <span style={{ fontFamily: theme.fontMono, fontSize: "0.75rem", color: theme.textDim }}>
            {fmt(progress * 100)}%
          </span>
        </div>
        <div style={{ height: 4, background: theme.border, borderRadius: 2 }}>
          <div style={{
            height: "100%",
            width: `${progress * 100}%`,
            background: theme.accent,
            borderRadius: 2,
            transition: "width 0.3s ease",
          }} />
        </div>
      </div>

      {/* Dist + ETA */}
      <div style={{ display: "flex", gap: "1rem" }}>
        <Stat label="Dist WP" value={`${fmt(distToWpM)} m`} />
        <Stat label="ETA" value={etaSec !== null ? `${fmt(etaSec)} s` : "—"} />
      </div>

      <div style={{ width: "100%", height: 1, background: theme.border }} />

      {/* Waypoint list */}
      <div style={{ overflowY: "auto", flex: 1, display: "flex", flexDirection: "column", gap: "0.2rem" }}>
        {mission.map(wp => {
          const isActive = frame ? frame.wpCurrent === wp.seq : false;
          return (
            <div key={wp.seq} style={{
              display: "flex",
              alignItems: "center",
              gap: "0.5rem",
              padding: "0.25rem 0.4rem",
              borderRadius: "0.2rem",
              background: isActive ? `${theme.accent}18` : "transparent",
              border: `1px solid ${isActive ? theme.accent : "transparent"}`,
            }}>
              <span style={{
                fontFamily: theme.fontMono,
                fontSize: "0.7rem",
                color: isActive ? theme.accent : theme.textDim,
                minWidth: "2rem",
              }}>
                {String(wp.seq).padStart(2, "0")}
              </span>
              <span style={{ fontFamily: theme.fontMono, fontSize: "0.7rem", color: theme.textDim, flex: 1 }}>
                {wp.lat.toFixed(4)}, {wp.lon.toFixed(4)}
              </span>
              <span style={{ fontFamily: theme.fontMono, fontSize: "0.7rem", color: theme.textDim }}>
                {wp.altRel} m
              </span>
            </div>
          );
        })}
      </div>
    </div>
  );
}

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div style={{ display: "flex", flexDirection: "column", gap: "0.15rem" }}>
      <span style={{ fontFamily: theme.fontSans, fontSize: "0.65rem", color: theme.textDim, textTransform: "uppercase" }}>{label}</span>
      <span style={{ fontFamily: theme.fontMono, fontSize: "0.85rem", color: theme.text }}>{value}</span>
    </div>
  );
}
