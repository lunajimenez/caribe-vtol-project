import { theme } from "./theme";
import { useTelemetry } from "./telemetry/useTelemetry";
import { useRole } from "./access/useRole";
import { TopBar } from "./components/TopBar";
import { MissionMap } from "./components/MissionMap";
import { MissionPanel } from "./components/MissionPanel";
import { TelemetryGrid } from "./components/TelemetryGrid";
import { AttitudeMini } from "./components/AttitudeMini";
import { LogPanel } from "./components/LogPanel";

export function App() {
  const { frame, mission } = useTelemetry();
  const role = useRole();

  return (
    <div style={{
      display: "flex",
      flexDirection: "column",
      height: "100vh",
      background: theme.bg,
      color: theme.text,
      fontFamily: theme.fontSans,
      overflow: "hidden",
    }}>
      <TopBar frame={frame} role={role} />

      {/* Main content */}
      <div style={{
        display: "flex",
        flex: 1,
        overflow: "hidden",
        minHeight: 0,
      }}>
        {/* Left: map (protagonist) */}
        <div style={{ flex: "2 1 0", minWidth: 0, position: "relative" }}>
          <MissionMap frame={frame} mission={mission} />
        </div>

        {/* Right sidebar */}
        <div style={{
          flex: "1 1 0",
          minWidth: 220,
          maxWidth: 360,
          display: "flex",
          flexDirection: "column",
          borderLeft: `1px solid ${theme.border}`,
          overflow: "hidden",
        }}>
          {/* Mission panel */}
          <div style={{
            flex: "1 1 0",
            overflow: "hidden",
            padding: "0.75rem",
            borderBottom: `1px solid ${theme.border}`,
          }}>
            <MissionPanel frame={frame} mission={mission} />
          </div>

          {/* Attitude instrument */}
          <div style={{
            flexShrink: 0,
            padding: "0.75rem",
            display: "flex",
            justifyContent: "center",
            borderBottom: `1px solid ${theme.border}`,
          }}>
            <AttitudeMini frame={frame} />
          </div>
        </div>
      </div>

      {/* Telemetry data row */}
      <div style={{
        borderTop: `1px solid ${theme.border}`,
        padding: "0.6rem 0.75rem",
        background: theme.panel,
        flexShrink: 0,
      }}>
        <TelemetryGrid frame={frame} />
      </div>

      {/* Log panel */}
      <div style={{
        borderTop: `1px solid ${theme.border}`,
        padding: "0.4rem 0.75rem",
        height: "5.5rem",
        background: theme.panel,
        flexShrink: 0,
      }}>
        <div style={{ fontFamily: theme.fontSans, fontSize: "0.65rem", color: theme.textDim, textTransform: "uppercase", letterSpacing: "0.06em", marginBottom: "0.2rem" }}>
          Log
        </div>
        <LogPanel frame={frame} />
      </div>
    </div>
  );
}
