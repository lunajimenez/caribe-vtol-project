import { useEffect, useRef, useState } from "react";
import { theme } from "../theme";
import type { TelemetryFrame } from "../telemetry/types";

const MAX_ENTRIES = 80;

// Synthetic STATUSTEXT messages that could appear during an AUTO mission
const STATUS_POOL: string[] = [
  "Iniciando misión AUTO",
  "Waypoint alcanzado",
  "Cambiando a siguiente waypoint",
  "Velocidad de crucero nominal",
  "GPS: 14 satélites visibles",
  "Altitud estabilizada",
  "Batería OK",
  "Parámetros de control nominal",
  "EKF estable",
  "Distancia al objetivo calculada",
  "Ángulo de rumbo ajustado",
  "Control de pitch nominal",
  "Veleta de viento estimada",
  "Misión en progreso",
  "Autopilot activo",
];

interface LogEntry {
  id: number;
  tMs: number;
  text: string;
}

let _nextId = 0;
let _lastStatusTick = 0;

export function LogPanel({ frame }: { frame: TelemetryFrame | null }) {
  const [entries, setEntries] = useState<LogEntry[]>([]);
  const bottomRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!frame) return;
    const tMs = frame.tMs;
    // Emit a status message roughly every 3 seconds (every 15 frames at 5 Hz)
    if (tMs - _lastStatusTick < 3000) return;
    _lastStatusTick = tMs;

    const text = STATUS_POOL[Math.floor((tMs / 3000) % STATUS_POOL.length)];
    const entry: LogEntry = { id: _nextId++, tMs, text };
    setEntries(prev => {
      const next = [...prev, entry];
      return next.length > MAX_ENTRIES ? next.slice(-MAX_ENTRIES) : next;
    });
  }, [frame]);

  // Auto-scroll to bottom
  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [entries]);

  function formatTime(tMs: number): string {
    const s = Math.floor(tMs / 1000);
    const ms = String(tMs % 1000).padStart(3, "0");
    const min = String(Math.floor(s / 60)).padStart(2, "0");
    const sec = String(s % 60).padStart(2, "0");
    return `${min}:${sec}.${ms}`;
  }

  return (
    <div style={{
      overflowY: "auto",
      height: "100%",
      display: "flex",
      flexDirection: "column",
      gap: "0.1rem",
    }}>
      {entries.length === 0 && (
        <span style={{ fontFamily: theme.fontMono, fontSize: "0.72rem", color: theme.textDim }}>
          Esperando telemetría…
        </span>
      )}
      {entries.map(e => (
        <div key={e.id} style={{
          display: "flex",
          gap: "0.75rem",
          fontFamily: theme.fontMono,
          fontSize: "0.72rem",
        }}>
          <span style={{ color: theme.textDim, flexShrink: 0 }}>{formatTime(e.tMs)}</span>
          <span style={{ color: theme.text }}>{e.text}</span>
        </div>
      ))}
      <div ref={bottomRef} />
    </div>
  );
}
