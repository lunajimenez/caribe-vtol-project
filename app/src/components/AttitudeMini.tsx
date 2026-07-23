import { theme } from "../theme";
import type { TelemetryFrame } from "../telemetry/types";

interface Props {
  frame: TelemetryFrame | null;
}

const SIZE = 120;
const CX = SIZE / 2;
const CY = SIZE / 2;
const R = SIZE / 2 - 4;

export function AttitudeMini({ frame }: Props) {
  const roll = frame?.rollDeg ?? 0;
  const pitch = frame?.pitchDeg ?? 0;

  // Pitch shifts the horizon vertically: 1° ≈ 1px at this scale
  const pitchShiftPx = Math.max(-R, Math.min(R, pitch * 1.5));

  return (
    <div style={{ display: "flex", flexDirection: "column", alignItems: "center", gap: "0.4rem" }}>
      <span style={{ fontFamily: theme.fontSans, fontSize: "0.65rem", color: theme.textDim, textTransform: "uppercase", letterSpacing: "0.05em" }}>
        Actitud
      </span>

      <svg width={SIZE} height={SIZE} style={{ display: "block" }}>
        <defs>
          <clipPath id="att-clip">
            <circle cx={CX} cy={CY} r={R} />
          </clipPath>
        </defs>

        {/* Rotating sky/ground disc */}
        <g clipPath="url(#att-clip)">
          <g transform={`rotate(${-roll}, ${CX}, ${CY})`}>
            {/* Sky */}
            <rect x={CX - R - 4} y={CY - R - 4 + pitchShiftPx - R * 2} width={(R + 4) * 2} height={R * 2} fill="#1a3a5c" />
            {/* Ground */}
            <rect x={CX - R - 4} y={CY + pitchShiftPx} width={(R + 4) * 2} height={R * 2 + 8} fill="#3a2a10" />
            {/* Horizon line */}
            <line x1={CX - R - 4} y1={CY + pitchShiftPx} x2={CX + R + 4} y2={CY + pitchShiftPx}
              stroke={theme.text} strokeWidth={1.5} />
            {/* Pitch ladder marks */}
            {[-10, 10].map(deg => {
              const y = CY + pitchShiftPx - deg * 1.5;
              return (
                <g key={deg}>
                  <line x1={CX - 16} y1={y} x2={CX + 16} y2={y} stroke={theme.text} strokeWidth={0.8} opacity={0.5} />
                  <text x={CX + 18} y={y + 3} fontSize={8} fill={theme.textDim} fontFamily={theme.fontMono}>
                    {Math.abs(deg)}
                  </text>
                </g>
              );
            })}
          </g>
        </g>

        {/* Fixed aircraft symbol */}
        <g>
          <line x1={CX - 22} y1={CY} x2={CX - 8} y2={CY} stroke={theme.accent} strokeWidth={2} />
          <line x1={CX + 8} y1={CY} x2={CX + 22} y2={CY} stroke={theme.accent} strokeWidth={2} />
          <circle cx={CX} cy={CY} r={2.5} fill={theme.accent} />
        </g>

        {/* Bezel */}
        <circle cx={CX} cy={CY} r={R} fill="none" stroke={theme.border} strokeWidth={2} />

        {/* Roll pointer at top */}
        <polygon
          points={`${CX},${CY - R + 6} ${CX - 4},${CY - R - 2} ${CX + 4},${CY - R - 2}`}
          fill={theme.textDim}
          transform={`rotate(${-roll}, ${CX}, ${CY})`}
        />
      </svg>

      <div style={{ display: "flex", gap: "0.75rem" }}>
        <MiniStat label="R" value={`${roll.toFixed(1)}°`} />
        <MiniStat label="P" value={`${pitch.toFixed(1)}°`} />
        <MiniStat label="Hdg" value={`${(frame?.yawDeg ?? 0).toFixed(0)}°`} />
      </div>
    </div>
  );
}

function MiniStat({ label, value }: { label: string; value: string }) {
  return (
    <span style={{ fontFamily: theme.fontMono, fontSize: "0.7rem", color: theme.textDim }}>
      <span style={{ color: theme.textDim }}>{label} </span>
      <span style={{ color: theme.text }}>{value}</span>
    </span>
  );
}
