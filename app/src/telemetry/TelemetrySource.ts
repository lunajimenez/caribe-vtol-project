import type { TelemetryFrame, Waypoint } from "./types";

export type FrameListener = (f: TelemetryFrame) => void;

export interface TelemetrySource {
  start(onFrame: FrameListener): void;
  stop(): void;
  getMission(): Waypoint[];
}
