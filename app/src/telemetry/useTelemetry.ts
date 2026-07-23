import { createContext, useContext, useEffect, useState, useRef, createElement } from "react";
import type { ReactNode } from "react";
import { MockTelemetrySource } from "./MockTelemetrySource";
import type { TelemetrySource } from "./TelemetrySource";
import type { TelemetryFrame, Waypoint } from "./types";

interface TelemetryContextValue {
  frame: TelemetryFrame | null;
  mission: Waypoint[];
}

const TelemetryContext = createContext<TelemetryContextValue>({
  frame: null,
  mission: [],
});

// Single change point: swap MockTelemetrySource for a real source here in Phase 1.
function createSource(): TelemetrySource {
  return new MockTelemetrySource();
}

export function TelemetryProvider({ children }: { children: ReactNode }) {
  const [frame, setFrame] = useState<TelemetryFrame | null>(null);
  const sourceRef = useRef<TelemetrySource>(createSource());
  const mission = sourceRef.current.getMission();

  useEffect(() => {
    const src = sourceRef.current;
    src.start(setFrame);
    return () => src.stop();
  }, []);

  return createElement(
    TelemetryContext.Provider,
    { value: { frame, mission } },
    children
  );
}

export function useTelemetry(): TelemetryContextValue {
  return useContext(TelemetryContext);
}
