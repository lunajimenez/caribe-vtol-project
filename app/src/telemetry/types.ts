export type FlightMode =
  | "QHOVER" | "QLOITER" | "QSTABILIZE"
  | "FBWA" | "CRUISE" | "AUTO" | "RTL";

export interface Waypoint {
  seq: number;
  lat: number;
  lon: number;
  altRel: number; // m above takeoff point
}

export interface TelemetryFrame {
  tMs: number;           // monotonic timestamp (ms)

  // Attitude — MAVLink ATTITUDE
  rollDeg: number;
  pitchDeg: number;
  yawDeg: number;

  // Position — MAVLink GLOBAL_POSITION_INT
  lat: number;
  lon: number;
  altRelM: number;
  altAmslM: number;

  // Speeds — MAVLink VFR_HUD
  groundSpeedMs: number;
  verticalSpeedMs: number;

  // Energy — MAVLink BATTERY_STATUS
  battVoltage: number;        // V
  battCurrent: number;        // A
  battRemainingPct: number;   // 0..100

  // GPS — MAVLink GPS_RAW_INT
  gpsFix: "NO_FIX" | "2D" | "3D" | "RTK";
  gpsSats: number;

  // State — MAVLink HEARTBEAT
  mode: FlightMode;
  armed: boolean;

  // Mission — MAVLink MISSION_CURRENT
  wpCurrent: number;
  wpTotal: number;
  distToWpM: number;
}

export type Role = "invitado" | "operador";
