// ALL values in this file are SYNTHETIC — generated for Phase 0 only.
// No real aircraft data is used here.

import type { TelemetrySource, FrameListener } from "./TelemetrySource";
import type { TelemetryFrame, Waypoint } from "./types";

// [PROVISIONAL] Base location: Cartagena de Indias, Colombia
const BASE_LAT = 10.3997;
const BASE_LON = -75.5144;

// [CONFIRMED] Cruise speed from project parameters
const CRUISE_SPEED_MS = 19.34;

// [PROVISIONAL] Voltage range
const VOLT_FULL = 16.8;
const VOLT_EMPTY = 14.0;

// Synthetic 8-waypoint mission loop around Cartagena bay
const MISSION: Waypoint[] = [
  { seq: 0, lat: BASE_LAT,          lon: BASE_LON,          altRel: 0    }, // home / takeoff
  { seq: 1, lat: BASE_LAT + 0.009,  lon: BASE_LON + 0.006,  altRel: 48   },
  { seq: 2, lat: BASE_LAT + 0.018,  lon: BASE_LON + 0.002,  altRel: 48   },
  { seq: 3, lat: BASE_LAT + 0.022,  lon: BASE_LON - 0.008,  altRel: 48   },
  { seq: 4, lat: BASE_LAT + 0.015,  lon: BASE_LON - 0.018,  altRel: 48   },
  { seq: 5, lat: BASE_LAT + 0.004,  lon: BASE_LON - 0.022,  altRel: 48   },
  { seq: 6, lat: BASE_LAT - 0.006,  lon: BASE_LON - 0.014,  altRel: 48   },
  { seq: 7, lat: BASE_LAT + 0.000,  lon: BASE_LON - 0.004,  altRel: 30   }, // approach
];

const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

function haversineM(lat1: number, lon1: number, lat2: number, lon2: number): number {
  const R = 6_371_000;
  const dLat = (lat2 - lat1) * DEG_TO_RAD;
  const dLon = (lon2 - lon1) * DEG_TO_RAD;
  const a = Math.sin(dLat / 2) ** 2 +
    Math.cos(lat1 * DEG_TO_RAD) * Math.cos(lat2 * DEG_TO_RAD) * Math.sin(dLon / 2) ** 2;
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

function bearingDeg(lat1: number, lon1: number, lat2: number, lon2: number): number {
  const dLon = (lon2 - lon1) * DEG_TO_RAD;
  const y = Math.sin(dLon) * Math.cos(lat2 * DEG_TO_RAD);
  const x = Math.cos(lat1 * DEG_TO_RAD) * Math.sin(lat2 * DEG_TO_RAD) -
    Math.sin(lat1 * DEG_TO_RAD) * Math.cos(lat2 * DEG_TO_RAD) * Math.cos(dLon);
  return (Math.atan2(y, x) * RAD_TO_DEG + 360) % 360;
}

// Segment lengths (meters) between consecutive waypoints
function buildSegments(): { dist: number; bearing: number }[] {
  const segs: { dist: number; bearing: number }[] = [];
  for (let i = 0; i < MISSION.length - 1; i++) {
    const a = MISSION[i];
    const b = MISSION[i + 1];
    segs.push({
      dist: haversineM(a.lat, a.lon, b.lat, b.lon),
      bearing: bearingDeg(a.lat, a.lon, b.lat, b.lon),
    });
  }
  return segs;
}

const SEGMENTS = buildSegments();
// [DERIVED] Total mission distance in meters
const TOTAL_DIST_M = SEGMENTS.reduce((s, seg) => s + seg.dist, 0);
// [DERIVED] Total mission duration at cruise speed
const TOTAL_DURATION_S = TOTAL_DIST_M / CRUISE_SPEED_MS;

function smoothNoise(t: number, freq: number, amp: number): number {
  return amp * Math.sin(t * freq * DEG_TO_RAD * 50);
}

export class MockTelemetrySource implements TelemetrySource {
  private timer: ReturnType<typeof setInterval> | null = null;
  private tMs = 0;

  getMission(): Waypoint[] {
    return MISSION;
  }

  start(onFrame: FrameListener): void {
    this.tMs = 0;
    this.timer = setInterval(() => {
      onFrame(this.buildFrame());
      this.tMs += 200;
    }, 200);
  }

  stop(): void {
    if (this.timer !== null) {
      clearInterval(this.timer);
      this.timer = null;
    }
  }

  private buildFrame(): TelemetryFrame {
    const t = this.tMs;
    const tSec = t / 1000;

    // Progress along mission: 0..1, cyclic
    const missionProgress = (tSec % TOTAL_DURATION_S) / TOTAL_DURATION_S;
    const traveledM = missionProgress * TOTAL_DIST_M;

    // Find current segment
    let accumulated = 0;
    let segIdx = 0;
    for (let i = 0; i < SEGMENTS.length; i++) {
      if (accumulated + SEGMENTS[i].dist >= traveledM) {
        segIdx = i;
        break;
      }
      accumulated += SEGMENTS[i].dist;
      segIdx = i;
    }

    const seg = SEGMENTS[segIdx];
    const withinSeg = Math.min(traveledM - accumulated, seg.dist);
    const segT = seg.dist > 0 ? withinSeg / seg.dist : 0;

    const wpA = MISSION[segIdx];
    const wpB = MISSION[segIdx + 1] ?? MISSION[segIdx];

    const lat = wpA.lat + (wpB.lat - wpA.lat) * segT;
    const lon = wpA.lon + (wpB.lon - wpA.lon) * segT;
    const altRelTarget = wpA.altRel + (wpB.altRel - wpA.altRel) * segT;

    const altRelM = altRelTarget + smoothNoise(tSec, 0.3, 0.4);
    const altAmslM = altRelM + 2; // [PROVISIONAL] terrain offset ~2 m

    const yawDeg = seg.bearing;
    const rollDeg = smoothNoise(tSec, 1.1, 4) + smoothNoise(tSec, 2.3, 1.5);
    const pitchDeg = smoothNoise(tSec, 0.7, 2) + 1.5; // slight climb bias
    const verticalSpeedMs = smoothNoise(tSec, 0.5, 0.25);

    // Battery decays from 100% to 40% over the mission loop [PROVISIONAL]
    const battRemainingPct = Math.max(40, 100 - missionProgress * 60);
    const battVoltage = VOLT_EMPTY + (VOLT_FULL - VOLT_EMPTY) * ((battRemainingPct - 40) / 60);
    const battCurrent = 20 + smoothNoise(tSec, 2.0, 3); // [PROVISIONAL]

    // Mission tracking
    const wpCurrent = Math.min(segIdx + 1, MISSION.length - 1);
    const wpNext = MISSION[wpCurrent];
    const distToWpM = haversineM(lat, lon, wpNext.lat, wpNext.lon);

    return {
      tMs: t,
      rollDeg,
      pitchDeg,
      yawDeg,
      lat,
      lon,
      altRelM,
      altAmslM,
      groundSpeedMs: CRUISE_SPEED_MS + smoothNoise(tSec, 1.5, 0.8),
      verticalSpeedMs,
      battVoltage,
      battCurrent,
      battRemainingPct,
      gpsFix: "3D",
      gpsSats: 14,
      mode: "AUTO",
      armed: true,
      wpCurrent,
      wpTotal: MISSION.length,
      distToWpM,
    };
  }
}
