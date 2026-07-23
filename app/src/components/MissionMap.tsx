import { useEffect, useRef } from "react";
import { MapContainer, TileLayer, Polyline, Marker, Popup, useMap } from "react-leaflet";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import { theme } from "../theme";
import type { TelemetryFrame, Waypoint } from "../telemetry/types";

// CARTO Dark Matter tiles — attribution required
const TILE_URL = "https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png";
const ATTRIBUTION = '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors &copy; <a href="https://carto.com/">CARTO</a>';

function makeAircraftIcon(yawDeg: number): L.DivIcon {
  const svg = `
    <svg xmlns="http://www.w3.org/2000/svg" width="32" height="32" viewBox="0 0 32 32">
      <g transform="rotate(${yawDeg}, 16, 16)">
        <!-- fuselage -->
        <polygon points="16,4 19,28 16,24 13,28" fill="${theme.accent}" opacity="0.95"/>
        <!-- wings -->
        <polygon points="16,14 28,20 16,17 4,20" fill="${theme.accent}" opacity="0.8"/>
      </g>
    </svg>`;
  return L.divIcon({
    html: svg,
    className: "",
    iconSize: [32, 32],
    iconAnchor: [16, 16],
  });
}

function makeWpIcon(active: boolean): L.DivIcon {
  const color = active ? theme.accent : theme.textDim;
  const size = active ? 10 : 7;
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${size * 2}" height="${size * 2}">
    <circle cx="${size}" cy="${size}" r="${size - 1}" fill="none" stroke="${color}" stroke-width="2"/>
    ${active ? `<circle cx="${size}" cy="${size}" r="${size / 2}" fill="${color}"/>` : ""}
  </svg>`;
  return L.divIcon({
    html: svg,
    className: "",
    iconSize: [size * 2, size * 2],
    iconAnchor: [size, size],
  });
}

// Follows the aircraft on the map
function AircraftFollower({ lat, lon }: { lat: number; lon: number }) {
  const map = useMap();
  const firstRef = useRef(true);

  useEffect(() => {
    if (firstRef.current) {
      map.setView([lat, lon], 14, { animate: false });
      firstRef.current = false;
    } else {
      map.panTo([lat, lon], { animate: true, duration: 0.5 });
    }
  }, [lat, lon, map]);

  return null;
}

interface Props {
  frame: TelemetryFrame | null;
  mission: Waypoint[];
}

export function MissionMap({ frame, mission }: Props) {
  const routePositions: [number, number][] = mission.map(wp => [wp.lat, wp.lon]);
  const centerLat = mission[0]?.lat ?? 10.3997;
  const centerLon = mission[0]?.lon ?? -75.5144;

  return (
    <div style={{ position: "relative", width: "100%", height: "100%", background: theme.bg }}>
      <MapContainer
        center={[centerLat, centerLon]}
        zoom={14}
        style={{ width: "100%", height: "100%", background: theme.bg }}
        zoomControl={true}
        attributionControl={true}
      >
        <TileLayer url={TILE_URL} attribution={ATTRIBUTION} />

        {/* Mission route */}
        <Polyline
          positions={routePositions}
          pathOptions={{ color: theme.accent, weight: 2, opacity: 0.6, dashArray: "6 4" }}
        />

        {/* Waypoints */}
        {mission.map(wp => (
          <Marker
            key={wp.seq}
            position={[wp.lat, wp.lon]}
            icon={makeWpIcon(frame ? frame.wpCurrent === wp.seq : false)}
          >
            <Popup>
              <span style={{ fontFamily: theme.fontMono, fontSize: "0.75rem" }}>
                WP {wp.seq} · {wp.altRel} m
              </span>
            </Popup>
          </Marker>
        ))}

        {/* Aircraft marker */}
        {frame && (
          <>
            <Marker
              position={[frame.lat, frame.lon]}
              icon={makeAircraftIcon(frame.yawDeg)}
            />
            <AircraftFollower lat={frame.lat} lon={frame.lon} />
          </>
        )}
      </MapContainer>

      {/* Mission label overlay */}
      {frame && (
        <div style={{
          position: "absolute",
          top: "0.5rem",
          left: "0.5rem",
          zIndex: 1000,
          background: "rgba(11,22,34,0.82)",
          border: `1px solid ${theme.border}`,
          borderRadius: "0.25rem",
          padding: "0.2rem 0.6rem",
          fontFamily: theme.fontMono,
          fontSize: "0.72rem",
          color: theme.textDim,
          pointerEvents: "none",
        }}>
          misión · wp {frame.wpCurrent}/{frame.wpTotal - 1}
        </div>
      )}
    </div>
  );
}
