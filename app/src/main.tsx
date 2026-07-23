import "@fontsource/jetbrains-mono";
import "@fontsource/inter";
import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { TelemetryProvider } from "./telemetry/useTelemetry";
import { App } from "./App";

// Global dark-mode base styles
const style = document.createElement("style");
style.textContent = `
  *, *::before, *::after { box-sizing: border-box; }
  html, body, #root { margin: 0; padding: 0; height: 100%; overflow: hidden; }
  body { background: #0B1622; color: #E2EAF2; }
  ::-webkit-scrollbar { width: 4px; }
  ::-webkit-scrollbar-track { background: #0B1622; }
  ::-webkit-scrollbar-thumb { background: #163049; border-radius: 2px; }
  .leaflet-container { background: #0B1622 !important; }
`;
document.head.appendChild(style);

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <TelemetryProvider>
      <App />
    </TelemetryProvider>
  </StrictMode>
);
