import { useState, useEffect } from "react";
import type { Role } from "../telemetry/types";

const OPERATOR_TOKEN = import.meta.env.VITE_OPERATOR_TOKEN as string | undefined;

export function useRole(): Role {
  const [role, setRole] = useState<Role>("invitado");

  useEffect(() => {
    // Read token from URL query param ?k=<token>
    const params = new URLSearchParams(window.location.search);
    const urlToken = params.get("k");

    if (urlToken) {
      // Remove token from URL immediately — must not remain in browser history
      params.delete("k");
      const newSearch = params.toString();
      const newUrl = window.location.pathname + (newSearch ? `?${newSearch}` : "");
      history.replaceState(null, "", newUrl);

      // Persist to session storage so page refreshes without the param stay operator
      sessionStorage.setItem("vtol_role_token", urlToken);
    }

    const stored = sessionStorage.getItem("vtol_role_token") ?? urlToken ?? "";
    if (OPERATOR_TOKEN && stored === OPERATOR_TOKEN) {
      setRole("operador");
    }
  }, []);

  return role;
}
