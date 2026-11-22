import { useEffect, useRef } from "react";
import type { Notice } from "./eventLog";
import type { MessageCenter } from "./messageCenter";
import type { Snapshot } from "./status";

// Stan podzespołów (/api/v1/ws) jako komunikaty paska: problemy ROS, komputera, CAN i kontrolerów oraz brak backendu.
const COMPONENTS: [keyof Snapshot["components"], string][] = [["ros", "ROS 2"], ["host", "Komputer robota"], ["can", "Magistrala CAN"], ["controllers", "Kontrolery ruchu"]];
export const SYSTEM_KEYS = [...COMPONENTS.map(([id]) => `system-${id}`), "system-backend"];
export const BACKEND_OFFLINE_MS = 5000;

export function systemConditions(snapshot: Snapshot | null, fresh: boolean): [string, Notice | null][] {
    return COMPONENTS.map(([id, label]) => {
        const component = fresh ? snapshot?.components?.[id] : undefined;
        const problem = component && (component.state === "warn" || component.state === "error");
        return [`system-${id}`, problem ? { source: `system-${id}`, severity: component.state === "error" ? "error" : "warning", context: label,
            title: `${label}: ${component.state === "error" ? "błąd" : "uwaga"}`, message: component.message || "Brak opisu",
            description: `${label} zgłasza ${component.state === "error" ? "błąd" : "ostrzeżenie"}: ${component.message || "brak opisu"}.`,
            remedy: "Sprawdź szczegóły w Diagnostyce (mapa systemu i procesy).", href: id === "can" ? "#can" : "#diagnostics" } : null];
    });
}
export function backendNotice(offlineMs: number): Notice | null {
    return offlineMs >= BACKEND_OFFLINE_MS ? { source: "system-backend", severity: "error", context: "Panel", title: "Brak połączenia z backendem",
        message: "Panel nie otrzymuje stanu systemu (strumień /api/v1/ws).",
        description: "Strumień stanu systemu jest przerwany. Wyświetlane dane mogą być nieaktualne; ruch fizyczny mógł nie zostać zatrzymany.",
        remedy: "Sprawdź sieć LAN i usługę robot-web-status. Po powrocie łączności sprawdź stan robota przed kolejną operacją.", href: "#diagnostics" } : null;
}

export function useSystemMessages(center: MessageCenter, snapshot: Snapshot | null, online: boolean, fresh: boolean) {
    const offlineSince = useRef<number | null>(null);
    useEffect(() => {
        if (online) offlineSince.current = null; else offlineSince.current ??= Date.now();
        const update = () => {
            // Nieaktualny status: komunikaty podzespołów znikają bez wpisu „ustąpił” (nie są obserwowane).
            if (!fresh) center.drop(COMPONENTS.map(([id]) => `system-${id}`));
            else for (const [key, notice] of systemConditions(snapshot, fresh)) center.condition(key, notice);
            center.condition("system-backend", backendNotice(offlineSince.current === null ? 0 : Date.now() - offlineSince.current), "Połączenie z backendem przywrócone.");
        };
        update();
        if (online) return;
        const timer = setInterval(update, 1000);
        return () => clearInterval(timer);
    }, [center, snapshot, online, fresh]);
}
