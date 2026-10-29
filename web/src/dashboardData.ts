import type { CanSnapshot } from "./canStatus";

export function axisPresence(snapshot: CanSnapshot | null, connected: boolean, receivedAt: number, now: number) {
    const fresh = connected && !!snapshot?.fresh && snapshot.age_seconds !== null &&
        snapshot.age_seconds + Math.max(0, now - receivedAt) / 1000 < 3;
    return Array.from({ length: 6 }, (_, i) => fresh ? snapshot!.devices.some(device =>
        device.id === i + 1 && device.values.online === true && device.state !== "stale") : null);
}
export function busStateLabel(state: unknown) {
    return state === "ERROR-ACTIVE" ? "OK (ERROR-ACTIVE)" : String(state ?? "—");
}
