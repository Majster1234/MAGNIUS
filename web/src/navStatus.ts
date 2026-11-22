import type { CanSnapshot } from "./canStatus";
import { numeric, type Snapshot } from "./status";

// Zużycie zasobów do panelu stanu w menu bocznym. Brak świeżych danych = null (panel pokazuje „—”).
export type NavResources = {
    cpu: number | null; cores: number[]; ram: number | null; ramUsed: number | null; ramTotal: number | null;
    canLoad: number | null; canLimit: number | null;
};

export function navResources(status: Snapshot | null, statusFresh: boolean, can: CanSnapshot | null, canFresh: boolean): NavResources {
    const host = statusFresh ? status?.components.host.values ?? {} : {};
    const summary = canFresh ? can?.summary.values ?? {} : {};
    const cores = Array.isArray(host.cpu_per_core_percent) ? host.cpu_per_core_percent.map(numeric).filter((value): value is number => value !== null) : [];
    return {
        cpu: numeric(host.cpu_percent), cores,
        ram: numeric(host.memory_percent), ramUsed: numeric(host.memory_used_bytes), ramTotal: numeric(host.memory_total_bytes),
        canLoad: numeric(summary.scheduled_utilization_percent), canLimit: numeric(summary.total_bus_planning_limit_percent),
    };
}

export const percentText = (value: number | null, digits = 0) =>
    value === null ? "—" : value.toLocaleString("pl-PL", { minimumFractionDigits: digits, maximumFractionDigits: digits });
export const gigabytes = (value: number | null) =>
    value === null ? "—" : (value / 2 ** 30).toLocaleString("pl-PL", { minimumFractionDigits: 1, maximumFractionDigits: 1 });
/** Szerokość/wysokość paska w procentach, ograniczona do 0–100. */
export const barPercent = (value: number | null) => value === null ? 0 : Math.max(0, Math.min(100, value));

// Gradient obciążenia: zielony (OK) → żółty → bursztynowy (ostrzeżenie) → czerwony (awaria) — barwy stanów z theme.css.
const LOAD_STOPS: [number, [number, number, number]][] = [[0, [33, 208, 122]], [.5, [230, 208, 60]], [.75, [255, 176, 32]], [1, [255, 59, 78]]];
/** Kolor dla udziału 0…1 (powyżej 1 = czerwony). */
export function loadColor(ratio: number | null): string | undefined {
    if (ratio === null || !Number.isFinite(ratio)) return undefined;
    const r = Math.max(0, Math.min(1, ratio));
    const upper = LOAD_STOPS.findIndex(([at]) => at >= r);
    const [a, ca] = LOAD_STOPS[Math.max(0, upper - 1)], [b, cb] = LOAD_STOPS[upper];
    const t = b === a ? 0 : (r - a) / (b - a);
    return `rgb(${ca.map((v, i) => Math.round(v + (cb[i] - v) * t)).join(", ")})`;
}
/** Tło paska: gradient rozpięty na skali 0…`full` % (np. limit pasma CAN), dalej czerwony. */
export function loadGradient(direction: "90deg" | "0deg", full = 100): string {
    const at = (share: number) => `${(share * full).toFixed(1)}%`;
    return `linear-gradient(${direction}, rgb(33, 208, 122) 0%, rgb(230, 208, 60) ${at(.5)}, rgb(255, 176, 32) ${at(.75)}, rgb(255, 59, 78) ${at(1)})`;
}
