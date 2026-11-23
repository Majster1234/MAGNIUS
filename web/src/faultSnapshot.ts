import { useEffect, useState } from "react";

// Migawki awarii (backend: robot_web_bridge/fault_api.py). Migawka powstaje ok. 5–7 s po awarii (okno „po” + zapis),
// więc karta awarii najpierw pokazuje „nagranie w przygotowaniu”, a potem uzupełnia się sama.
export type FaultTrigger = { source?: string; event?: string; detail?: string };
export type FaultListItem = { id: string; time: string; utc_ns: number; duration_s: number; rows: number; axis: number | null; triggers: FaultTrigger[] };
export type FaultEvent = { t_ms: number; time: string; source: string; event: string; axis: number | null; command: string; data: string; detail: string; level: "error" | "warning" | ""; key: boolean };
export type FaultMini = { axis: number; unit: string; source: "shaft" | "can"; plan_kind: "reference" | "command" | "hold" | null; start_ms: number; end_ms: number;
    samples: [number, number][]; plan: [number, number][]; gaps: [number, number][] };
export type FaultSummary = { id: string; time: string; utc_ns: number; duration_s: number; window: { pre_s: number; post_s: number }; rows: number; counts: Record<string, number>;
    triggers: FaultTrigger[]; axis: number | null; axis_source: "trigger" | "event" | null; excerpt: FaultEvent[]; key_events: number;
    hint: { text: string; facts: string[] } | null; mini: FaultMini | null; mini_missing: string | null; has_shaft: boolean;
    geometry: Record<string, unknown> | null; configuration_revision?: string };
export type FaultTraceAxis = { axis: number; source: string; plan_kind: string | null; samples: [number, number][]; plan: [number, number][]; gaps: [number, number][] };
export type FaultTrace = { axis: number | null; unit: string; axes: FaultTraceAxis[] };
export type FaultEvents = { total: number; offset: number; rows: FaultEvent[] };

export const ROBOT_FAULT_SOURCES = new Set(["controller", "recovery", "tcp-live-fault"]);
const MATCH_BEFORE_MS = 20000, MATCH_AFTER_MS = 10000;
export const SNAPSHOT_PENDING_MS = 30000;

const normalize = (text: string) => text.toLowerCase().replace(/\s+/g, " ").replace(/,/g, ".").trim();
// Dopasowanie komunikatu do migawki: wspólny początek treści albo awaria robota blisko w czasie (najbliższa).
export function matchSnapshot(message: { source: string; message: string; severity: string }, seenAt: number, snapshots: FaultListItem[]): FaultListItem | null {
    if (message.severity !== "error") return null;
    const text = normalize(message.message).slice(0, 48);
    const near = snapshots.filter(s => { const at = s.utc_ns / 1e6; return at >= seenAt - MATCH_BEFORE_MS && at <= seenAt + MATCH_AFTER_MS; });
    // Treść wystarcza także dla awarii sprzed załadowania panelu (komunikat „widziany” później niż powstała migawka).
    const byText = snapshots.filter(s => s.utc_ns / 1e6 <= seenAt + MATCH_AFTER_MS && text.length >= 12 &&
        s.triggers.some(t => { const detail = normalize(t.detail ?? ""); return detail.includes(text) || (detail.length >= 12 && text.includes(detail.slice(0, 48))); }));
    const pool = byText.length ? byText : ROBOT_FAULT_SOURCES.has(message.source) ? near : [];
    return pool.sort((a, b) => Math.abs(a.utc_ns / 1e6 - seenAt) - Math.abs(b.utc_ns / 1e6 - seenAt))[0] ?? null;
}

async function getJson<T>(url: string): Promise<T> {
    const response = await fetch(url, { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return await response.json() as T;
}
const summaries = new Map<string, Promise<FaultSummary>>();
export function fetchFaultSummary(id: string): Promise<FaultSummary> {
    let pending = summaries.get(id);
    if (!pending) { pending = getJson<FaultSummary>(`/api/v1/faults/${id}`); summaries.set(id, pending); pending.catch(() => summaries.delete(id)); }
    return pending;
}
export const fetchFaultTrace = (id: string) => getJson<FaultTrace>(`/api/v1/faults/${id}/trace`);
export function fetchFaultEvents(id: string, kind: string, axis: number | null, query: string, offset: number, limit = 200) {
    const params = new URLSearchParams({ kind, offset: String(offset), limit: String(limit) });
    if (axis) params.set("axis", String(axis));
    if (query.trim()) params.set("q", query.trim());
    return getJson<FaultEvents>(`/api/v1/faults/${id}/events?${params}`);
}
export const faultDownloadUrl = (id: string) => `/api/v1/faults/${id}/download`;

// Lista migawek: przy otwarciu panelu i co 2 s, dopóki któryś błąd czeka na swoją migawkę (najdłużej 30 s od awarii).
export function useFaultSnapshots(active: boolean, waiting: boolean): { snapshots: FaultListItem[]; available: boolean } {
    const [state, setState] = useState<{ snapshots: FaultListItem[]; available: boolean }>({ snapshots: [], available: false });
    useEffect(() => {
        if (!active) return;
        let stopped = false, timer: ReturnType<typeof setTimeout> | undefined;
        const load = async () => {
            try { const data = await getJson<{ available: boolean; snapshots: FaultListItem[] }>("/api/v1/faults?limit=100"); if (!stopped) setState(data); }
            catch { /* Brak migawek nie blokuje paska. */ }
            if (!stopped && waiting) timer = setTimeout(() => void load(), 2000);
        };
        void load();
        return () => { stopped = true; clearTimeout(timer); };
    }, [active, waiting]);
    return state;
}
