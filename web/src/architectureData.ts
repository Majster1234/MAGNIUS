import type { CanSnapshot } from "./canStatus";
import { numeric, type Snapshot } from "./status";
import { pulseState, seconds, type PulseState } from "./systemPulse";

export type OverviewProcess = {
    pid: number; name: string | null; package: string | null; overlay: string | null; cpu_percent: number | null;
    memory_rss_bytes: number; threads: number; uptime_seconds: number;
};
export type Overview = {
    schema_version: 1; age_seconds: number;
    host: { os: string | null; kernel: string; hostname: string };
    ros: { distro: string | null; overlay: string | null; nodes: string[] | null; processes: OverviewProcess[] };
    backend: Partial<OverviewProcess> & { websocket_clients: Record<string, number> };
    services: { unit: string; active_state: string; sub_state: string; result: string; main_pid: number | null }[] | null;
};
export function isOverview(value: unknown): value is Overview {
    const v = value as Overview;
    return typeof v === "object" && v !== null && v.schema_version === 1 && typeof v.ros === "object" && v.ros !== null &&
        Array.isArray(v.ros.processes) && typeof v.backend === "object" && v.backend !== null && typeof v.host === "object";
}

export type Group = "SYSTEM" | "ROS" | "BACKEND" | "FRONTEND";
export type RowState = PulseState | "off";
export type Row = {
    id: string; group: Group; state: RowState; name: string; type: string; pid?: number | null;
    cpu?: number | null; ram?: number | null; uptime?: number | null; message: string; details: [string, string][];
};
export type ArchitectureInput = {
    status: Snapshot | null; statusFresh: boolean; statusOnline: boolean;
    can: CanSnapshot | null; canFresh: boolean; canOnline: boolean;
    overview: Overview | null; overviewError: string | null; build: { revision: string; built_at: string };
};

export const groups: { id: Group; label: string }[] = [
    { id: "SYSTEM", label: "System" }, { id: "ROS", label: "ROS 2" }, { id: "BACKEND", label: "Backend Python" }, { id: "FRONTEND", label: "Frontend" }];
export const formatBytes = (value: number | null | undefined) =>
    value === null || value === undefined ? "—" : value >= 2 ** 30 ? `${(value / 2 ** 30).toFixed(1)}G` : `${Math.round(value / 2 ** 20)}M`;
export function formatDuration(seconds: number | null | undefined) {
    if (seconds === null || seconds === undefined || !Number.isFinite(seconds)) return "—";
    const s = Math.floor(seconds), d = Math.floor(s / 86400), h = Math.floor(s % 86400 / 3600), m = Math.floor(s % 3600 / 60);
    return d ? `${d} d ${h} h` : h ? `${h} h ${m} min` : m ? `${m} min` : `${s} s`;
}
const text = (value: unknown, fallback = "—") => value === null || value === undefined || value === "" ? fallback : String(value);
const num = (value: unknown, digits = 1) => { const n = numeric(value); return n === null ? "—" : n.toLocaleString("pl-PL", { maximumFractionDigits: digits }); };

function serviceState(active: string): RowState {
    return active === "active" ? "ok" : active === "activating" || active === "reloading" ? "warn" : active === "inactive" ? "off" : "error";
}

export function architectureRows(input: ArchitectureInput): Row[] {
    const { status, statusFresh, can, canFresh, overview } = input;
    const c = status?.components;
    const stale = (state: Parameters<typeof pulseState>[0] | undefined): RowState => !statusFresh || !state ? "stale" : pulseState(state);
    const missing = overview ? null : input.overviewError ?? "Oczekiwanie na przegląd systemu";
    const rows: Row[] = [];
    // SYSTEM
    rows.push({ id: "host", group: "SYSTEM", state: stale(c?.host.state), name: overview?.host.hostname ?? "Komputer", type: "host",
        cpu: numeric(c?.host.values.cpu_percent), ram: numeric(c?.host.values.memory_used_bytes), uptime: numeric(c?.host.values.uptime_seconds),
        message: overview ? `${text(overview.host.os)} · ${overview.host.kernel}` : text(c?.host.message),
        details: [["System", text(overview?.host.os)], ["Jądro", text(overview?.host.kernel)],
            ["RAM", `${formatBytes(numeric(c?.host.values.memory_used_bytes))} / ${formatBytes(numeric(c?.host.values.memory_total_bytes))}`]] });
    if (overview?.services) for (const unit of overview.services) rows.push({ id: `unit:${unit.unit}`, group: "SYSTEM",
        state: serviceState(unit.active_state), name: unit.unit, type: "systemd", pid: unit.main_pid,
        message: `${unit.active_state} (${unit.sub_state})`, details: [["Stan", unit.active_state], ["Podstan", unit.sub_state], ["Wynik", unit.result]] });
    else rows.push({ id: "units", group: "SYSTEM", state: "stale", name: "Usługi systemd", type: "systemd", message: missing ?? "Brak odczytu systemctl", details: [] });
    const link = c?.can.values ?? {};
    const bitrate = numeric(link.bitrate);
    rows.push({ id: "can-link", group: "SYSTEM", state: stale(c?.can.state), name: `${text(link.interface, "can0")} / gs_usb`, type: "SocketCAN",
        message: `${text(link.bus_state)} · ${bitrate === null ? "—" : `${bitrate / 1e6} Mbit/s`} · błędy RX/TX ${text(link.rx_errors)}/${text(link.tx_errors)} · bus-off ${text(link.bus_off_count)}`,
        details: [["Komunikat", text(c?.can.message)], ["Ramki RX/TX", `${text(link.rx_packets)} / ${text(link.tx_packets)}`], ["Błędy magistrali", text(link.bus_errors)]] });
    // ROS 2
    if (overview) for (const proc of overview.ros.processes) rows.push({ id: `proc:${proc.pid}`, group: "ROS", state: "ok",
        name: proc.name ?? `PID ${proc.pid}`, type: proc.package ?? "proces ROS", pid: proc.pid, cpu: proc.cpu_percent, ram: proc.memory_rss_bytes,
        uptime: proc.uptime_seconds, message: text(proc.overlay), details: [["Pakiet", text(proc.package)], ["Nakładka", text(proc.overlay)], ["Wątki", String(proc.threads)]] });
    else rows.push({ id: "procs", group: "ROS", state: "stale", name: "Procesy ROS", type: "procesy", message: missing!, details: [] });
    const summary = can?.summary;
    const sv = summary?.values ?? {};
    rows.push({ id: "can-diag", group: "ROS", state: !canFresh || !summary ? "stale" : pulseState(summary.state), name: "Diagnostyka CAN", type: "mks_servo_can",
        message: canFresh && summary ? summary.message : "Brak świeżych danych diagnostyki CAN",
        details: [["Stan magistrali", text(sv.bus_state)], ["Napędy online", `${text(sv.online_count)} / ${text(sv.device_count)}`],
            ["RTT maks.", `${num(sv.rtt_max_ms)} ms / timeout ${text(sv.response_timeout_ms)} ms`], ["Planowane obciążenie", `${num(sv.scheduled_utilization_percent)}%`],
            ["Odzyskiwanie", text(sv.recovery_state)], ["Komunikat odzyskiwania", text(sv.recovery_message)], ["Maska oczekujących", text(sv.recovery_pending_mask)],
            ["Opóźnienie SYNC maks.", `${num(sv.sync_lateness_max_ms)} ms`], ["Timeouty / błędne odpowiedzi", `${text(sv.timeouts)} / ${text(sv.invalid_responses)}`]] });
    rows.push({ id: "monitor", group: "ROS", state: stale(c?.ros.state), name: "Monitor systemu", type: "temat", message: text(c?.ros.message),
        details: [["Temat", text(c?.ros.values.topic)]] });
    const controllers = Array.isArray(c?.controllers.values.controllers) ? c!.controllers.values.controllers as { name: string; state: string; type: string }[] : [];
    if (controllers.length) for (const item of controllers) rows.push({ id: `ctrl:${item.name}`, group: "ROS", state: !statusFresh ? "stale" : item.state === "active" ? "ok" : "off",
        name: item.name, type: "kontroler", message: item.state, details: [["Typ", item.type]] });
    else rows.push({ id: "ctrl", group: "ROS", state: !statusFresh ? "stale" : c?.controllers.state === "not_configured" ? "off" : stale(c?.controllers.state),
        name: "ros2_control", type: "kontrolery", message: text(c?.controllers.message), details: [] });
    if (overview?.ros.nodes) rows.push({ id: "graph", group: "ROS", state: "ok", name: "Graf ROS", type: "węzły", message: `${overview.ros.nodes.length} węzłów`,
        details: overview.ros.nodes.map(name => [name, ""]) });
    // BACKEND
    const backend = overview?.backend;
    rows.push({ id: "bridge", group: "BACKEND", state: input.statusOnline ? "ok" : "error", name: "robot_web_bridge", type: "FastAPI", pid: backend?.pid,
        cpu: backend?.cpu_percent, ram: backend?.memory_rss_bytes, uptime: backend?.uptime_seconds,
        message: input.statusOnline ? `uvicorn ${text(status?.network.url)}` : "Backend niedostępny",
        details: [["Adres", text(status?.network.url)], ["Interfejs LAN", text(status?.network.interface)], ["Podsieć", text(status?.network.subnet)],
            ["Wątki", text(backend?.threads)], ["Nakładka", text(backend?.overlay)]] });
    const ws = backend?.websocket_clients;
    rows.push({ id: "ws", group: "BACKEND", state: ws ? "ok" : "stale", name: "Klienci WebSocket", type: "WebSocket",
        message: ws ? `status ${ws.status ?? 0} · CAN ${ws.can ?? 0} · ręczne ${ws.manual ?? 0}` : missing!, details: [] });
    rows.push({ id: "snapshot", group: "BACKEND", state: !input.statusOnline ? "stale" : statusFresh ? "ok" : "warn", name: "Snapshot statusu", type: "REST/WS",
        message: status?.age_seconds === null || status?.age_seconds === undefined ? "Brak danych monitora" : `wiek ${seconds(status.age_seconds)} s`, details: [] });
    // FRONTEND
    rows.push({ id: "build", group: "FRONTEND", state: "ok", name: "Build panelu", type: "web/dist", message: input.build.revision,
        details: [["Wersja", input.build.revision], ["Zbudowano", input.build.built_at ? new Date(input.build.built_at).toLocaleString("pl-PL") : "—"]] });
    rows.push({ id: "tab", group: "FRONTEND", state: input.statusOnline && input.canOnline ? "ok" : input.statusOnline || input.canOnline ? "warn" : "stale",
        name: "Ta karta", type: "przeglądarka", message: `status ${input.statusOnline ? "połączony" : "rozłączony"} · CAN ${input.canOnline ? "połączony" : "rozłączony"}`, details: [] });
    return rows;
}

export const isProblem = (row: Row) => row.state === "warn" || row.state === "error" || row.state === "stale";

export type Meter = { label: string; value: number | null; max: number; limit?: number; text: string };
export function architectureMeters(input: ArchitectureInput): Meter[] {
    const host = input.statusFresh ? input.status?.components.host.values ?? {} : {};
    const sv = input.canFresh ? input.can?.summary.values ?? {} : {};
    const rtt = numeric(sv.rtt_max_ms), timeout = numeric(sv.response_timeout_ms), load = numeric(sv.scheduled_utilization_percent);
    return [
        { label: "CPU", value: numeric(host.cpu_percent), max: 100, text: `${num(host.cpu_percent)}%` },
        { label: "RAM", value: numeric(host.memory_percent), max: 100, text: `${formatBytes(numeric(host.memory_used_bytes))}/${formatBytes(numeric(host.memory_total_bytes))}` },
        { label: "CAN", value: load, max: 100, limit: numeric(sv.total_bus_planning_limit_percent) ?? undefined,
            text: `${num(load)}% / ${text(sv.total_bus_planning_limit_percent)}` },
        { label: "RTT", value: rtt, max: timeout ?? 1, text: `${num(rtt)}/${text(timeout)} ms` },
    ];
}
