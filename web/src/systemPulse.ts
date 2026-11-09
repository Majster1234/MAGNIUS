import type { CanDevice, CanSnapshot } from "./canStatus";
import { numeric, type Component, type Snapshot, type State } from "./status";

export type PulseState = "ok" | "warn" | "stale" | "error";
export type SectionId = "SYS" | "ROS" | "API" | "WEB";
export type PulseSection = { id: SectionId; state: PulseState; title: string; lines: string[] };
export type DriveCell = { axis: number; state: PulseState; text: string };
export type PulseInput = {
    status: Snapshot | null; statusOnline: boolean; statusFresh: boolean; statusConnecting: boolean;
    can: CanSnapshot | null; canOnline: boolean; canFresh: boolean; build: string;
};

const rank: Record<PulseState, number> = { ok: 0, warn: 1, stale: 2, error: 3 };
export const pulseLabels: Record<PulseState, string> = { ok: "OK", warn: "Ostrzeżenie", stale: "Brak danych", error: "Awaria" };
export const sectionTitles: Record<SectionId, string> = { SYS: "System", ROS: "ROS 2", API: "Backend", WEB: "Frontend" };

export function worst(states: PulseState[]): PulseState {
    return states.reduce<PulseState>((a, b) => rank[b] > rank[a] ? b : a, "ok");
}
// „Brak integracji” nie jest awarią: komponent świadomie nieuruchomiony.
export function pulseState(state: State): PulseState {
    return state === "not_configured" ? "ok" : state;
}
export const seconds = (value: number) => value.toLocaleString("pl-PL", { minimumFractionDigits: 1, maximumFractionDigits: 1 });
const percent = (value: unknown) => { const n = numeric(value); return n === null ? "—" : `${Math.round(n)}%`; };
const problem = (label: string, component: Component | undefined) =>
    component && pulseState(component.state) !== "ok" ? [`${label}: ${component.message}`] : [];
export function freshCan(can: CanSnapshot | null, online: boolean, receivedAt: number, now: number) {
    return online && !!can?.fresh && can.age_seconds !== null && can.age_seconds + Math.max(0, now - receivedAt) / 1000 < 3;
}

function section(id: SectionId, state: PulseState, lines: string[]): PulseSection {
    return { id, state, title: sectionTitles[id], lines };
}

export function pulseSections(input: PulseInput): PulseSection[] {
    const { status, statusFresh, can, canFresh } = input;
    const c = status?.components;
    const sys = !statusFresh || !c ? section("SYS", "stale", ["Brak aktualnych danych monitora systemu"]) : (() => {
        const state = worst([pulseState(c.host.state), pulseState(c.can.state)]);
        const lines = [...problem("Komputer", c.host), ...problem(`Interfejs ${String(c.can.values.interface ?? "can0")}`, c.can)];
        return section("SYS", state, lines.length ? lines : [
            `CPU ${percent(c.host.values.cpu_percent)} · RAM ${percent(c.host.values.memory_percent)}`,
            `${String(c.can.values.interface ?? "can0")}: ${String(c.can.values.bus_state ?? "—")}`]);
    })();
    const ros = !statusFresh || !c ? section("ROS", "stale", ["Brak wiadomości z węzła diagnostycznego"]) : (() => {
        const diag: PulseState = !canFresh ? "warn" : pulseState(can!.summary.state);
        const lines = [...problem("Monitor systemu", c.ros), ...problem("Kontrolery", c.controllers)];
        if (!canFresh) lines.push("Diagnostyka CAN: brak świeżych danych");
        else if (diag !== "ok") lines.push(`Diagnostyka CAN: ${can!.summary.message}`);
        const recovery = canFresh ? String(can!.summary.values.recovery_message ?? "") : "";
        if (canFresh && can!.summary.values.fault === true && recovery) lines.push(`Odzyskiwanie: ${recovery}`);
        const state = worst([pulseState(c.ros.state), pulseState(c.controllers.state), diag]);
        return section("ROS", state, lines.length ? lines : ["Węzły diagnostyczne odpowiadają", `Diagnostyka CAN: ${can!.summary.message}`]);
    })();
    const api = input.statusOnline ? section("API", "ok", [
        `Backend odpowiada · ${status?.network.url ?? ""}`.trim(),
        `Wiek snapshotu: ${status?.age_seconds === null || status?.age_seconds === undefined ? "—" : `${seconds(status.age_seconds)} s`}`])
        : section("API", input.statusConnecting ? "stale" : "error", [input.statusConnecting ? "Łączenie z backendem" : "Backend niedostępny"]);
    const streams = [input.statusOnline, input.canOnline];
    const web = section("WEB", streams.every(Boolean) ? "ok" : streams.some(Boolean) ? "warn" : "stale", [
        `Strumień statusu: ${input.statusOnline ? "połączony" : "rozłączony"}`,
        `Strumień CAN: ${input.canOnline ? "połączony" : "rozłączony"}`,
        `Wersja panelu: ${input.build}`]);
    return [sys, ros, api, web];
}

function driveText(axis: number, device: CanDevice | undefined, fresh: boolean): [PulseState, string] {
    const head = `J${axis} · CAN ID ${axis}`;
    if (!fresh) return ["stale", `${head}: brak aktualnych danych CAN`];
    if (!device) return ["stale", `${head}: nie wykryto`];
    if (device.values.online !== true) return ["error", `${head}: ${device.message}`];
    const enable = device.values.enable === true ? "1" : device.values.enable === false ? "0" : "—";
    const rpm = numeric(device.values.reported_speed_rpm);
    const text = `${head}: ENABLE ${enable} · ${rpm === null ? "—" : rpm} RPM`;
    return device.values.stall === true ? ["warn", `${text} · zablokowanie`] : [pulseState(device.state), text];
}
export function driveCells(can: CanSnapshot | null, fresh: boolean): DriveCell[] {
    return Array.from({ length: 6 }, (_, i) => {
        const [state, text] = driveText(i + 1, can?.devices.find(device => device.id === i + 1), fresh);
        return { axis: i + 1, state, text };
    });
}

// Kanał pikselowy: 32 kolumny; każde odebranie danych przesuwa go o jeden piksel.
export const PULSE_COLUMNS = 32;
export function nextColumn(columns: boolean[], seed: number): boolean[] {
    const last = columns[columns.length - 1] ?? false;
    let run = 0;
    for (let i = columns.length - 1; i >= 0 && columns[i] === last; i--) run++;
    // Deterministyczna długość odcinka 1–4 z numeru taktu (bez Math.random w renderze).
    const length = 1 + ((seed * 2654435761) >>> 0) % 4;
    return [...columns, run >= length ? !last : last].slice(-PULSE_COLUMNS);
}
