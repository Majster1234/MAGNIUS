// Komunikacja z magazynem programów (robot_web_bridge/programs.py).
import { useEffect, useState } from "react";
import type { CommandLibrary, Position, Program, Variable } from "./programModel";
import type { RunStatus } from "./programRun";

export class ProgramApiError extends Error { constructor(message: string, readonly status: number) { super(message); } }
export type ProgramSummary = { id: string; name: string; approved?: boolean; description?: string; updated_at?: string; revision?: string; blocks: number; variables: number; positions: number; broken?: boolean };
export type ProgramReply = { program: Program; problems: string[]; unchanged?: boolean };
export type LibrarySnapshot = { directory: string; commands: CommandLibrary; warning: string | null;
    variables: { items: Variable[]; revision: string }; positions: { items: Position[]; revision: string } };

async function call<T>(path: string, method = "GET", data?: unknown): Promise<T> {
    const response = await fetch(`/api/v1/${path}`, { method, headers: data === undefined ? undefined : { "Content-Type": "application/json" },
        body: data === undefined ? undefined : JSON.stringify(data), signal: AbortSignal.timeout(6000) });
    const answer = await response.json().catch(() => ({}));
    if (!response.ok) throw new ProgramApiError(typeof answer.detail === "string" ? answer.detail : `Błąd ${response.status}`, response.status);
    return answer as T;
}
export const programApi = {
    library: () => call<LibrarySnapshot>("program-library"),
    list: () => call<{ items: ProgramSummary[]; directory: string }>("programs"),
    get: (id: string) => call<ProgramReply>(`programs/${encodeURIComponent(id)}`),
    create: (program: Program) => call<ProgramReply>("programs", "POST", { program }),
    save: (id: string, program: Program, revision: string | undefined) => call<ProgramReply>(`programs/${encodeURIComponent(id)}`, "PUT", { program, expected_revision: revision }),
    approve: (id: string, revision: string) => call<ProgramReply>(`programs/${encodeURIComponent(id)}/approve`, "POST", { revision }),
    revoke: (id: string) => call<ProgramReply>(`programs/${encodeURIComponent(id)}/approve`, "DELETE"),
    remove: (id: string) => call<{ deleted: string }>(`programs/${encodeURIComponent(id)}`, "DELETE"),
    run: (id: string, mode: "check" | "execute", revision: string | undefined) => call<RunStatus>(`programs/${encodeURIComponent(id)}/run`, "POST", { mode, revision }),
    checkDocument: (program: Program) => call<RunStatus>("program-run/check-document", "POST", { program }),
    runStatus: () => call<RunStatus>("program-run"),
    stopRun: () => call<RunStatus>("program-run/stop", "POST", {}),
    setLibrary: <K extends "variables" | "positions">(name: K, items: K extends "variables" ? Variable[] : Position[], revision: string) =>
        call<{ items: unknown[]; revision: string }>(`program-library/${name}`, "PUT", { items, expected_revision: revision }),
};
// Bieżące kąty przegubów z /api/v1/manual/status (wszystkie 6 osi muszą mieć znaną pozycję przegubu).
export async function currentJoints(): Promise<number[]> {
    const state = await call<{ axes?: { axis: number; joint_position_known?: boolean; joint_position_degrees?: number }[] }>("manual/status");
    const axes = (state.axes ?? []).slice(0, 6);
    const missing = axes.filter(a => !a.joint_position_known || !Number.isFinite(a.joint_position_degrees)).map(a => `J${a.axis}`);
    if (axes.length !== 6 || missing.length) throw new ProgramApiError(`Brak pozycji przegubu: ${missing.join(", ") || "osie niedostępne"}`, 409);
    return axes.map(a => Math.round((a.joint_position_degrees as number) * 1000) / 1000);
}
export async function activeConfigurationValues(): Promise<Record<string, unknown> | null> {
    try { const data = await call<{ active?: { document?: { values?: Record<string, unknown> } } }>("configuration"); return data.active?.document?.values ?? null; }
    catch { return null; }
}
// Stan wykonawcy programów: WebSocket 10 Hz, przy braku połączenia odpytywanie co 1 s.
export function useProgramRun(): RunStatus | null {
    const [status, setStatus] = useState<RunStatus | null>(null);
    useEffect(() => {
        let stopped = false, socket: WebSocket | null = null, retry: ReturnType<typeof setTimeout> | undefined, poll: ReturnType<typeof setInterval> | undefined;
        const startPolling = () => { if (poll) return; poll = setInterval(() => { programApi.runStatus().then(s => !stopped && setStatus(s)).catch(() => undefined); }, 1000); };
        const connect = () => {
            if (stopped) return;
            try { socket = new WebSocket(`${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/api/v1/program-run/ws`); }
            catch { startPolling(); return; }
            socket.onmessage = event => { if (stopped) return; try { setStatus(JSON.parse(event.data) as RunStatus); if (poll) { clearInterval(poll); poll = undefined; } } catch { socket?.close(); } };
            socket.onerror = () => socket?.close();
            socket.onclose = () => { if (stopped) return; startPolling(); retry = setTimeout(connect, 3000); };
        };
        programApi.runStatus().then(s => !stopped && setStatus(s)).catch(() => undefined);
        connect();
        return () => { stopped = true; socket?.close(); if (retry) clearTimeout(retry); if (poll) clearInterval(poll); };
    }, []);
    return status;
}
