// Przebieg programu (wykonawca w moście, robot_web_bridge/program_runner.py): kursory robota i programu, oś czasu.
import type { Block, Program } from "./programModel";

export type RunWarning = { block_id: string | null; code: string; text: string };
export type TimelineEntry = { block_id: string; chain: number; start_s: number; end_s: number; transition_end_s: number;
    stop: boolean; blend_mm_requested: number; blend_mm_applied: number };
export type ChainTiming = { segment: number; tag: string; start_time: number; end_time: number; transition_end: number; stop: boolean;
    requested_blend_radius: number; applied_blend_radius: number };
export type RobotCursor = { block_id: string; phase: "moving" | "blending" | "stopped" | "arming" | "settling" | "done";
    blend: { from: string; to: string; progress: number } | null };
export type RunStatus = {
    run_id: string | null; mode: "check" | "execute" | null; active: boolean;
    state: string; program_id: string | null; revision: string | null; name: string | null; message: string;
    started_at: string | null; finished_at: string | null;
    program_cursor: { block_id: string | null; state: "computing" | "waiting_robot" | "waiting_sync" | "done" };
    robot_cursor: RobotCursor | null;
    chain: { index: number; duration_s: number; elapsed_s: number; executor_state: string; segments: ChainTiming[] } | null;
    next_chain: { index?: number; state?: string; progress?: number; phase?: string } | null;
    chains: number; warnings: RunWarning[]; error: { block_id: string | null; text: string } | null;
    timeline: TimelineEntry[]; total_s: number | null;
};

export const runStateLabels: Record<string, string> = {
    idle: "Bezczynny", checking: "Sprawdzanie", starting: "Start", planning: "Liczenie ciągu", preparing: "Przygotowanie ciągu",
    arming: "Uzbrajanie napędów", running: "W ruchu", settling: "Dolot końcowy", stopping: "Zatrzymywanie",
    stopped: "Zatrzymany", succeeded: "Zakończony", failed: "Przerwany",
};
export const phaseLabels: Record<string, string> = { moving: "ruch", blending: "przejście", stopped: "postój", arming: "uzbrajanie", settling: "dolot", done: "koniec" };

// Dotyczy przebiegu tego programu (ten sam id; przy sprawdzeniu dokumentu bez id — dowolnego).
export function runFor(status: RunStatus | null, program: Program | null): RunStatus | null {
    if (!status || !program || status.state === "idle") return null;
    return status.program_id === (program.id ?? null) ? status : null;
}

// Klasy CSS kursorów dla bloku.
export function cursorClasses(status: RunStatus | null, blockId: string, ancestors: Set<string>): string {
    if (!status) return "";
    const out: string[] = [];
    const robot = status.active ? status.robot_cursor : null;
    if (robot) {
        if (robot.blend && (robot.blend.from === blockId || robot.blend.to === blockId)) out.push(robot.blend.from === blockId ? "run-blend-from" : "run-blend-to");
        else if (robot.block_id === blockId) out.push("run-robot");
    }
    if (status.active && status.program_cursor.block_id === blockId) out.push("run-program");
    if (status.active && ancestors.has(blockId)) out.push("run-ancestor");
    if (status.error?.block_id === blockId && !status.active) out.push("run-error");
    return out.join(" ");
}

// Bloki nadrzędne (pętle/warunki) bloku — do zaznaczenia ścieżki kursora.
export function ancestorIds(program: Program, id: string | null | undefined): string[] {
    if (!id) return [];
    const walk = (blocks: Block[], path: string[]): string[] | null => {
        for (const block of blocks) {
            if (block.id === id) return path;
            for (const [key, value] of Object.entries(block)) {
                if (key !== "params" && Array.isArray(value)) { const found = walk(value as Block[], [...path, block.id]); if (found) return found; }
            }
        }
        return null;
    };
    return walk(program.blocks, []) ?? [];
}

// Znacznik promienia pominięcia punktu na bloku ruchu (po sprawdzeniu: zastosowany promień).
export function blendBadge(block: Block, entry: TimelineEntry | undefined): { text: string; tone: "" | "warn"; title: string } | null {
    if (block.command !== "move_ptp" && block.command !== "move_lin") return null;
    const raw = block.params.blend_mm;
    if (typeof raw !== "number") return typeof raw === "object" && raw && "var" in raw ? { text: `⌀ ${raw.var}`, tone: "", title: "Promień ze zmiennej" } : null;
    if (!entry) return raw > 0 ? { text: `⌀ ${raw} mm`, tone: "", title: "Płynne przejście do następnego ruchu w tym promieniu" } : { text: "■ stop", tone: "", title: "Dojazd do punktu i postój" };
    if (raw > 0 && entry.blend_mm_applied <= 0) return { text: "⌀ ✕ stop", tone: "warn", title: "Przejście niemożliwe — postój w punkcie (szczegóły w uwagach przebiegu)" };
    if (raw > 0 && Math.abs(entry.blend_mm_applied - raw) > 0.05) return { text: `⌀ ${raw}→${entry.blend_mm_applied.toFixed(1)} mm`, tone: "warn", title: "Promień ograniczony przez planer" };
    return raw > 0 ? { text: `⌀ ${raw} mm`, tone: "", title: "Płynne przejście (zastosowane)" } : { text: "■ stop", tone: "", title: "Dojazd do punktu i postój" };
}

// Odcinki osi czasu do paska: ruch, przejście, przerwa między ciągami.
export type TimelineBar = { kind: "move" | "blend" | "stop" | "gap"; start: number; end: number; block?: string };
export function timelineBars(timeline: TimelineEntry[]): TimelineBar[] {
    const bars: TimelineBar[] = [];
    let previous: TimelineEntry | null = null;
    for (const entry of timeline) {
        if (previous && previous.chain !== entry.chain && entry.start_s > previous.transition_end_s) bars.push({ kind: "gap", start: previous.transition_end_s, end: entry.start_s });
        bars.push({ kind: "move", start: entry.start_s, end: entry.end_s, block: entry.block_id });
        if (entry.transition_end_s > entry.end_s) bars.push({ kind: entry.stop ? "stop" : "blend", start: entry.end_s, end: entry.transition_end_s, block: entry.block_id });
        previous = entry;
    }
    return bars;
}
