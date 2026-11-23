import { describe, expect, it } from "vitest";
import { emptyProgram, problems, type CommandLibrary, type Program } from "./programModel";
import { ancestorIds, blendBadge, cursorClasses, runFor, timelineBars, type RunStatus, type TimelineEntry } from "./programRun";

const library: CommandLibrary = { schema_version: 1, categories: [{ id: "ruch", label: "Ruch" }], io: { outputs: [], inputs: [] }, commands: [
    { id: "move_ptp", category: "ruch", label: "PTP do {position} v {speed} ⌀ {blend_mm}", params: [{ name: "position", kind: "position", label: "pozycja" },
        { name: "speed", kind: "number", label: "prędkość", unit: "%", default: 30, min: 1, max: 100 },
        { name: "blend_mm", kind: "number", label: "promień pominięcia", unit: "mm", default: 0, min: 0, max: 200 }] },
    { id: "for", category: "logika", label: "powtórz {counter}", slots: [{ name: "body" }], params: [{ name: "counter", kind: "variable", label: "licznik" }] }] };

function program(): Program {
    const p = emptyProgram("Ciąg"); p.id = "ciag"; p.revision = "r1";
    p.variables = [{ name: "i", type: "INT", init: 0 }];
    p.positions = [{ name: "A", joints_deg: [0, 0, 0, 0, 45, 0] }];
    p.blocks = [{ id: "a", command: "move_ptp", params: { position: { position: "A" }, speed: 30, blend_mm: 10 } },
        { id: "f", command: "for", params: { counter: { var: "i" } }, body: [{ id: "b", command: "move_ptp", params: { position: { position: "A" }, speed: 30, blend_mm: 0 } }] }];
    return p;
}
function status(extra: Partial<RunStatus> = {}): RunStatus {
    return { run_id: "x", mode: "execute", active: true, state: "running", program_id: "ciag", revision: "r1", name: "Ciąg", message: "", started_at: null, finished_at: null,
        program_cursor: { block_id: "b", state: "waiting_robot" }, robot_cursor: { block_id: "a", phase: "moving", blend: null }, chain: null, next_chain: null,
        chains: 1, warnings: [], error: null, timeline: [], total_s: null, ...extra };
}
const entry = (block: string, applied: number, extra: Partial<TimelineEntry> = {}): TimelineEntry =>
    ({ block_id: block, chain: 0, start_s: 0, end_s: 1, transition_end_s: 1.2, stop: applied <= 0, blend_mm_requested: 10, blend_mm_applied: applied, ...extra });

describe("kursory przebiegu programu", () => {
    it("kursor robota, kursor programu, przejście i ścieżka pętli", () => {
        const p = program(); const ancestors = new Set(ancestorIds(p, "b"));
        expect([...ancestors]).toEqual(["f"]);
        expect(cursorClasses(status(), "a", ancestors)).toBe("run-robot");
        expect(cursorClasses(status(), "b", ancestors)).toBe("run-program");
        expect(cursorClasses(status(), "f", ancestors)).toBe("run-ancestor");
        const blending = status({ robot_cursor: { block_id: "a", phase: "blending", blend: { from: "a", to: "b", progress: .3 } } });
        expect(cursorClasses(blending, "a", new Set())).toBe("run-blend-from");
        expect(cursorClasses(blending, "b", new Set())).toContain("run-blend-to");
        expect(cursorClasses(status({ active: false, state: "failed", error: { block_id: "b", text: "x" } }), "b", new Set())).toBe("run-error");
        expect(runFor(status(), p)).not.toBeNull(); expect(runFor(status({ program_id: "inny" }), p)).toBeNull();
    });
    it("znacznik promienia: żądany, ograniczony, niemożliwy, postój", () => {
        const [a] = program().blocks; const b = program().blocks[1].body![0];
        expect(blendBadge(a, undefined)?.text).toBe("⌀ 10 mm");
        expect(blendBadge(a, entry("a", 10))?.tone).toBe("");
        expect(blendBadge(a, entry("a", 4))).toMatchObject({ text: "⌀ 10→4.0 mm", tone: "warn" });
        expect(blendBadge(a, entry("a", 0))).toMatchObject({ tone: "warn" });
        expect(blendBadge(b, undefined)?.text).toBe("■ stop");
        expect(blendBadge(program().blocks[1], undefined)).toBeNull();
    });
    it("oś czasu: ruch, przejście, postój i przerwa między ciągami", () => {
        const bars = timelineBars([entry("a", 10, { stop: false }), entry("b", 0, { start_s: 1.2, end_s: 2, transition_end_s: 2.01 }),
            entry("c", 0, { chain: 1, start_s: 4.01, end_s: 5, transition_end_s: 5 })]);
        expect(bars.map(b => b.kind)).toEqual(["move", "blend", "move", "stop", "gap", "move"]);
    });
    it("uwagi: wartości spoza zakresu parametrów", () => {
        const p = program(); p.blocks[0].params.speed = 150; p.blocks[0].params.blend_mm = -2;
        const texts = problems(p, library).map(i => i.text);
        expect(texts.some(t => t.includes("150") && t.includes("maksimum"))).toBe(true);
        p.blocks[0].params.speed = 50; p.blocks[0].params.blend_mm = 5;
        expect(problems(p, library)).toEqual([]);
    });
});
