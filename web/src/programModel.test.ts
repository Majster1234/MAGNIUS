import { describe, expect, it } from "vitest";
import { accepts, allIds, changeCommand, compatibleCommands, countBlocks, duplicateBlock, emptyProgram, findBlock, insertBlock, isDescendant, moveBlock, nameError, newBlock,
    parseInit, problems, programText, removeBlock, renameRefs, setParam, usage, type CommandLibrary, type Program } from "./programModel";

const library: CommandLibrary = { schema_version: 1, categories: [{ id: "ruch", label: "Ruch" }], io: { outputs: ["OUT1"], inputs: ["IN1"] }, commands: [
    { id: "move_ptp", category: "ruch", label: "jedź PTP do {position} z prędkością {speed}", text: "MOVE PTP {position} v={speed}",
      params: [{ name: "position", kind: "position", label: "pozycja" }, { name: "speed", kind: "number", label: "prędkość", unit: "%", default: 30 }] },
    { id: "for", category: "logika", label: "powtórz {counter} od {from} do {to}", text: "FOR {counter} := {from} TO {to}", end: "END_FOR", slots: [{ name: "body" }],
      params: [{ name: "counter", kind: "variable", label: "licznik", types: ["INT"] }, { name: "from", kind: "number", label: "od", default: 1 }, { name: "to", kind: "number", label: "do", default: 3 }] },
    { id: "if", category: "logika", label: "jeśli {condition} to", text: "IF {condition} THEN", end: "END_IF", slots: [{ name: "body" }, { name: "else", text: "ELSE" }],
      params: [{ name: "condition", kind: "condition", label: "warunek", default: "TRUE" }] },
] };
const cmd = (id: string) => library.commands.find(c => c.id === id)!;

function sample(): Program {
    const p = emptyProgram("Test");
    p.variables = [{ name: "i", type: "INT", init: 0 }, { name: "cel", type: "POZYCJA", init: "" }, { name: "ok", type: "BOOL", init: false }];
    p.positions = [{ name: "Home", joints_deg: [0, 0, 0, 0, 45, 0] }];
    p.blocks = [{ id: "a", command: "move_ptp", params: { position: { position: "Home" }, speed: 30 } },
        { id: "f", command: "for", params: { counter: { var: "i" }, from: 1, to: 3 }, body: [{ id: "b", command: "move_ptp", params: { position: { var: "cel" }, speed: 40 } }] }];
    return p;
}

describe("drzewo bloków", () => {
    it("wstawia nowy blok z domyślnymi parametrami do korzenia i do slotu", () => {
        let p = sample(); const block = newBlock(cmd("if"), allIds(p.blocks));
        expect(block.params).toEqual({ condition: "TRUE" }); expect(block.body).toEqual([]); expect(block.else).toEqual([]);
        p = insertBlock(p, { parent: "f", slot: "body", index: 0 }, block);
        expect(findBlock(p, block.id)?.parent?.id).toBe("f"); expect(countBlocks(p.blocks)).toBe(4);
        p = insertBlock(p, { parent: block.id, slot: "else", index: 0 }, newBlock(cmd("move_ptp"), allIds(p.blocks)));
        expect(findBlock(p, block.id)!.block.else).toHaveLength(1);
    });
    it("przenosi w obrębie listy z korektą indeksu i nie pozwala wstawić bloku do własnego wnętrza", () => {
        let p = sample();
        p = moveBlock(p, "a", { parent: null, slot: "blocks", index: 2 });
        expect(p.blocks.map(b => b.id)).toEqual(["f", "a"]);
        expect(isDescendant(p, "f", "b")).toBe(true);
        expect(moveBlock(p, "f", { parent: "f", slot: "body", index: 0 })).toBe(p);
        p = moveBlock(p, "b", { parent: null, slot: "blocks", index: 0 });
        expect(p.blocks.map(b => b.id)).toEqual(["b", "f", "a"]); expect(findBlock(p, "f")!.block.body).toEqual([]);
    });
    it("usuwa, duplikuje z nowymi identyfikatorami i ustawia parametr bez mutacji", () => {
        const p = sample();
        const { program: r, block } = removeBlock(p, "f"); expect(block?.id).toBe("f"); expect(countBlocks(r.blocks)).toBe(1); expect(countBlocks(p.blocks)).toBe(3);
        const d = duplicateBlock(p, "f"); expect(countBlocks(d.blocks)).toBe(5); expect(allIds(d.blocks).size).toBe(5);
        const s = setParam(p, "b", "speed", 70); expect(findBlock(s, "b")!.block.params.speed).toBe(70); expect(findBlock(p, "b")!.block.params.speed).toBe(40);
    });
});

describe("zmienne, pozycje i podgląd", () => {
    it("liczy użycia i zmienia nazwy odwołań", () => {
        const p = sample(); expect(usage(p, "var", "i")).toBe(1); expect(usage(p, "position", "Home")).toBe(1);
        const r = renameRefs(p, "position", "Home", "Baza"); expect(findBlock(r, "a")!.block.params.position).toEqual({ position: "Baza" });
    });
    it("waliduje nazwy i wartości początkowe", () => {
        expect(nameError("Odbiór_2", [])).toBeNull(); expect(nameError("2x", [])).toBeTruthy(); expect(nameError("i", ["i"])).toBeTruthy(); expect(nameError("i", ["i"], "i")).toBeNull();
        expect(parseInit("INT", "3")).toBe(3); expect(parseInit("INT", "3.5")).toBeNull(); expect(parseInit("REAL", "2,5")).toBe(2.5); expect(parseInit("BOOL", "true")).toBe(true);
    });
    it("dopasowuje przeciągane zmienne i pozycje do typów parametrów", () => {
        const p = sample(), [pos, speed] = cmd("move_ptp").params!, counter = cmd("for").params![0], cond = cmd("if").params![0];
        expect(accepts(pos, { kind: "position", name: "Home" }, p.variables)).toBe(true);
        expect(accepts(pos, { kind: "variable", name: "cel" }, p.variables)).toBe(true);
        expect(accepts(pos, { kind: "variable", name: "i" }, p.variables)).toBe(false);
        expect(accepts(speed, { kind: "variable", name: "i" }, p.variables)).toBe(true);
        expect(accepts(counter, { kind: "variable", name: "ok" }, p.variables)).toBe(false);
        expect(accepts(cond, { kind: "variable", name: "ok" }, p.variables)).toBe(true);
    });
    it("generuje tekst z deklaracjami i zgłasza brakujące odwołania", () => {
        const p = sample(); const text = programText(p, library).map(l => "  ".repeat(l.depth) + l.text);
        expect(text).toContain("VAR"); expect(text).toContain("  i : INT := 0"); expect(text).toContain("MOVE PTP Home v=30%");
        expect(text).toContain("  MOVE PTP cel v=40%"); expect(text).toContain("END_FOR");
        expect(problems(p, library)).toEqual([]);
        const broken = setParam(renameRefs(p, "var", "cel", "zniknęła"), "a", "position", null);
        expect(problems(broken, library).map(x => x.text)).toEqual(["jedź …: brak „pozycja”", "Brak zmiennej „zniknęła”"]);
    });
});

describe("zamiana rozkazu bloku", () => {
    const motion: CommandLibrary = { schema_version: 1, categories: [{ id: "ruch", label: "Ruch" }], io: { outputs: [], inputs: [] }, commands: [
        { id: "move_ptp", category: "ruch", label: "PTP do {position} v {speed} ⌀ {blend_mm}", params: [{ name: "position", kind: "position", label: "pozycja" },
            { name: "speed", kind: "number", label: "prędkość", unit: "%", default: 30, min: 1, max: 100 }, { name: "blend_mm", kind: "number", label: "promień", unit: "mm", default: 0, min: 0, max: 200 }] },
        { id: "move_lin", category: "ruch", label: "LIN do {position} v {speed} ⌀ {blend_mm}", params: [{ name: "position", kind: "position", label: "pozycja" },
            { name: "speed", kind: "number", label: "prędkość", unit: "mm/s", default: 50, min: 0.1, max: 10000 }, { name: "blend_mm", kind: "number", label: "promień", unit: "mm", default: 0, min: 0, max: 200 }] },
        library.commands[1]] };
    it("PTP ↔ LIN: ten sam blok, parametry zostają, wartość spoza zakresu → domyślna", () => {
        const [ptp, lin, loop] = motion.commands;
        expect(compatibleCommands(motion, ptp).map(c => c.id)).toEqual(["move_lin"]);
        expect(compatibleCommands(motion, loop)).toEqual([]);
        const p = emptyProgram("T"); p.blocks = [{ id: "m", command: "move_ptp", params: { position: { position: "A" }, speed: 40, blend_mm: 5 } }];
        const toLin = changeCommand(p, "m", lin);
        expect(toLin.program.blocks[0]).toEqual({ id: "m", command: "move_lin", params: { position: { position: "A" }, speed: 40, blend_mm: 5 } });
        expect(toLin.reset).toEqual([]); expect(p.blocks[0].command).toBe("move_ptp");
        toLin.program.blocks[0].params.speed = 250;
        const back = changeCommand(toLin.program, "m", ptp);
        expect(back.program.blocks[0].params).toEqual({ position: { position: "A" }, speed: 30, blend_mm: 5 }); expect(back.reset).toEqual(["prędkość"]);
    });
});
