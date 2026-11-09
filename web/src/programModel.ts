// Model programu ruchowego (edytor blokowy): typy zgodne z robot_web_bridge/programs.py i operacje na drzewie bloków.
// Język programu i wykonanie są poza zakresem — tekst służy wyłącznie do podglądu.
export type VarType = "INT" | "REAL" | "BOOL" | "POZYCJA" | "TEKST";
export const VAR_TYPES: VarType[] = ["INT", "REAL", "BOOL", "POZYCJA", "TEKST"];
export type Variable = { name: string; type: VarType; init: number | boolean | string; unit?: string; note?: string; min?: number | null; max?: number | null };
export type Position = { name: string; joints_deg: number[]; tcp?: number[] | null; note?: string; source?: string; created_at?: string };
export type Ref = { var: string } | { position: string };
export type Value = number | boolean | string | null | Ref;
export type Block = { id: string; command: string; params: Record<string, Value>; disabled?: boolean; body?: Block[]; else?: Block[] };
export type ParamKind = "position" | "number" | "variable" | "condition" | "text" | "bool" | "output" | "input" | "value";
export type ParamDef = { name: string; kind: ParamKind; label: string; unit?: string; default?: Value; min?: number; max?: number; integer?: boolean; types?: VarType[] };
export type SlotDef = { name: string; label?: string; text?: string };
export type Command = { id: string; category: string; icon?: string; label: string; text?: string; end?: string; slots?: SlotDef[]; params?: ParamDef[] };
export type CommandLibrary = { schema_version: number; categories: { id: string; label: string }[]; io?: { outputs?: string[]; inputs?: string[]; note?: string }; commands: Command[] };
export type Program = { schema_version?: number; id?: string; name: string; description?: string; created_at?: string; updated_at?: string; revision?: string;
    variables: Variable[]; positions: Position[]; blocks: Block[];
    // Zatwierdzenie operatora do wykonania — ważne tylko dla tej rewizji (ustawia backend, nie zapis programu).
    approval?: { revision: string; approved_at: string; client?: string } };
export type DropTarget = { parent: string | null; slot: string; index: number };
export type DragPayload = { kind: "command"; command: string } | { kind: "block"; id: string } | { kind: "variable"; name: string } | { kind: "position"; name: string };

export const NAME_RE = /^\p{L}[\p{L}\p{N}_]{0,39}$/u;
export const isRef = (v: Value): v is Ref => typeof v === "object" && v !== null;
export const refName = (v: Ref) => "var" in v ? v.var : v.position;
const clone = <T,>(x: T): T => structuredClone(x);

export function emptyProgram(name: string): Program { return { name, description: "", variables: [], positions: [], blocks: [] }; }
export function commandMap(library: CommandLibrary | null) { return new Map((library?.commands ?? []).map(c => [c.id, c])); }
export function slotsOf(command: Command | undefined) { return command?.slots ?? []; }
export function slotList(block: Block, slot: string): Block[] { return (block as unknown as Record<string, Block[] | undefined>)[slot] ?? []; }
function setSlot(block: Block, slot: string, list: Block[]) { (block as unknown as Record<string, Block[]>)[slot] = list; }

let counter = 0;
export function newId(existing: Set<string>) {
    let id; do { id = `b${Date.now().toString(36)}${(counter++).toString(36)}`; } while (existing.has(id));
    return id;
}
export function allIds(blocks: Block[], out = new Set<string>(), slotNames = ["body", "else"]): Set<string> {
    for (const b of blocks) { out.add(b.id); for (const s of slotNames) allIds(slotList(b, s), out, slotNames); }
    return out;
}
export function newBlock(command: Command, existing: Set<string>): Block {
    const params: Record<string, Value> = {};
    for (const p of command.params ?? []) params[p.name] = p.default !== undefined ? clone(p.default) : null;
    const block: Block = { id: newId(existing), command: command.id, params };
    for (const s of slotsOf(command)) setSlot(block, s.name, []);
    return block;
}

// ----- nawigacja po drzewie -----
function walk(blocks: Block[], visit: (b: Block, parent: Block | null, slot: string, index: number) => boolean | void, parent: Block | null = null, slot = "blocks"): boolean {
    for (let i = 0; i < blocks.length; i++) {
        const b = blocks[i];
        if (visit(b, parent, slot, i)) return true;
        for (const s of ["body", "else"]) if (walk(slotList(b, s), visit, b, s)) return true;
    }
    return false;
}
export function findBlock(program: Program, id: string) {
    let found: { block: Block; parent: Block | null; slot: string; index: number } | null = null;
    walk(program.blocks, (block, parent, slot, index) => { if (block.id === id) { found = { block, parent, slot, index }; return true; } });
    return found as { block: Block; parent: Block | null; slot: string; index: number } | null;
}
function listFor(program: Program, parent: string | null, slot: string): Block[] {
    if (parent === null) return program.blocks;
    const found = findBlock(program, parent);
    if (!found) throw Error("Nie znaleziono bloku docelowego");
    if (!slotList(found.block, slot) || !(slot in found.block)) setSlot(found.block, slot, []);
    return slotList(found.block, slot);
}
export function isDescendant(program: Program, ancestor: string, candidate: string | null): boolean {
    if (candidate === null) return false;
    if (candidate === ancestor) return true;
    const a = findBlock(program, ancestor);
    if (!a) return false;
    let hit = false;
    walk(["body", "else"].flatMap(s => slotList(a.block, s)), b => { if (b.id === candidate) { hit = true; return true; } });
    return hit;
}
export function countBlocks(blocks: Block[]): number { let n = 0; walk(blocks, () => { n++; }); return n; }

// ----- operacje (zwracają nowy program) -----
export function insertBlock(program: Program, target: DropTarget, block: Block): Program {
    const next = clone(program); const list = listFor(next, target.parent, target.slot);
    list.splice(Math.max(0, Math.min(target.index, list.length)), 0, clone(block));
    return next;
}
export function removeBlock(program: Program, id: string): { program: Program; block: Block | null } {
    const next = clone(program); const found = findBlock(next, id);
    if (!found) return { program, block: null };
    const list = found.parent ? slotList(found.parent, found.slot) : next.blocks;
    const [block] = list.splice(found.index, 1);
    return { program: next, block };
}
export function moveBlock(program: Program, id: string, target: DropTarget): Program {
    if (isDescendant(program, id, target.parent)) return program;   // nie wolno wstawić bloku do jego wnętrza
    const source = findBlock(program, id); if (!source) return program;
    const sameList = (source.parent?.id ?? null) === target.parent && source.slot === (target.parent === null ? "blocks" : target.slot);
    const { program: removed, block } = removeBlock(program, id); if (!block) return program;
    const index = sameList && target.index > source.index ? target.index - 1 : target.index;
    return insertBlock(removed, { ...target, index }, block);
}
export function duplicateBlock(program: Program, id: string): Program {
    const found = findBlock(program, id); if (!found) return program;
    const ids = allIds(program.blocks); const copy = clone(found.block);
    walk([copy], b => { b.id = newId(ids); ids.add(b.id); });
    return insertBlock(program, { parent: found.parent?.id ?? null, slot: found.slot === "blocks" ? "body" : found.slot, index: found.index + 1 }, copy);
}
export function setParam(program: Program, id: string, name: string, value: Value): Program {
    const next = clone(program); const found = findBlock(next, id);
    if (found) found.block.params = { ...found.block.params, [name]: value };
    return next;
}
// Rozkazy, na które można zamienić blok bez usuwania: ta sama kategoria i te same gniazda, co najmniej jeden wspólny parametr.
export function compatibleCommands(library: CommandLibrary | null, command: Command | undefined): Command[] {
    if (!library || !command) return [];
    const slots = slotsOf(command).map(s => s.name).join("|"), names = new Set((command.params ?? []).map(p => p.name));
    return library.commands.filter(c => c.id !== command.id && c.category === command.category && slotsOf(c).map(s => s.name).join("|") === slots
        && (c.params ?? []).some(p => names.has(p.name)));
}
// Zamiana rozkazu bloku: id, gniazda i parametry o tej samej nazwie zostają; liczba spoza zakresu nowego rozkazu → domyślna.
export function changeCommand(program: Program, id: string, command: Command): { program: Program; reset: string[] } {
    const next = clone(program); const found = findBlock(next, id); const reset: string[] = [];
    if (!found) return { program, reset };
    const old = found.block.params, params: Record<string, Value> = {};
    for (const p of command.params ?? []) {
        const value = old[p.name], fallback = p.default !== undefined ? clone(p.default) : null;
        const outside = typeof value === "number" && ((p.min !== undefined && value < p.min) || (p.max !== undefined && value > p.max) || (p.integer && !Number.isInteger(value)));
        if (value === undefined || value === null || value === "") params[p.name] = fallback;
        else if (outside) { params[p.name] = fallback; reset.push(p.label); }
        else params[p.name] = value;
    }
    found.block.command = command.id; found.block.params = params;
    return { program: next, reset };
}
export function toggleDisabled(program: Program, id: string): Program {
    const next = clone(program); const found = findBlock(next, id);
    if (found) { if (found.block.disabled) delete found.block.disabled; else found.block.disabled = true; }
    return next;
}

// ----- zmienne i pozycje -----
export function nameError(name: string, taken: string[], current?: string): string | null {
    if (!NAME_RE.test(name)) return "Nazwa: zaczyna się literą, dalej litery, cyfry lub _ (do 40 znaków)";
    if (name !== current && taken.includes(name)) return "Nazwa jest już używana";
    return null;
}
export function usage(program: Program, kind: "var" | "position", name: string): number {
    let n = 0;
    walk(program.blocks, b => { for (const v of Object.values(b.params)) if (isRef(v) && kind in v && refName(v) === name) n++; });
    return n;
}
export function renameRefs(program: Program, kind: "var" | "position", from: string, to: string): Program {
    const next = clone(program);
    walk(next.blocks, b => { for (const [k, v] of Object.entries(b.params)) if (isRef(v) && kind in v && refName(v) === from) b.params[k] = kind === "var" ? { var: to } : { position: to }; });
    return next;
}
export function defaultInit(type: VarType): Variable["init"] { return type === "BOOL" ? false : type === "INT" || type === "REAL" ? 0 : ""; }
export function parseInit(type: VarType, raw: string): Variable["init"] | null {
    if (type === "BOOL") return raw === "true" || raw === "TRUE";
    if (type === "INT") return /^-?\d+$/.test(raw.trim()) ? Number(raw) : null;
    if (type === "REAL") { const v = Number(raw.replace(",", ".")); return raw.trim() !== "" && Number.isFinite(v) ? v : null; }
    return raw;
}

// ----- zgodność przeciąganych zmiennych/pozycji z parametrem -----
export function accepts(param: ParamDef, drag: DragPayload | null, variables: Variable[]): boolean {
    if (!drag) return false;
    if (drag.kind === "position") return param.kind === "position" || param.kind === "value";
    if (drag.kind !== "variable") return false;
    const type = variables.find(v => v.name === drag.name)?.type;
    if (!type) return false;
    if (param.types && !param.types.includes(type)) return false;
    switch (param.kind) {
        case "position": return type === "POZYCJA";
        case "number": return type === "INT" || type === "REAL";
        case "bool": case "condition": return type === "BOOL";
        case "text": return type === "TEKST";
        case "variable": case "value": return true;
        default: return false;
    }
}
export function dropValue(param: ParamDef, drag: DragPayload): Value {
    if (drag.kind === "position") return { position: drag.name };
    if (drag.kind === "variable") return { var: drag.name };
    return null;
}

// ----- tekst (podgląd) i problemy -----
export function formatValue(value: Value, param?: ParamDef): string {
    if (value === null || value === undefined || value === "") return "?";
    if (isRef(value)) return refName(value);
    if (typeof value === "boolean") return value ? "TRUE" : "FALSE";
    if (typeof value === "number") return param?.unit ? (param.unit === "%" ? `${value}%` : `${value} ${param.unit}`) : String(value);
    return param?.kind === "text" ? `"${value}"` : value;
}
export function fillTemplate(template: string, block: Block, command: Command) {
    return template.replace(/\{(\w+)\}/g, (_, name) => formatValue(block.params[name], command.params?.find(p => p.name === name)));
}
export type TextLine = { key: string; block?: string; depth: number; text: string; kind?: "decl" | "end" | "comment" };
export function programText(program: Program, library: CommandLibrary | null): TextLine[] {
    const commands = commandMap(library);
    const lines: TextLine[] = [{ key: "var", depth: 0, text: "VAR", kind: "decl" }];
    for (const v of program.variables) lines.push({ key: `var-${v.name}`, depth: 1, kind: "decl",
        text: `${v.name} : ${v.type} := ${typeof v.init === "boolean" ? String(v.init).toUpperCase() : v.type === "TEKST" || v.type === "POZYCJA" ? (v.init === "" ? "''" : v.init) : v.init}${v.unit ? " " + v.unit : ""}${v.note ? "   // " + v.note : ""}` });
    lines.push({ key: "var-end", depth: 0, text: "END_VAR", kind: "decl" });
    if (program.positions.length) {
        lines.push({ key: "pos", depth: 0, text: "// Pozycje (J1–J6 °):", kind: "comment" });
        for (const p of program.positions) lines.push({ key: `pos-${p.name}`, depth: 0, kind: "comment", text: `//   ${p.name} = [${p.joints_deg.map(v => +v.toFixed(2)).join(", ")}]` });
    }
    lines.push({ key: "blank", depth: 0, text: "" });
    const emit = (blocks: Block[], depth: number) => {
        for (const b of blocks) {
            const c = commands.get(b.command);
            const head = c ? fillTemplate(c.text ?? c.label, b, c) : `?? ${b.command}`;
            lines.push({ key: b.id, block: b.id, depth, text: b.disabled ? `// (wyłączony) ${head}` : head, kind: b.disabled ? "comment" : undefined });
            slotsOf(c).forEach((s, k) => {
                if (k > 0) lines.push({ key: `${b.id}-${s.name}`, block: b.id, depth, text: s.text ?? s.name.toUpperCase(), kind: "end" });
                emit(slotList(b, s.name), depth + 1);
            });
            if (c?.end) lines.push({ key: `${b.id}-end`, block: b.id, depth, text: c.end, kind: "end" });
        }
    };
    emit(program.blocks, 0);
    return lines;
}
export type Problem = { block?: string; text: string };
export function problems(program: Program, library: CommandLibrary | null): Problem[] {
    const commands = commandMap(library), vars = new Set(program.variables.map(v => v.name)), pos = new Set(program.positions.map(p => p.name));
    const out: Problem[] = [];
    walk(program.blocks, b => {
        const c = commands.get(b.command);
        if (!c) { out.push({ block: b.id, text: `Nieznany rozkaz „${b.command}”` }); return; }
        for (const p of c.params ?? []) {
            const v = b.params[p.name];
            if (v === null || v === undefined || v === "") out.push({ block: b.id, text: `${c.label.split(" ")[0]} …: brak „${p.label}”` });
            else if (typeof v === "number" && !b.disabled) {
                const unit = p.unit ? ` ${p.unit}` : "";
                if (p.min !== undefined && v < p.min) out.push({ block: b.id, text: `${p.label}: ${v}${unit} poniżej minimum ${p.min}${unit}` });
                else if (p.max !== undefined && v > p.max) out.push({ block: b.id, text: `${p.label}: ${v}${unit} powyżej maksimum ${p.max}${unit}` });
                else if (p.integer && !Number.isInteger(v)) out.push({ block: b.id, text: `${p.label}: wymagana liczba całkowita` });
            }
            else if (isRef(v) && "var" in v && !vars.has(v.var)) out.push({ block: b.id, text: `Brak zmiennej „${v.var}”` });
            else if (isRef(v) && "position" in v && !pos.has(v.position)) out.push({ block: b.id, text: `Brak pozycji „${v.position}”` });
        }
    });
    return out;
}
export function labelParts(label: string): ({ text: string } | { param: string })[] {
    return label.split(/(\{\w+\})/).filter(Boolean).map(part => /^\{\w+\}$/.test(part) ? { param: part.slice(1, -1) } : { text: part });
}
