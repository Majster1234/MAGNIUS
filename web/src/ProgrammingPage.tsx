// Tryb „Programowanie” (wariant v2b): edytor blokowy z przeciąganiem, zmienne i pozycje programu, biblioteka,
// zapis programów w backendzie. Sprawdzenie i wykonanie: wykonawca w moście (kursor robota i kursor programu, ciągi ruchów
// z promieniem pominięcia punktu) — panel „Wykonanie”, logika w programRun.ts.
import { Fragment, useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState, type DragEvent, type ReactNode } from "react";
import { accepts, addCase, allIds, canDrop, changeCommand, checkExpression, commandMap, commandName, compatibleCommands, countBlocks, defaultInit, dropValue, duplicateBlock,
    emptyProgram, EXPRESSION_FUNCTIONS, findBlock, formatValue, insertBlock, isRef, labelNames, labelParts, moveBlock, nameError, newBlock, parseInit, problems as findProblems,
    programText, removeBlock, renameRefs, setParam, shapeOf, slotList, slotsOf, toggleDisabled, usage, VAR_TYPES, type Block, type Command, type CommandLibrary, type DragPayload, type DropTarget, type ParamDef,
    type Position, type Program, type Value, type Variable, type VarType } from "./programModel";
import { animationsEnabled } from "./animations";
import { activeConfigurationValues, currentJoints, programApi, ProgramApiError, useProgramRun, type LibrarySnapshot, type PreflightStep, type ProgramSummary } from "./programApi";
import { describe as describeViolations, jointRanges, limitProblems, poseLimits, violations, type JointRange } from "./programLimits";
import { ancestorIds, blendBadge, cursorClasses, phaseLabels, runFor, runStateLabels, timelineBars, type RunStatus, type TimelineEntry } from "./programRun";

const LAST_KEY = "robot.programming.last";
type LaunchState = "pending" | "running" | "ok" | "warn" | "error" | "action" | "skipped";
type LaunchStep = { id: "save" | PreflightStep["id"] | "check" | "approve" | "start"; label: string; state: LaunchState; detail: string; items?: PreflightStep["items"] };
const LAUNCH_STEPS: Omit<LaunchStep, "state" | "detail">[] = [
    { id: "save", label: "Zapis programu" }, { id: "program", label: "Program bez uwag" }, { id: "limits", label: "Pozycje w zakresie osi" },
    { id: "robot", label: "Stan robota" }, { id: "check", label: "Symulacja programu" }, { id: "approve", label: "Potwierdzenie i zatwierdzenie" },
    { id: "drives", label: "Napędy załączone (ENABLE)" }, { id: "start", label: "Start programu" }];
const LAUNCH_ICON: Record<LaunchState, string> = { pending: "○", running: "◌", ok: "✓", warn: "⚠", error: "✕", action: "↻", skipped: "–" };
const ZOOM_KEY = "robot.programming.zoom";
const ZOOMS = [0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.25];
const readZoom = () => { try { const z = Number(localStorage.getItem(ZOOM_KEY)); return z >= 0.4 && z <= 1.5 ? z : 1; } catch { return 1; } };
const TYPE_HINT: Record<VarType, string> = { INT: "całkowita", REAL: "liczba", BOOL: "tak / nie", POZYCJA: "📍 punkt", TEKST: "napis" };
const fmtTime = (iso?: string) => iso ? new Date(iso).toLocaleTimeString("pl-PL", { hour: "2-digit", minute: "2-digit" }) : "";
const initText = (v: Variable) => typeof v.init === "boolean" ? (v.init ? "TRUE" : "FALSE") : v.init === "" ? "—" : `${v.init}${v.unit ? " " + v.unit : ""}`;
const uniqueName = (base: string, taken: string[]) => { let n = taken.length + 1, name = `${base}_${n}`; while (taken.includes(name)) name = `${base}_${++n}`; return name; };

export function ProgrammingPage() {
    const [library, setLibrary] = useState<LibrarySnapshot | null>(null);
    const [list, setList] = useState<ProgramSummary[]>([]);
    const [program, setProgram] = useState<Program | null>(null);
    const [saved, setSaved] = useState<Program | null>(null);
    const [history, setHistory] = useState<{ past: Program[]; future: Program[] }>({ past: [], future: [] });
    const [view, setView] = useState<"blocks" | "text">("blocks");
    const [tab, setTab] = useState<"blocks" | "variables" | "positions">("blocks");
    const [selected, setSelected] = useState<string | null>(null);
    const [selectedPos, setSelectedPos] = useState<string | null>(null);
    const [message, setMessage] = useState<{ text: string; tone: "ok" | "warn" | "error" } | null>(null);
    const [busy, setBusy] = useState(false);
    const [varModal, setVarModal] = useState<{ edit: Variable | null } | null>(null);
    const [newDialog, setNewDialog] = useState<"new" | "duplicate" | null>(null);
    const [dragging, setDragging] = useState<DragPayload | null>(null);
    // Aktywna konfiguracja robota (zakresy osi, geometria podglądu 3D).
    const [configValues, setConfigValues] = useState<Record<string, unknown> | null>(null);
    useEffect(() => { let closed = false; void activeConfigurationValues().then(v => { if (!closed) setConfigValues(v); }); return () => { closed = true; }; }, []);
    const ranges = useMemo(() => jointRanges(configValues), [configValues]);
    // Proces „▶ Uruchom”: kroki doprowadzenia programu i robota do stanu startu.
    const [launch, setLaunch] = useState<{ steps: LaunchStep[]; active: boolean } | null>(null);
    const launchAbort = useRef(false);
    // Blok właśnie upuszczony: rozsunięcie programu i „lądowanie” (CSS; pomijane przy wyłączonych animacjach).
    const [landing, setLanding] = useState<string | null>(null);
    useEffect(() => { if (!landing) return; const t = setTimeout(() => setLanding(null), 1100); return () => clearTimeout(t); }, [landing]);
    const land = (id: string) => { if (animationsEnabled()) setLanding(id); };
    const drag = useRef<DragPayload | null>(null);
    const commands = library?.commands ?? null;
    const cmap = useMemo(() => commandMap(commands), [commands]);
    const dirty = !!program && JSON.stringify(program) !== JSON.stringify(saved);
    const notify = (text: string, tone: "ok" | "warn" | "error" = "ok") => setMessage({ text, tone });
    useEffect(() => { if (message?.tone !== "ok") return; const t = setTimeout(() => setMessage(null), 4000); return () => clearTimeout(t); }, [message]);

    const refreshList = useCallback(async () => { const data = await programApi.list(); setList(data.items); return data.items; }, []);
    const open = useCallback(async (id: string) => {
        const reply = await programApi.get(id); setProgram(reply.program); setSaved(reply.program); setHistory({ past: [], future: [] });
        setSelected(null); setSelectedPos(reply.program.positions[0]?.name ?? null);
        try { localStorage.setItem(LAST_KEY, id); } catch { /* bez pamięci przeglądarki */ }
    }, []);
    useEffect(() => {
        let closed = false;
        (async () => {
            try {
                const [lib, items] = await Promise.all([programApi.library(), refreshList()]);
                if (closed) return; setLibrary(lib); if (lib.warning) notify(lib.warning, "warn");
                let last: string | null = null; try { last = localStorage.getItem(LAST_KEY); } catch { /* */ }
                const id = items.find(i => i.id === last && !i.broken)?.id ?? items.find(i => !i.broken)?.id;
                if (id) await open(id);
            } catch (error) { if (!closed) notify(`Magazyn programów niedostępny: ${(error as Error).message}`, "error"); }
        })();
        return () => { closed = true; };
    }, [open, refreshList]);

    const update = useCallback((change: (p: Program) => Program) => {
        setProgram(current => { if (!current) return current; const next = change(current); if (next === current) return current;
            setHistory(h => ({ past: [...h.past.slice(-49), current], future: [] })); return next; });
    }, []);
    const undo = () => setHistory(h => { if (!h.past.length || !program) return h; const prev = h.past[h.past.length - 1]; setProgram(prev); return { past: h.past.slice(0, -1), future: [program, ...h.future] }; });
    const redo = () => setHistory(h => { if (!h.future.length || !program) return h; const [next, ...rest] = h.future; setProgram(next); return { past: [...h.past, program], future: rest }; });

    const screen = useRef<HTMLDivElement>(null);
    useEffect(() => {
        const el = screen.current; if (!el) return;
        const wheel = (e: WheelEvent) => { if (e.ctrlKey || e.metaKey) e.preventDefault(); };
        el.addEventListener("wheel", wheel, { passive: false }); return () => el.removeEventListener("wheel", wheel);
    }, []);

    async function save() {
        if (!program?.id || busy) return; setBusy(true);
        try {
            const reply = await programApi.save(program.id, program, saved?.revision);
            setProgram(reply.program); setSaved(reply.program); await refreshList();
            notify(reply.problems.length ? `Zapisano — ${reply.problems.length} uwag do uzupełnienia` : "Zapisano program", reply.problems.length ? "warn" : "ok");
        } catch (error) {
            notify((error as Error).message + (error instanceof ProgramApiError && error.status === 409 ? " — użyj „Wczytaj ponownie”" : ""), "error");
        } finally { setBusy(false); }
    }
    async function createProgram(name: string, mode: "new" | "duplicate") {
        try {
            const base = mode === "duplicate" && program ? { ...structuredClone(program), name } : emptyProgram(name);
            delete base.id; delete base.revision; delete base.created_at; delete base.updated_at;
            const reply = await programApi.create(base); await refreshList(); await open(reply.program.id!); setNewDialog(null);
            notify(mode === "duplicate" ? "Utworzono kopię programu" : "Utworzono nowy program");
        } catch (error) { notify((error as Error).message, "error"); }
    }
    const approved = !!program?.approval && program.approval.revision === program.revision && !dirty;
    async function approve() {
        if (!program?.id || !program.revision) return;
        if (!window.confirm(`Zatwierdzić program „${program.name}” (rewizja ${program.revision}) do wykonania na robocie?\n\nWykonanie uruchamia ruch fizyczny. Każda zmiana programu unieważni to zatwierdzenie.`)) return;
        try { const reply = await programApi.approve(program.id, program.revision); setProgram(reply.program); setSaved(reply.program); await refreshList(); notify("Program zatwierdzony do wykonania (ta rewizja)"); }
        catch (error) { notify((error as Error).message, "error"); }
    }
    async function revoke() {
        if (!program?.id) return;
        try { const reply = await programApi.revoke(program.id); setProgram(reply.program); setSaved(reply.program); await refreshList(); notify("Cofnięto zatwierdzenie"); }
        catch (error) { notify((error as Error).message, "error"); }
    }
    async function deleteProgram() {
        if (!program?.id || !window.confirm(`Usunąć program „${program.name}”? Plik trafi do kosza (programs/.trash).`)) return;
        try { await programApi.remove(program.id); const items = await refreshList(); setProgram(null); setSaved(null);
            const next = items.find(i => !i.broken); if (next) await open(next.id); notify("Program przeniesiony do kosza"); }
        catch (error) { notify((error as Error).message, "error"); }
    }
    async function switchTo(id: string) {
        if (dirty && !window.confirm("Masz niezapisane zmiany. Porzucić je i otworzyć inny program?")) return;
        try { await open(id); } catch (error) { notify((error as Error).message, "error"); }
    }

    // ----- skróty klawiszowe -----
    useEffect(() => {
        const key = (e: KeyboardEvent) => {
            const typing = (e.target as HTMLElement)?.closest?.("input,textarea,select");
            if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "s") { e.preventDefault(); void save(); return; }
            if (typing) return;
            if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "z") { e.preventDefault(); if (e.shiftKey) redo(); else undo(); }
            else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "y") { e.preventDefault(); redo(); }
            else if (selected && (e.key === "Delete" || e.key === "Backspace")) { e.preventDefault(); update(p => removeBlock(p, selected).program); setSelected(null); }
            else if (selected && (e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "d") { e.preventDefault(); update(p => duplicateBlock(p, selected)); }
        };
        window.addEventListener("keydown", key); return () => window.removeEventListener("keydown", key);
    });
    useEffect(() => { const esc = (e: KeyboardEvent) => { if (e.key === "Escape") { setVarModal(null); setNewDialog(null); } };
        window.addEventListener("keydown", esc); return () => window.removeEventListener("keydown", esc); }, []);
    useEffect(() => { const warn = (e: BeforeUnloadEvent) => { if (dirty) { e.preventDefault(); e.returnValue = ""; } };
        window.addEventListener("beforeunload", warn); return () => window.removeEventListener("beforeunload", warn); }, [dirty]);

    // ----- przeciąganie -----
    const startDrag = (payload: DragPayload) => (e: DragEvent) => {
        drag.current = payload; setDragging(payload); e.dataTransfer.effectAllowed = payload.kind === "block" ? "move" : "copy";
        e.dataTransfer.setData("text/plain", JSON.stringify(payload)); e.stopPropagation();
    };
    const endDrag = () => { drag.current = null; setDragging(null); };
    const dropBlock = (target: DropTarget) => {
        const payload = drag.current; endDrag(); if (!payload || !program) return;
        if (payload.kind === "command") {
            const c = cmap.get(payload.command); if (!c || !canDrop(program, cmap, c.id, target)) return;
            const block = newBlock(c, allIds(program.blocks), cmap);
            if (c.id === "label") block.params.name = uniqueName("Etykieta", labelNames(program));
            update(p => insertBlock(p, target, block)); setSelected(block.id); land(block.id);
        }
        else if (payload.kind === "block") {
            const found = findBlock(program, payload.id); if (!found || !canDrop(program, cmap, found.block.command, target)) return;
            const next = moveBlock(program, payload.id, target); if (next !== program) { update(() => next); land(payload.id); }
        }
    };
    const dropParam = (block: string, param: ParamDef) => { const payload = drag.current; endDrag(); if (payload && program && accepts(param, payload, program.variables)) update(p => setParam(p, block, param.name, dropValue(param, payload))); };

    const issues = useMemo(() => program ? [...findProblems(program, commands), ...limitProblems(program, ranges)] : [], [program, commands, ranges]);
    // Zaznaczony blok ruchu → jego pozycja w podglądzie 3D (z limitami osi).
    useEffect(() => {
        if (!program || !selected) return;
        const value = findBlock(program, selected)?.block.params.position;
        const name = value !== undefined && isRef(value) ? ("position" in value ? value.position : program.variables.find(v => v.name === value.var && v.type === "POZYCJA")?.init) : undefined;
        if (typeof name === "string" && program.positions.some(p => p.name === name)) setSelectedPos(name);
    }, [selected]);   // eslint-disable-line react-hooks/exhaustive-deps
    const text = useMemo(() => program ? programText(program, commands) : [], [program, commands]);
    const runStatus = useProgramRun();
    const run = runFor(runStatus, program);
    const executing = !!run?.active && run.mode === "execute";
    const ancestors = useMemo(() => new Set(program && run?.active ? [...ancestorIds(program, run.robot_cursor?.block_id), ...ancestorIds(program, run.program_cursor.block_id)] : []),
        [program, run?.active, run?.robot_cursor?.block_id, run?.program_cursor.block_id]);
    const timeline = useMemo(() => new Map((run && run.revision === program?.revision ? run.timeline : []).map(e => [e.block_id, e])), [run, program?.revision]);
    const dragCommand = useMemo(() => dragging?.kind === "command" ? dragging.command : dragging?.kind === "block" && program ? findBlock(program, dragging.id)?.block.command : undefined,
        [dragging, program]);
    // Skok ↔ etykieta: zaznaczenie jednego podświetla drugie.
    const linked = useMemo(() => { const b = program && selected ? findBlock(program, selected)?.block : undefined;
        const v = b?.command === "goto" ? b.params.target : b?.command === "label" ? b.params.name : null; return typeof v === "string" && v ? v : null; }, [program, selected]);
    const ctx: CanvasContext = { program: program!, cmap, library: commands, selected, setSelected, dragging, startDrag, endDrag, dropBlock, dropParam, update, issues,
        run, ancestors, timeline, locked: executing, dragCommand, linked, landing, land };
    // ▶ Uruchom: zapis → analiza (program, zakresy osi, stan robota, napędy) → symulacja → potwierdzenie operatora →
    // zatwierdzenie rewizji → załączenie napędów → start. Każdy krok widoczny w panelu Wykonanie; błąd zatrzymuje proces.
    async function launchProgram() {
        if (!program?.id || launch?.active) return;
        const id = program.id;
        launchAbort.current = false;
        setLaunch({ steps: LAUNCH_STEPS.map(s => ({ ...s, state: "pending", detail: "" })), active: true });
        const mark = (step: LaunchStep["id"], state: LaunchState, detail = "") =>
            setLaunch(l => l && { ...l, steps: l.steps.map(s => s.id === step ? { ...s, state, detail: detail || s.detail } : s) });
        const finish = (text: string, tone: "ok" | "warn" | "error") => { setLaunch(l => l && { ...l, active: false }); notify(text, tone); };
        const wait = (ms: number) => new Promise(resolve => setTimeout(resolve, ms));
        let step: LaunchStep["id"] = "save";
        try {
            let current = program;
            if (dirty) { mark("save", "running"); const reply = await programApi.save(id, program, saved?.revision); current = reply.program; setProgram(reply.program); setSaved(reply.program); await refreshList(); mark("save", "ok", "zapisano"); }
            else mark("save", "ok", "bez zmian");
            step = "program";
            if (issues.length) { mark("program", "error", issues.slice(0, 4).map(i => i.text).join("; ")); finish("Uruchomienie wstrzymane: program ma uwagi (panel Sprawdzenie)", "error"); return; }
            mark("program", "running");
            const pre = await programApi.preflight(id);
            for (const item of pre.steps) mark(item.id, item.state === "action" ? "action" : item.state, item.detail);
            setLaunch(l => l && { ...l, steps: l.steps.map(s => ({ ...s, items: pre.steps.find(p => p.id === s.id)?.items ?? s.items })) });
            if (!pre.ok) { finish("Uruchomienie wstrzymane: analiza wykazała błędy (panel Wykonanie)", "error"); return; }
            step = "check"; mark("check", "running", "symulacja całego programu od bieżącej pozycji");
            const started = await programApi.run(id, "check", current.revision);
            let result = started;
            while (!launchAbort.current) { await wait(250); result = await programApi.runStatus(); if (result.run_id === started.run_id && !result.active) break; }
            if (launchAbort.current) { mark("check", "error", "przerwano"); finish("Uruchomienie przerwane", "warn"); return; }
            if (result.state !== "succeeded") { mark("check", "error", result.message || "sprawdzenie nieudane"); finish(`Sprawdzenie nieudane: ${result.message}`, "error"); return; }
            const moves = result.timeline.filter(e => !e.kind).length;
            mark("check", result.warnings.length ? "warn" : "ok", `${moves} ruchów · ${result.chains} ciąg(i) · ~${(result.total_s ?? 0).toFixed(1)} s${result.warnings.length ? ` · ${result.warnings.length} uwag` : ""}`);
            step = "approve";
            const drives = pre.steps.find(s => s.id === "drives")?.disabled ?? [];
            const approvedNow = current.approval?.revision === current.revision;
            if (!window.confirm(`Uruchomić program „${current.name}” na robocie?\n\n`
                + `• ${moves} ruchów, ${result.chains} ciąg(i), ok. ${(result.total_s ?? 0).toFixed(1)} s${result.warnings.length ? `, ${result.warnings.length} uwag sprawdzenia` : ""}\n`
                + `${approvedNow ? "" : `• zatwierdzenie rewizji ${current.revision} do wykonania\n`}`
                + `${drives.length ? `• załączenie napędów ${drives.map(a => `J${a}`).join(", ")} (ENABLE=1)\n` : ""}`
                + "• pierwszy ruch prowadzi z bieżącej pozycji robota do pozycji z programu\n\n"
                + "Upewnij się, że strefa pracy jest wolna. Zatrzymanie: ■ STOP w panelu lub wyłącznik bezpieczeństwa.")) {
                mark("approve", "skipped", "anulowano przez operatora"); finish("Uruchomienie anulowane", "warn"); return;
            }
            if (!approvedNow) { mark("approve", "running"); const reply = await programApi.approve(id, current.revision!); current = reply.program; setProgram(reply.program); setSaved(reply.program); await refreshList(); }
            mark("approve", "ok", `rewizja ${current.revision}`);
            step = "drives";
            if (drives.length) {
                mark("drives", "running", `załączanie ${drives.map(a => `J${a}`).join(", ")}`);
                await programApi.enableAll();
                let enabled = false;
                for (let k = 0; k < 20 && !enabled && !launchAbort.current; k++) { await wait(150); enabled = (await programApi.drives()).every(a => a.enable_known && a.enabled); }
                if (!enabled) { mark("drives", "error", "napędy nie potwierdziły ENABLE=1 w 3 s"); finish("Napędy nie zostały załączone", "error"); return; }
                mark("drives", "ok", "J1–J6 załączone");
            }
            step = "start"; mark("start", "running");
            await programApi.run(id, "execute", current.revision);
            mark("start", "ok", "program uruchomiony"); finish("Program uruchomiony", "ok");
        } catch (error) {
            mark(step, "error", (error as Error).message); finish(`Uruchomienie przerwane (${LAUNCH_STEPS.find(s => s.id === step)?.label}): ${(error as Error).message}`, "error");
        }
    }
    async function startRun(mode: "check" | "execute") {
        if (!program?.id) return;
        if (mode === "execute" && !window.confirm(`Uruchomić program „${program.name}” na robocie?\n\nRobot wykona ruchy fizycznie (ciągi PTP/LIN z promieniami z bloków). `
            + "Upewnij się, że strefa pracy jest wolna. Zatrzymanie: ■ STOP w panelu lub wyłącznik bezpieczeństwa.")) return;
        try { await programApi.run(program.id, mode, program.revision); notify(mode === "check" ? "Sprawdzanie programu…" : "Program uruchomiony"); }
        catch (error) { notify(`${mode === "check" ? "Sprawdzenie" : "Uruchomienie"} odrzucone: ${(error as Error).message}`, "error"); }
    }
    async function resumeRun() {
        try { await programApi.resumeRun(); notify("Kontynuacja programu"); }
        catch (error) { notify(`Kontynuacja: ${(error as Error).message}`, "error"); }
    }
    async function stopRun() {
        launchAbort.current = true;
        try { await programApi.stopRun(); notify("Zatrzymano przebieg programu", "warn"); }
        catch (error) { notify(`STOP: ${(error as Error).message}`, "error"); }
    }

    return <div ref={screen} className="prg-screen">
        <header className="prg-header">
            <h1>Programowanie</h1>
            <select aria-label="Program" value={program?.id ?? ""} onChange={e => void switchTo(e.target.value)} disabled={!list.length}>
                {!program && <option value="">— brak programów —</option>}
                {list.map(p => <option key={p.id} value={p.id} disabled={p.broken}>{p.name}{p.broken ? " (uszkodzony plik)" : ""}</option>)}
            </select>
            {program && <input className="prg-name" aria-label="Nazwa programu" value={program.name} maxLength={80} onChange={e => { const name = e.target.value; update(p => ({ ...p, name })); }} />}
            <button className="secondary-button" onClick={() => setNewDialog("new")}>＋ Nowy</button>
            <button className="secondary-button" disabled={!program} onClick={() => setNewDialog("duplicate")}>⧉ Duplikuj</button>
            <button className="secondary-button" disabled={!program} onClick={() => void deleteProgram()}>Usuń</button>
            <span className="prg-viewseg" role="tablist" aria-label="Widok programu">
                <button role="tab" aria-selected={view === "blocks"} onClick={() => setView("blocks")}>▦ Bloki</button>
                <button role="tab" aria-selected={view === "text"} onClick={() => setView("text")}>≡ Tekst</button></span>
            <button className="secondary-button" title="Cofnij (Ctrl+Z)" disabled={!history.past.length} onClick={undo}>↶</button>
            <button className="secondary-button" title="Ponów (Ctrl+Shift+Z)" disabled={!history.future.length} onClick={redo}>↷</button>
            <button className={dirty ? "primary-button" : "secondary-button"} disabled={!program || !dirty || busy} onClick={() => void save()} title="Ctrl+S">{busy ? "Zapisywanie…" : "Zapisz"}</button>
            {program && <span className={`badge ${dirty ? "warn" : "ok"}`}><span className="dot" />{dirty ? "Niezapisane zmiany" : `Zapisano ${fmtTime(saved?.updated_at)}`}</span>}
            {dirty && program?.id && <button className="secondary-button" onClick={() => program.id && void switchTo(program.id)}>Wczytaj ponownie</button>}
            {program && (approved ? <><span className="badge ok" title={`Zatwierdzono ${program.approval?.approved_at ?? ""} z ${program.approval?.client ?? "?"}`}><span className="dot" />Zatwierdzony do wykonania · {fmtTime(program.approval?.approved_at)}</span>
                <button className="secondary-button" onClick={() => void revoke()}>Cofnij zatwierdzenie</button></> :
                <button className="secondary-button" disabled={dirty || issues.length > 0 || !program.id} title={dirty ? "Najpierw zapisz program" : issues.length ? "Najpierw usuń uwagi z panelu Sprawdzenie" : "Pozwala wykonać tę rewizję programu (scripts/run_program.py --execute)"}
                    onClick={() => void approve()}>✔ Zatwierdź do wykonania</button>)}
            <span className="prg-mode"><span />PROGRAMOWANIE · edycja</span>
        </header>
        {message && <div className={`prg-message ${message.tone}`} role="status">{message.text}<button aria-label="Zamknij" onClick={() => setMessage(null)}>×</button></div>}
        {!program ? <section className="prg-empty"><h2>Brak otwartego programu</h2><p>Utwórz pierwszy program. Pliki zapisywane są w <code>{library?.directory ?? "~/.local/state/robot/programs"}</code>.</p>
            <button className="primary-button" onClick={() => setNewDialog("new")}>＋ Nowy program</button></section> :
        <div className="prg-layout">
            <section className="prg-panel prg-left">
                <div className="prg-tabs" role="tablist">
                    <button role="tab" aria-selected={tab === "blocks"} onClick={() => setTab("blocks")}>Bloki</button>
                    <button role="tab" aria-selected={tab === "variables"} onClick={() => setTab("variables")}>Zmienne ({program.variables.length})</button>
                    <button role="tab" aria-selected={tab === "positions"} onClick={() => setTab("positions")}>Pozycje ({program.positions.length})</button></div>
                {tab === "blocks" && <Palette library={commands} startDrag={startDrag} endDrag={endDrag} />}
                {tab === "variables" && <VariablesPanel program={program} library={library} update={update} setLibrary={setLibrary} notify={notify}
                    startDrag={startDrag} endDrag={endDrag} onCreate={() => setVarModal({ edit: null })} onEdit={v => setVarModal({ edit: v })} />}
                {tab === "positions" && <PositionsPanel program={program} library={library} update={update} setLibrary={setLibrary} notify={notify}
                    startDrag={startDrag} endDrag={endDrag} selected={selectedPos} setSelected={setSelectedPos} ranges={ranges} />}
            </section>
            <section className="prg-panel prg-center">
                <div className="prg-panel-head">Program <small>{view === "blocks" ? "schemat blokowy · przeciągaj z palety na łączniki (strzałki)" : "tekst — podgląd tylko do odczytu"}</small><span className="spacer" />
                    <span className="badge">{countBlocks(program.blocks)} bloków</span>{selected && <><button className="secondary-button sm" onClick={() => update(p => duplicateBlock(p, selected))}>⧉ Duplikuj</button>
                    <button className="secondary-button sm" onClick={() => update(p => toggleDisabled(p, selected))}>⊘ Wyłącz/włącz</button>
                    <button className="secondary-button sm" onClick={() => { update(p => removeBlock(p, selected).program); setSelected(null); }}>Usuń blok</button></>}</div>
                {view === "blocks" ? <Canvas ctx={ctx} /> : <TextView lines={text} selected={selected} onSelect={id => { setSelected(id); setView("blocks"); }} />}
            </section>
            <section className="prg-right">
                <RunPanel program={program} run={run} global={runStatus} approved={approved} dirty={dirty} issues={issues.length}
                    onCheck={() => void startRun("check")} onExecute={() => void launchProgram()} onStop={() => void stopRun()} onResume={() => void resumeRun()}
                    launch={launch} onCloseLaunch={() => setLaunch(null)} onPosition={name => setSelectedPos(name)}
                    onBlock={id => { setSelected(id); setView("blocks"); }} />
                <Preview3D program={program} selected={selectedPos} values={configValues} ranges={ranges} />
                <WatchPanel program={program} issues={issues} directory={library?.directory} onIssue={id => { if (id) { setSelected(id); setView("blocks"); } }} />
            </section>
        </div>}
        {varModal && program && <VariableModal program={program} edit={varModal.edit} onClose={() => setVarModal(null)}
            onSave={(variable, previous) => { update(p => { let next = previous && previous.name !== variable.name ? renameRefs(p, "var", previous.name, variable.name) : p;
                next = { ...next, variables: previous ? next.variables.map(v => v.name === previous.name ? variable : v) : [...next.variables, variable] }; return next; });
                setVarModal(null); notify(previous ? `Zmieniono zmienną ${variable.name}` : `Utworzono zmienną ${variable.name}`); }} />}
        {newDialog && <NameDialog title={newDialog === "new" ? "Nowy program" : "Kopia programu"} initial={newDialog === "new" ? "Nowy program" : `${program?.name ?? ""} (kopia)`}
            onCancel={() => setNewDialog(null)} onOk={name => void createProgram(name, newDialog)} />}
    </div>;
}

// ================================================================= paleta
function Palette({ library, startDrag, endDrag }: { library: CommandLibrary | null; startDrag: (p: DragPayload) => (e: DragEvent) => void; endDrag: () => void }) {
    if (!library) return <p className="prg-note prg-pad">Wczytywanie biblioteki rozkazów…</p>;
    if (!library.commands.length) return <div className="prg-palette"><p className="prg-note">Biblioteka rozkazów jest pusta — logika programu jest definiowana od nowa.
        Rozkazy (bloki) dodaje się w <code>library/commands.json</code> (opis formatu: <code>docs/programming.md</code>).</p></div>;
    return <div className="prg-palette">{library.categories.map(cat => {
        const items = library.commands.filter(c => c.category === cat.id && !c.parents?.length); if (!items.length) return null;
        return <div key={cat.id}><h4 className={`cat-${cat.id}`}>{cat.label.toUpperCase()}</h4>{items.map(c => <div key={c.id} className={`prg-pblk cat-${c.category}`} draggable
            onDragStart={startDrag({ kind: "command", command: c.id })} onDragEnd={endDrag} title={`${c.help ? c.help + "\n" : ""}Przeciągnij na płótno`}>
            <span className="prg-icon">{c.icon}</span>{labelParts(c.label).map((p, k) => "text" in p ? <span key={k}>{p.text}</span> : <span key={k} className="prg-slot-hint">…</span>)}</div>)}</div>;
    })}<p className="prg-note">Biblioteka rozkazów: <code>library/commands.json</code></p></div>;
}

// ================================================================= płótno (schemat blokowy)
// Bloki łączą się strzałkami (łączniki = miejsca upuszczania). „Jeśli” i „wybierz” rozgałęziają przepływ na kolumny
// (TAK / NIE, przypadki / inaczej), które scalają się pod blokiem; pętle mają ramkę z linią powrotu.
type CanvasContext = { program: Program; cmap: Map<string, Command>; library: CommandLibrary | null; selected: string | null; setSelected: (id: string | null) => void;
    dragging: DragPayload | null; startDrag: (p: DragPayload) => (e: DragEvent) => void; endDrag: () => void; dropBlock: (t: DropTarget) => void;
    dropParam: (block: string, param: ParamDef) => void; update: (f: (p: Program) => Program) => void; issues: { block?: string; text: string }[];
    run: RunStatus | null; ancestors: Set<string>; timeline: Map<string, TimelineEntry>; locked: boolean; dragCommand: string | undefined; linked: string | null;
    landing: string | null; land: (id: string) => void };
function Canvas({ ctx }: { ctx: CanvasContext }) {
    const [overTrash, setOverTrash] = useState(false);
    const [zoom, setZoomState] = useState(readZoom);
    const host = useRef<HTMLDivElement>(null), flow = useRef<HTMLDivElement>(null);
    const setZoom = (z: number) => setZoomState(Math.round(Math.min(1.5, Math.max(0.4, z)) * 100) / 100);
    useEffect(() => { try { localStorage.setItem(ZOOM_KEY, String(zoom)); } catch { /* bez pamięci przeglądarki */ } }, [zoom]);
    // Ctrl+kółko (także szczypanie na touchpadzie) nad schematem: powiększenie schematu wokół punktu pod kursorem
    // zamiast powiększania całej strony. Samo kółko przewija schemat.
    const zoomRef = useRef(zoom); zoomRef.current = zoom;
    const anchor = useRef<{ x: number; y: number; clientX: number; clientY: number } | null>(null);
    useEffect(() => {
        const el = host.current; if (!el) return;
        const wheel = (e: WheelEvent) => {
            if (!(e.ctrlKey || e.metaKey)) return;
            e.preventDefault();
            const old = zoomRef.current, next = Math.round(Math.min(1.5, Math.max(0.4, old * Math.exp(-e.deltaY * 0.002))) * 100) / 100;
            if (next === old || !flow.current) return;
            const r = flow.current.getBoundingClientRect();
            anchor.current = { x: (e.clientX - r.left) / old, y: (e.clientY - r.top) / old, clientX: e.clientX, clientY: e.clientY };
            setZoomState(next);
        };
        el.addEventListener("wheel", wheel, { passive: false });
        return () => el.removeEventListener("wheel", wheel);
    }, []);
    const step = (dir: 1 | -1) => setZoom(dir > 0 ? ZOOMS.find(z => z > zoom + 1e-3) ?? zoom : [...ZOOMS].reverse().find(z => z < zoom - 1e-3) ?? zoom);
    // Dopasowanie szerokości schematu do panelu (naturalna szerokość = zmierzona / bieżące powiększenie).
    const fit = () => { if (!host.current || !flow.current) return; const natural = flow.current.getBoundingClientRect().width / zoom;
        setZoom(Math.min(1, (host.current.clientWidth - 48) / Math.max(1, natural))); };
    useLayoutEffect(() => {
        const el = host.current, a = anchor.current; if (!el) return;
        if (a && flow.current) {   // punkt schematu pod kursorem zostaje pod kursorem
            const r = flow.current.getBoundingClientRect();
            el.scrollLeft += r.left + a.x * zoom - a.clientX; el.scrollTop += r.top + a.y * zoom - a.clientY; anchor.current = null;
        } else el.scrollLeft = Math.max(0, (el.scrollWidth - el.clientWidth) / 2);
    }, [ctx.program.id, zoom]);
    const blockDrag = ctx.dragging?.kind === "block" || ctx.dragging?.kind === "command";
    const deselect = (e: { target: EventTarget; currentTarget: EventTarget }) => { if (e.target === e.currentTarget) ctx.setSelected(null); };
    return <div ref={host} className={`prg-canvas${blockDrag ? " dragging" : ""}${ctx.locked ? " locked" : ""}`} onClick={deselect}>
        <div className="prg-zoom" role="group" aria-label="Powiększenie schematu">
            <button title="Pomniejsz" onClick={() => step(-1)}>−</button><button title="100 %" onClick={() => setZoom(1)}>{Math.round(zoom * 100)}%</button>
            <button title="Powiększ" onClick={() => step(1)}>+</button><button title="Dopasuj szerokość schematu do panelu" onClick={fit}>⤢</button></div>
        <div className="prg-varsbar">Zmienne programu: {ctx.program.variables.length ? ctx.program.variables.map(v => <span key={v.name} className="prg-pill var" draggable
            onDragStart={ctx.startDrag({ kind: "variable", name: v.name })} onDragEnd={ctx.endDrag}>{v.name}</span>) : <span className="prg-note">brak — utwórz w zakładce „Zmienne”</span>}</div>
        <div ref={flow} className="prg-flow" style={{ zoom }} onClick={deselect}>
            <div className="prg-term start">▶ Start · {ctx.program.name}</div>
            <FlowList ctx={ctx} blocks={ctx.program.blocks} parent={null} slot="blocks" />
            <div className="prg-term end">■ Koniec</div>
        </div>
        {ctx.dragging?.kind === "block" && <div className={`prg-trash${overTrash ? " over" : ""}`} onDragOver={e => { e.preventDefault(); setOverTrash(true); }} onDragLeave={() => setOverTrash(false)}
            onDrop={e => { e.preventDefault(); setOverTrash(false); const p = ctx.dragging; ctx.endDrag(); if (p?.kind === "block") { ctx.update(pr => removeBlock(pr, p.id).program); ctx.setSelected(null); } }}>🗑 upuść, aby usunąć</div>}
    </div>;
}
// Łącznik ze strzałką; podczas przeciągania — miejsce upuszczenia (tylko tam, gdzie blok jest dozwolony): układ się nie
// zmienia, a nad wybranym miejscem pojawia się zielona linia. Rozsunięcie programu następuje dopiero po upuszczeniu.
function DropZone({ ctx, target, empty, tail }: { ctx: CanvasContext; target: DropTarget; empty?: string; tail?: boolean }) {
    const [over, setOver] = useState(false);
    const active = (ctx.dragging?.kind === "command" || ctx.dragging?.kind === "block") && canDrop(ctx.program, ctx.cmap, ctx.dragCommand, target);
    useEffect(() => { if (!active) setOver(false); }, [active]);
    return <div className={`prg-drop${over && active ? " over" : ""}${active ? " active" : ""}${empty ? " empty" : ""}${tail ? " tail" : ""}`}
        onDragOver={e => { if (!active) return; e.preventDefault(); e.stopPropagation(); setOver(true); }} onDragLeave={() => setOver(false)}
        onDrop={e => { if (!active) return; e.preventDefault(); e.stopPropagation(); setOver(false); ctx.dropBlock(target); }}>
        {empty && <span className="prg-dropbox">{empty}</span>}{over && active && <i className="prg-dropline" aria-hidden />}</div>;
}
function FlowList({ ctx, blocks, parent, slot, fill }: { ctx: CanvasContext; blocks: Block[]; parent: string | null; slot: string; fill?: boolean }) {
    return <div className={`prg-flist${fill ? " fill" : ""}`}>
        <DropZone ctx={ctx} target={{ parent, slot, index: 0 }} empty={!blocks.length && parent === null ? "Przeciągnij tutaj pierwszy blok z palety" : undefined} tail={fill && !blocks.length} />
        {blocks.map((b, i) => <Fragment key={b.id}><div className={`prg-nodewrap${ctx.landing === b.id ? " landing" : ""}`}><div className="prg-landinner"><FlowNode ctx={ctx} block={b} /></div></div>
            <DropZone ctx={ctx} target={{ parent, slot, index: i + 1 }} tail={fill && i === blocks.length - 1} /></Fragment>)}
        {fill && <i className="prg-wire grow" />}
    </div>;
}
function FlowNode({ ctx, block }: { ctx: CanvasContext; block: Block }) {
    const command = ctx.cmap.get(block.command), shape = shapeOf(command), slots = slotsOf(command);
    const scope = ctx.selected === block.id && slots.length > 0;
    const cls = `prg-node shape-${shape} cat-${command?.category ?? "inne"}${block.disabled ? " disabled" : ""}${scope ? " scope" : ""}`;
    const drag = { draggable: !ctx.locked, onDragStart: ctx.startDrag({ kind: "block", id: block.id }), onDragEnd: ctx.endDrag };
    if (shape === "decision" || shape === "switch") {
        const caseSlot = shape === "switch" ? slots.find(s => s.accepts?.length) : undefined;
        const branches = slots.filter(s => s !== caseSlot);
        const cases = caseSlot ? slotList(block, caseSlot.name) : [];
        const addCaseButton = caseSlot && !ctx.locked && <button className="prg-addcase" title="Dodaj przypadek" onClick={e => { e.stopPropagation();
            const { program, id } = addCase(ctx.program, block.id, ctx.cmap); if (id) { ctx.update(() => program); ctx.setSelected(id); ctx.land(id); } }}>＋ przypadek</button>;
        return <div className={`${cls} branch`} {...drag}>
            <BlockHead ctx={ctx} block={block} command={command} extra={addCaseButton} />
            <div className="prg-branches">
                {cases.map((c, k) => <CaseColumn key={c.id} ctx={ctx} parent={block} slot={caseSlot!.name} block={c} index={k} last={k === cases.length - 1} />)}
                {branches.map((s, k) => <div key={s.name} className="prg-bcol"><i className="prg-wire s" />
                    <span className={`prg-blabel${shape === "decision" ? (k === 0 ? " yes" : " no") : ""}`}>{s.label ?? s.name}</span>
                    <FlowList ctx={ctx} blocks={slotList(block, s.name)} parent={block.id} slot={s.name} fill /></div>)}
            </div>
            <div className="prg-merge"><i /></div>
        </div>;
    }
    if (slots.length) return <div className={`${cls} loop`} {...drag}>
        <BlockHead ctx={ctx} block={block} command={command} />
        {slots.map(s => <div key={s.name} className="prg-loop-body"><FlowList ctx={ctx} blocks={slotList(block, s.name)} parent={block.id} slot={s.name} /></div>)}
        <div className="prg-loop-foot" title="Powrót na początek pętli">↺ {block.command === "repeat_until" ? "sprawdź warunek i powtórz" : "powrót"}</div>
    </div>;
    return <div className={cls} {...drag}><BlockHead ctx={ctx} block={block} command={command} /></div>;
}
// Kolumna przypadku „wybierz”: nagłówek = blok przypadku (przeciągany, edytowalny), pod nim jego bloki.
function CaseColumn({ ctx, parent, slot, block, index, last }: { ctx: CanvasContext; parent: Block; slot: string; block: Block; index: number; last: boolean }) {
    const command = ctx.cmap.get(block.command);
    return <div className={`prg-bcol case${block.disabled ? " disabled" : ""}${ctx.landing === block.id ? " landing" : ""}`}>
        <CaseDrop ctx={ctx} target={{ parent: parent.id, slot, index }} side="left" />
        {last && <CaseDrop ctx={ctx} target={{ parent: parent.id, slot, index: index + 1 }} side="right" />}
        <i className="prg-wire s" />
        <div className={`prg-node shape-case cat-${command?.category ?? "inne"}`} draggable={!ctx.locked} onDragStart={ctx.startDrag({ kind: "block", id: block.id })} onDragEnd={ctx.endDrag}>
            <BlockHead ctx={ctx} block={block} command={command} /></div>
        <FlowList ctx={ctx} blocks={slotList(block, "body")} parent={block.id} slot="body" fill />
    </div>;
}
function CaseDrop({ ctx, target, side }: { ctx: CanvasContext; target: DropTarget; side: "left" | "right" }) {
    const [over, setOver] = useState(false);
    const active = (ctx.dragging?.kind === "command" || ctx.dragging?.kind === "block") && canDrop(ctx.program, ctx.cmap, ctx.dragCommand, target);
    useEffect(() => { if (!active) setOver(false); }, [active]);
    if (!active) return null;
    return <div className={`prg-casedrop ${side}${over ? " over" : ""}`} onDragOver={e => { e.preventDefault(); e.stopPropagation(); setOver(true); }} onDragLeave={() => setOver(false)}
        onDrop={e => { e.preventDefault(); e.stopPropagation(); setOver(false); ctx.dropBlock(target); }} />;
}
function BlockHead({ ctx, block, command, extra }: { ctx: CanvasContext; block: Block; command: Command | undefined; extra?: ReactNode }) {
    const flagged = ctx.issues.some(i => i.block === block.id);
    const runCls = cursorClasses(ctx.run, block.id, ctx.ancestors);
    const linked = ctx.linked !== null && ctx.selected !== block.id && ((block.command === "label" && block.params.name === ctx.linked) || (block.command === "goto" && block.params.target === ctx.linked));
    const cls = `prg-blk shape-${shapeOf(command)} cat-${command?.category ?? "inne"}${ctx.selected === block.id ? " selected" : ""}${flagged ? " flagged" : ""}${linked ? " linked" : ""}${runCls ? " " + runCls : ""}`;
    const badge = blendBadge(block, ctx.timeline.get(block.id));
    const blend = ctx.run?.active && ctx.run.robot_cursor?.blend && (ctx.run.robot_cursor.blend.from === block.id || ctx.run.robot_cursor.blend.to === block.id) ? ctx.run.robot_cursor.blend : null;
    const blendStyle = blend ? { ["--blend" as string]: `${Math.round((blend.from === block.id ? 1 - blend.progress : blend.progress) * 100)}%` } : undefined;
    const wait = ctx.timeline.get(block.id);
    return <div className={cls} style={blendStyle} title={command?.help}><div className="prg-bh" onClick={e => { e.stopPropagation(); ctx.setSelected(block.id); }}>
        <span className="prg-icon">{command?.icon ?? "?"}</span>
        {command ? labelParts(command.label).map((p, k) => "text" in p ? (k === 0 ? <CommandSwitch key={k} ctx={ctx} block={block} command={command} text={p.text} /> : <span key={k}>{p.text}</span>) :
            <ParamPill key={k} ctx={ctx} block={block} def={command.params?.find(d => d.name === p.param)} />) : <span>nieznany rozkaz „{block.command}”</span>}
        {extra}
        {badge && <span className={`prg-blend ${badge.tone}`} title={badge.title}>{badge.text}</span>}
        {wait?.kind === "wait" && <span className="prg-blend" title="Czas czekania na osi czasu sprawdzenia">⏱ {(wait.end_s - wait.start_s).toFixed(1)} s</span>}
        {flagged && <span className="prg-flag" title={ctx.issues.filter(i => i.block === block.id).map(i => i.text).join("\n")}>!</span>}
        {runCls.includes("run-robot") || blend ? <span className="prg-cursor robot" title="Kursor robota">◆ robot{ctx.run?.robot_cursor ? ` · ${phaseLabels[ctx.run.robot_cursor.phase] ?? ""}` : ""}</span> : null}
        {runCls.includes("run-program") && <span className="prg-cursor program" title="Kursor programu (interpreter liczy z wyprzedzeniem)">▷ program</span>}
    </div></div>;
}
// Nazwa rozkazu w bloku: kliknięcie → lista rozkazów zgodnych (np. PTP ↔ LIN) — zamiana bez usuwania bloku.
function CommandSwitch({ ctx, block, command, text }: { ctx: CanvasContext; block: Block; command: Command; text: string }) {
    const [open, setOpen] = useState(false), [note, setNote] = useState("");
    const ref = useRef<HTMLSpanElement>(null);
    useEffect(() => { if (!open) return; const close = (e: MouseEvent) => { if (!ref.current?.contains(e.target as Node)) setOpen(false); };
        document.addEventListener("mousedown", close); return () => document.removeEventListener("mousedown", close); }, [open]);
    const options = compatibleCommands(ctx.library, command);
    const [, word = "", rest = ""] = /^(\s*\S+)(.*)$/s.exec(text) ?? [];
    if (!options.length || ctx.locked) return <span>{text}</span>;
    const choose = (target: Command) => {
        const { program: next, reset } = changeCommand(ctx.program, block.id, target);
        ctx.update(() => next);
        setNote(reset.length ? `Przywrócono domyślne: ${reset.join(", ")} (poza zakresem ${commandName(target)})` : ""); setOpen(false);
    };
    return <><span ref={ref} className="prg-cmd" title="Kliknij, aby zmienić rodzaj rozkazu (parametry o tej samej nazwie zostają)" onClick={e => { e.stopPropagation(); ctx.setSelected(block.id); setOpen(o => !o); setNote(""); }}>
        {word}<span className="prg-cmd-caret">▾</span>
        {open && <span className="prg-editor prg-cmd-menu" onClick={e => e.stopPropagation()}>
            <b>Zmień rozkaz</b>
            {[command, ...options].map(c => <button key={c.id} className={c.id === command.id ? "on" : ""} disabled={c.id === command.id} onClick={() => choose(c)}>
                <span className="prg-icon">{c.icon}</span>{c.label.replace(/\{(\w+)\}/g, (_, name: string) => c.params?.find(d => d.name === name)?.unit ? `[${c.params.find(d => d.name === name)!.unit}]` : "…")}</button>)}
            <span className="prg-note">Parametry o tej samej nazwie zostają; wartość spoza zakresu nowego rozkazu wraca do domyślnej.</span>
        </span>}
    </span><span>{rest}</span>{note && <span className="prg-blend warn" title={note}>⟲ {note}</span>}</>;
}
function ParamPill({ ctx, block, def }: { ctx: CanvasContext; block: Block; def?: ParamDef }) {
    const [open, setOpen] = useState(false), [over, setOver] = useState(false);
    const ref = useRef<HTMLSpanElement>(null);
    useEffect(() => { if (!open) return; const close = (e: MouseEvent) => { if (!ref.current?.contains(e.target as Node)) setOpen(false); };
        document.addEventListener("mousedown", close); return () => document.removeEventListener("mousedown", close); }, [open]);
    if (!def) return <span className="prg-pill">?</span>;
    const value = block.params[def.name];
    const ok = accepts(def, ctx.dragging, ctx.program.variables);
    const kind = isRef(value) ? ("var" in value ? "var" : "pos") : value === null || value === "" ? "missing" : "";
    const set = (v: Value) => ctx.update(p => setParam(p, block.id, def.name, v));
    return <span ref={ref} className={`prg-pill ${kind}${ok ? " accept" : ""}${over ? " over" : ""}`} title={`${def.label}${def.unit ? " [" + def.unit + "]" : ""} — kliknij, aby zmienić`}
        onClick={e => { e.stopPropagation(); setOpen(o => !o); }}
        onDragOver={e => { if (!ok) return; e.preventDefault(); e.stopPropagation(); setOver(true); }} onDragLeave={() => setOver(false)}
        onDrop={e => { if (!ok) return; e.preventDefault(); e.stopPropagation(); setOver(false); ctx.dropParam(block.id, def); }}>
        {kind === "pos" ? "📍 " : ""}{value === null || value === "" ? def.label : formatValue(value, def)}
        {open && <ParamEditor ctx={ctx} def={def} value={value} set={set} close={() => setOpen(false)} />}
    </span>;
}
function ParamEditor({ ctx, def, value, set, close }: { ctx: CanvasContext; def: ParamDef; value: Value; set: (v: Value) => void; close: () => void }) {
    const vars = ctx.program.variables.filter(v => accepts(def, { kind: "variable", name: v.name }, ctx.program.variables));
    const literal = isRef(value) ? "" : value === null ? "" : String(value);
    const io = def.kind === "output" ? ctx.library?.io?.outputs ?? [] : def.kind === "input" ? ctx.library?.io?.inputs ?? [] : [];
    const varSelect = vars.length > 0 && <label>zmienna<select value={isRef(value) && "var" in value ? value.var : ""} onChange={e => e.target.value && set({ var: e.target.value })}>
        <option value="">—</option>{vars.map(v => <option key={v.name} value={v.name}>{v.name} · {v.type}</option>)}</select></label>;
    return <span className="prg-editor" onClick={e => e.stopPropagation()} onKeyDown={e => { if (e.key === "Enter" || e.key === "Escape") close(); }}>
        <b>{def.label}{def.unit ? ` [${def.unit}]` : ""}</b>
        {def.kind === "number" && <label>wartość<NumberInput value={value} integer={def.integer} onValue={set} /></label>}
        {(def.kind === "condition" || def.kind === "text") && <label>{def.kind === "text" ? "tekst" : "wyrażenie"}<input autoFocus value={literal} onChange={e => set(e.target.value)} /></label>}
        {def.kind === "expression" && <label>wyrażenie<ZeroWhenEmptyInput text={literal} onText={t => set(t === "" ? "0" : t)} /></label>}
        {(def.kind === "condition" || def.kind === "expression") && <ExpressionHint text={literal} names={ctx.program.variables.map(v => v.name)} />}
        {def.kind === "label" && <label>nazwa etykiety<input autoFocus list={`labels-${def.name}`} value={literal} onChange={e => set(e.target.value)} />
            <datalist id={`labels-${def.name}`}>{labelNames(ctx.program).map(n => <option key={n} value={n} />)}</datalist></label>}
        {def.kind === "choice" && <span className="prg-choices">{(def.options ?? []).map(o => <button key={o} className={value === o ? "on" : ""} onClick={() => set(o)}>{o}</button>)}</span>}
        {def.kind === "value" && <label>wartość<ZeroWhenEmptyInput text={literal} onText={raw => { const n = Number(raw.replace(",", "."));
            set(raw.trim() === "" ? 0 : Number.isFinite(n) ? n : raw === "TRUE" ? true : raw === "FALSE" ? false : raw); }} /></label>}
        {def.kind === "bool" && <span className="prg-seg">{[true, false].map(b => <button key={String(b)} className={value === b ? "on" : ""} onClick={() => set(b)}>{b ? "TRUE" : "FALSE"}</button>)}</span>}
        {(def.kind === "output" || def.kind === "input") && <label>{def.kind === "output" ? "wyjście" : "wejście"}<select value={literal} onChange={e => set(e.target.value)}>
            {io.map(n => <option key={n}>{n}</option>)}</select></label>}
        {(def.kind === "position" || def.kind === "value" || def.kind === "expression") && <label>pozycja<select value={isRef(value) && "position" in value ? value.position : ""} onChange={e => e.target.value && set({ position: e.target.value })}>
            <option value="">—</option>{ctx.program.positions.map(p => <option key={p.name}>{p.name}</option>)}</select></label>}
        {varSelect}
        {def.kind === "variable" && !vars.length && <span className="prg-note">Brak pasujących zmiennych{def.types ? ` (${def.types.join("/")})` : ""} — utwórz w zakładce „Zmienne”.</span>}
        {def.kind !== "variable" && def.default !== undefined && <button className="secondary-button sm" onClick={() => set(structuredClone(def.default) as Value)}>domyślna</button>}
        {!["choice", "label", "variable"].includes(def.kind) && <span className="prg-note">Można też przeciągnąć {def.kind === "position" ? "pozycję lub zmienną POZYCJA" : "zmienną"} na to pole.</span>}
    </span>;
}
// Pole liczby w okienku bloku: własny tekst roboczy (można skasować, wpisać „0”, „-”, „0,5”); pusty → 0 (zaznaczone,
// więc kolejna cyfra je zastępuje); poprawna liczba trafia od razu do bloku (INT — zaokrąglenie).
const NUMBER_RE = /^[-+]?(\d+\.?\d*|\.\d+)(e[-+]?\d+)?$/i;
function NumberInput({ value, integer, onValue }: { value: Value; integer?: boolean; onValue: (v: number) => void }) {
    const shown = typeof value === "number" ? String(value) : "";
    const [draft, setDraft] = useState(shown);
    const ref = useRef<HTMLInputElement>(null);
    useEffect(() => { if (Number(draft.replace(",", ".")) !== value) setDraft(shown); }, [value]);   // eslint-disable-line react-hooks/exhaustive-deps
    const change = (raw: string) => {
        const text = raw.replace(",", ".").trim();
        if (text === "") { setDraft("0"); onValue(0); requestAnimationFrame(() => ref.current?.select()); return; }
        setDraft(raw);
        if (NUMBER_RE.test(text)) { const v = Number(text); if (Number.isFinite(v)) onValue(integer ? Math.round(v) : v); }
    };
    return <input ref={ref} type="text" inputMode="decimal" autoFocus value={draft} aria-invalid={draft.trim() !== "" && !NUMBER_RE.test(draft.replace(",", ".").trim())}
        onFocus={e => e.target.select()} onChange={e => change(e.target.value)} onBlur={() => setDraft(shown)} />;
}
// Pole tekstowe, które po całkowitym skasowaniu przyjmuje „0” (zaznaczone).
function ZeroWhenEmptyInput({ text, onText }: { text: string; onText: (t: string) => void }) {
    const ref = useRef<HTMLInputElement>(null);
    return <input ref={ref} autoFocus value={text} onFocus={e => e.target.select()}
        onChange={e => { const raw = e.target.value; onText(raw); if (raw.trim() === "") requestAnimationFrame(() => ref.current?.select()); }} />;
}
// Podpowiedź składni i błąd wyrażenia na żywo (ta sama gramatyka co interpreter w moście).
function ExpressionHint({ text, names }: { text: string; names: string[] }) {
    const error = text.trim() ? checkExpression(text, new Set(names)) : null;
    return <span className="prg-note">{error ? <span className="prg-error">⚠ {error}</span> : text.trim() ? "✓ wyrażenie poprawne" : null}
        <br />Operatory: + − * / MOD ^ · = &lt;&gt; &lt; &lt;= &gt; &gt;= · AND OR XOR NOT · TRUE FALSE PI · 'tekst'
        <br />Funkcje: {Object.keys(EXPRESSION_FUNCTIONS).join(", ")} (kąty w °)</span>;
}
function TextView({ lines, selected, onSelect }: { lines: ReturnType<typeof programText>; selected: string | null; onSelect: (id: string) => void }) {
    return <div className="prg-text">{lines.map((l, k) => <div key={l.key} className={`prg-cl${l.kind ? " " + l.kind : ""}${l.block && l.block === selected ? " selected" : ""}`}
        onClick={() => l.block && onSelect(l.block)}><span className="n">{k + 1}</span><span className="code">{"    ".repeat(l.depth)}{l.text}</span></div>)}</div>;
}

// ================================================================= zmienne
type PanelProps = { program: Program; library: LibrarySnapshot | null; update: (f: (p: Program) => Program) => void; setLibrary: (l: LibrarySnapshot) => void;
    notify: (t: string, tone?: "ok" | "warn" | "error") => void; startDrag: (p: DragPayload) => (e: DragEvent) => void; endDrag: () => void };
function LibrarySection({ title, children, count }: { title: string; children: ReactNode; count: number }) {
    const [open, setOpen] = useState(false);
    return <div className="prg-lib"><button className="prg-lib-head" onClick={() => setOpen(o => !o)}>{open ? "▾" : "▸"} {title} ({count})</button>{open && <div>{children}</div>}</div>;
}
async function storeLibrary<K extends "variables" | "positions">(props: PanelProps, name: K, items: K extends "variables" ? Variable[] : Position[]) {
    if (!props.library) return;
    try { const r = await programApi.setLibrary(name, items, props.library[name].revision);
        props.setLibrary({ ...props.library, [name]: { items: r.items, revision: r.revision } }); props.notify("Zapisano bibliotekę"); }
    catch (error) { props.notify((error as Error).message, "error"); }
}
function VariablesPanel(props: PanelProps & { onCreate: () => void; onEdit: (v: Variable) => void }) {
    const { program, update } = props, lib = props.library?.variables.items ?? [];
    return <div className="prg-page">
        <button className="primary-button prg-wide" onClick={props.onCreate}>＋ Utwórz zmienną</button>
        {program.variables.map(v => <div key={v.name} className="prg-vcard" draggable onDragStart={props.startDrag({ kind: "variable", name: v.name })} onDragEnd={props.endDrag}>
            <div className="prg-row"><b>ⓥ {v.name}</b><span className="prg-type">{v.type}</span><span className="spacer" />
                <button className="secondary-button sm" title="Edytuj" onClick={() => props.onEdit(v)}>✎</button>
                <button className="secondary-button sm" title="Zapisz w bibliotece" onClick={() => void storeLibrary(props, "variables", [...lib.filter(x => x.name !== v.name), v])}>⇪</button>
                <button className="secondary-button sm" title="Usuń" onClick={() => { const n = usage(program, "var", v.name);
                    if (n && !window.confirm(`Zmienna ${v.name} jest użyta w ${n} miejscach. Usunąć? Odwołania pozostaną do poprawy.`)) return;
                    update(p => ({ ...p, variables: p.variables.filter(x => x.name !== v.name) })); }}>×</button></div>
            <div className="prg-init">= {initText(v)}</div>{v.note && <div className="prg-note">{v.note}</div>}
            <div className="prg-note">użyta: {usage(program, "var", v.name)}×</div></div>)}
        {!program.variables.length && <p className="prg-note">Zmienne deklaruje się przed kodem. Po utworzeniu przeciągnij zmienną na pole bloku (prędkość, pozycja, warunek…).</p>}
        <LibrarySection title="Biblioteka zmiennych" count={lib.length}>{lib.map(v => <div key={v.name} className="prg-libitem"><span>ⓥ {v.name} <span className="prg-type">{v.type}</span></span>
            <button className="secondary-button sm" title="Dodaj do programu" onClick={() => update(p => ({ ...p, variables: [...p.variables, { ...v, name: p.variables.some(x => x.name === v.name) ? uniqueName(v.name, p.variables.map(x => x.name)) : v.name }] }))}>＋</button>
            <button className="secondary-button sm" title="Usuń z biblioteki" onClick={() => void storeLibrary(props, "variables", lib.filter(x => x.name !== v.name))}>×</button></div>)}
            {!lib.length && <p className="prg-note">Pusta — użyj ⇪ przy zmiennej, aby ją tu zapisać (<code>library/variables.json</code>).</p>}</LibrarySection>
    </div>;
}
function VariableModal({ program, edit, onClose, onSave }: { program: Program; edit: Variable | null; onClose: () => void; onSave: (v: Variable, previous: Variable | null) => void }) {
    const [type, setType] = useState<VarType>(edit?.type ?? "INT"), [name, setName] = useState(edit?.name ?? uniqueName("zmienna", program.variables.map(v => v.name)));
    const [init, setInit] = useState(edit ? (typeof edit.init === "boolean" ? String(edit.init) : String(edit.init)) : "0");
    const [unit, setUnit] = useState(edit?.unit ?? ""), [note, setNote] = useState(edit?.note ?? "");
    const [min, setMin] = useState(edit?.min != null ? String(edit.min) : ""), [max, setMax] = useState(edit?.max != null ? String(edit.max) : "");
    useEffect(() => { if (!edit) setInit(String(defaultInit(type))); }, [type, edit]);
    const parsed = parseInit(type, init), err = nameError(name, program.variables.map(v => v.name), edit?.name) ?? (parsed === null ? `Nieprawidłowa wartość dla typu ${type}` : null);
    const numeric = type === "INT" || type === "REAL";
    const submit = () => { if (err || parsed === null) return; const v: Variable = { name, type, init: parsed };
        if (numeric && unit) v.unit = unit; if (note) v.note = note; if (numeric && min !== "") v.min = Number(min); if (numeric && max !== "") v.max = Number(max); onSave(v, edit); };
    return <div className="prg-modal-bg" onMouseDown={e => { if (e.target === e.currentTarget) onClose(); }}><div className="prg-modal" role="dialog" aria-label="Zmienna programu">
        <h3>{edit ? `Zmienna ${edit.name}` : "Nowa zmienna programu"}</h3>
        <div className="prg-note">Zmienne deklaruje się przed kodem; w blokach pojawiają się jako ⓥ.</div>
        <div className="prg-types">{VAR_TYPES.map(t => <button key={t} className={t === type ? "on" : ""} onClick={() => setType(t)}><b>{t}</b>{TYPE_HINT[t]}</button>)}</div>
        <div className="prg-form">
            <label>Nazwa</label><input autoFocus value={name} onChange={e => setName(e.target.value)} />
            <label>Wartość początkowa</label>{type === "BOOL" ? <select value={init} onChange={e => setInit(e.target.value)}><option value="false">FALSE</option><option value="true">TRUE</option></select>
                : type === "POZYCJA" ? <select value={init} onChange={e => setInit(e.target.value)}><option value="">— brak —</option>{program.positions.map(p => <option key={p.name}>{p.name}</option>)}</select>
                : <input value={init} onChange={e => setInit(e.target.value)} />}
            {numeric && <><label>Jednostka</label><select value={unit} onChange={e => setUnit(e.target.value)}>{["", "mm", "mm/s", "°", "°/s", "s", "%"].map(u => <option key={u} value={u}>{u || "—"}</option>)}</select>
                <label>Zakres (opcjonalnie)</label><span><input placeholder="min" size={6} value={min} onChange={e => setMin(e.target.value)} /> … <input placeholder="maks" size={6} value={max} onChange={e => setMax(e.target.value)} /></span></>}
            <label>Opis</label><input value={note} maxLength={200} onChange={e => setNote(e.target.value)} />
        </div>
        {err && <p className="prg-error">{err}</p>}
        <div className="prg-modal-actions"><button className="secondary-button" onClick={onClose}>Anuluj</button><button className="primary-button" disabled={!!err} onClick={submit}>{edit ? "Zapisz zmiany" : "Utwórz zmienną"}</button></div>
    </div></div>;
}

// ================================================================= pozycje
function PositionsPanel(props: PanelProps & { selected: string | null; setSelected: (n: string | null) => void; ranges: JointRange[] }) {
    const { program, update, notify } = props, lib = props.library?.positions.items ?? [];
    const [editing, setEditing] = useState<{ original: string; name: string; joints: string[]; note: string } | null>(null);
    const [busy, setBusy] = useState(false);
    async function teach(target?: string) {
        setBusy(true);
        try { const joints = await currentJoints(), stamp = new Date().toISOString();
            if (target) { update(p => ({ ...p, positions: p.positions.map(x => x.name === target ? { ...x, joints_deg: joints, source: "pomiar", created_at: stamp } : x) })); notify(`Zaktualizowano ${target} z pomiaru robota`); }
            else { const name = uniqueName("Pozycja", program.positions.map(p => p.name)); update(p => ({ ...p, positions: [...p.positions, { name, joints_deg: joints, source: "pomiar", created_at: stamp }] }));
                props.setSelected(name); notify(`Zapamiętano ${name} z pomiaru robota`); } }
        catch (error) { notify(`Nie można odczytać pozycji robota: ${(error as Error).message}`, "error"); }
        finally { setBusy(false); }
    }
    const commit = () => { if (!editing) return; const joints = editing.joints.map(v => Number(v.replace(",", ".")));
        const err = nameError(editing.name, program.positions.map(p => p.name), editing.original) ?? (joints.some(v => !Number.isFinite(v)) ? "Kąty muszą być liczbami" : null);
        if (err) { notify(err, "error"); return; }
        update(p => { const next = editing.original !== editing.name ? renameRefs(p, "position", editing.original, editing.name) : p;
            return { ...next, positions: next.positions.map(x => x.name === editing.original ? { ...x, name: editing.name, joints_deg: joints, note: editing.note || undefined } : x),
                variables: next.variables.map(v => v.type === "POZYCJA" && v.init === editing.original ? { ...v, init: editing.name } : v) }; });
        props.setSelected(editing.name); setEditing(null); };
    return <div className="prg-page">
        <button className="primary-button prg-wide" disabled={busy} onClick={() => void teach()}>📍 Zapamiętaj bieżącą pozycję robota</button>
        <button className="secondary-button prg-wide" onClick={() => { const name = uniqueName("Pozycja", program.positions.map(p => p.name));
            update(p => ({ ...p, positions: [...p.positions, { name, joints_deg: [0, 0, 0, 0, 0, 0], source: "ręcznie" }] })); setEditing({ original: name, name, joints: Array(6).fill("0"), note: "" }); props.setSelected(name); }}>＋ Dodaj ręcznie (J1–J6)</button>
        {program.positions.map(p => editing?.original === p.name ? <div key={p.name} className="prg-vcard editing">
            <input value={editing.name} onChange={e => setEditing({ ...editing, name: e.target.value })} aria-label="Nazwa pozycji" />
            <div className="prg-joints">{editing.joints.map((v, i) => <label key={i}>J{i + 1}<input value={v} onChange={e => { const joints = [...editing.joints]; joints[i] = e.target.value; setEditing({ ...editing, joints }); }} /></label>)}</div>
            <input placeholder="opis" value={editing.note} onChange={e => setEditing({ ...editing, note: e.target.value })} />
            <div className="prg-row"><span className="spacer" /><button className="secondary-button sm" onClick={() => setEditing(null)}>Anuluj</button><button className="primary-button sm" onClick={commit}>Zapisz</button></div>
        </div> : <div key={p.name} className={`prg-vcard pos${props.selected === p.name ? " selected" : ""}`} draggable onDragStart={props.startDrag({ kind: "position", name: p.name })} onDragEnd={props.endDrag}
            onClick={() => props.setSelected(p.name)}>
            <div className="prg-row"><b>📍 {p.name}</b><span className="spacer" />
                <button className="secondary-button sm" title="Edytuj nazwę i kąty" onClick={e => { e.stopPropagation(); setEditing({ original: p.name, name: p.name, joints: p.joints_deg.map(String), note: p.note ?? "" }); }}>✎</button>
                <button className="secondary-button sm" title="Zastąp bieżącą pozycją robota" disabled={busy} onClick={e => { e.stopPropagation(); void teach(p.name); }}>⟲</button>
                <button className="secondary-button sm" title="Zapisz w bibliotece" onClick={e => { e.stopPropagation(); void storeLibrary(props, "positions", [...lib.filter(x => x.name !== p.name), p]); }}>⇪</button>
                <button className="secondary-button sm" title="Usuń" onClick={e => { e.stopPropagation(); const n = usage(program, "position", p.name);
                    if (n && !window.confirm(`Pozycja ${p.name} jest użyta w ${n} blokach. Usunąć?`)) return; update(pr => ({ ...pr, positions: pr.positions.filter(x => x.name !== p.name) })); }}>×</button></div>
            <div className="prg-init">J {p.joints_deg.map((v, i) => { const out = violations([v], [props.ranges[i]]).length > 0;
                return <Fragment key={i}>{i > 0 && " / "}<span className={out ? "prg-out" : undefined} title={out ? `J${i + 1} poza zakresem ${props.ranges[i]![0]}…${props.ranges[i]![1]}°` : undefined}>{+v.toFixed(1)}</span></Fragment>; })}°</div>
            {violations(p.joints_deg, props.ranges).length > 0 && <div className="prg-out-note">⚠ poza zakresem osi: {describeViolations(violations(p.joints_deg, props.ranges))}</div>}
            <div className="prg-note">{p.source ?? ""}{p.note ? ` · ${p.note}` : ""} · użyta: {usage(program, "position", p.name)}×</div></div>)}
        {!program.positions.length && <p className="prg-note">Zapamiętaj pozycję z robota (wymaga skalibrowanych osi) albo dodaj ręcznie. Przeciągnij pozycję na pole „pozycja” bloku ruchu.</p>}
        <LibrarySection title="Biblioteka pozycji" count={lib.length}>{lib.map(p => <div key={p.name} className="prg-libitem"><span>📍 {p.name}</span>
            <button className="secondary-button sm" title="Dodaj do programu" onClick={() => update(pr => ({ ...pr, positions: [...pr.positions, { ...p, name: pr.positions.some(x => x.name === p.name) ? uniqueName(p.name, pr.positions.map(x => x.name)) : p.name }] }))}>＋</button>
            <button className="secondary-button sm" title="Usuń z biblioteki" onClick={() => void storeLibrary(props, "positions", lib.filter(x => x.name !== p.name))}>×</button></div>)}
            {!lib.length && <p className="prg-note">Pusta — użyj ⇪ przy pozycji, aby ją tu zapisać (<code>library/positions.json</code>).</p>}</LibrarySection>
    </div>;
}

// ================================================================= podgląd 3D i zmiennych
// ================================================================= wykonanie
function RunPanel({ program, run, global, approved, dirty, issues, onCheck, onExecute, onStop, onResume, onBlock, launch, onCloseLaunch, onPosition }: { program: Program; run: RunStatus | null; global: RunStatus | null;
    approved: boolean; dirty: boolean; issues: number; onCheck: () => void; onExecute: () => void; onStop: () => void; onResume: () => void; onBlock: (id: string) => void;
    launch: { steps: LaunchStep[]; active: boolean } | null; onCloseLaunch: () => void; onPosition: (name: string) => void }) {
    const other = global?.active && !run ? global : null;
    const active = !!run?.active, busy = active || !!other;
    const timeline = run && run.revision === program.revision ? run.timeline : [];
    const total = Math.max(run?.total_s ?? 0, ...timeline.map(e => e.transition_end_s), 0.001);
    const bars = timelineBars(timeline);
    const chainStart = run?.chain ? Math.min(...timeline.filter(e => e.chain === run.chain!.index).map(e => e.start_s), Infinity) : Infinity;
    const marker = active && run?.mode === "execute" && run.chain && Number.isFinite(chainStart) ? Math.min(total, chainStart + run.chain.elapsed_s) : null;
    const checkTitle = dirty ? "Najpierw zapisz program" : issues ? "Najpierw usuń uwagi" : "Symulacja programu od bieżącej pozycji robota — bez ruchu";
    const runTitle = "Doprowadza program i robota do stanu startu: zapis, analiza (uwagi, zakresy osi, stan robota), symulacja, potwierdzenie, "
        + `${approved ? "" : "zatwierdzenie rewizji, "}załączenie napędów i start (ruch fizyczny)`;
    const tone = run?.state === "failed" ? "error" : run?.state === "succeeded" ? "ok" : run?.state === "stopped" ? "warn" : "";
    return <div className="prg-panel prg-run">
        <div className="prg-panel-head">Wykonanie <small>{run ? `${run.mode === "execute" ? "robot" : "sprawdzenie"} · ${runStateLabels[run.state] ?? run.state}` : "kursory: ◆ robot · ▷ program"}</small><span className="spacer" />
            {run?.state && run.state !== "idle" && <span className={`badge ${tone}`}><span className="dot" />{runStateLabels[run.state] ?? run.state}</span>}</div>
        <div className="prg-run-actions">
            <button className="secondary-button" disabled={busy || dirty || issues > 0 || !program.id || !program.blocks.length} title={checkTitle} onClick={onCheck}>⟲ Sprawdź</button>
            <button className="primary-button" disabled={busy || !!launch?.active || !program.id || !program.blocks.length} title={runTitle} onClick={onExecute}>{launch?.active ? "◌ Przygotowanie…" : "▶ Uruchom"}</button>
            <button className="motion-stop" disabled={!busy} onClick={onStop} title="Zatrzymaj przebieg i ruch robota">■ STOP</button>
        </div>
        {launch && <div className="prg-launch" aria-label="Przygotowanie do startu">
            <div className="prg-launch-head">Przygotowanie do startu<span className="spacer" />{!launch.active && <button className="link" onClick={onCloseLaunch} title="Ukryj">×</button>}</div>
            {launch.steps.map(s => <div key={s.id} data-step={s.id} className={`prg-launch-step ${s.state}`}>
                <span className="prg-launch-icon">{LAUNCH_ICON[s.state]}</span><b>{s.label}</b>{s.detail && <span className="prg-note">{s.detail}</span>}
                {s.items?.map(item => <button key={item.name} className={`prg-launch-item${item.used ? " used" : ""}`} onClick={() => { onPosition(item.name); if (item.block_id) onBlock(item.block_id); }}
                    title="Pokaż pozycję w podglądzie 3D">📍 {item.name}{item.used ? "" : " (nieużywana)"}: {item.violations.map(v => `J${v.axis} ${+v.value.toFixed(2)}° ∉ [${v.lower}, ${v.upper}]`).join(", ")}</button>)}
            </div>)}
        </div>}
        {other && <p className="prg-note prg-pad">Trwa przebieg programu „{other.name ?? other.program_id}” ({runStateLabels[other.state] ?? other.state}).</p>}
        {run?.active && run.state === "waiting_operator" && <div className="prg-run-pause">
            <span>✋ {run.wait?.text || "Czekanie na operatora"}</span>
            <button className="primary-button" onClick={onResume} title="Robot wykona dalszą część programu">⏵ Kontynuuj</button></div>}
        {run?.active && run.state === "waiting" && run.wait && <div className="prg-run-pause wait"><span>⏱ {run.wait.text}</span>
            {run.wait.until_monotonic_s !== undefined && run.server_monotonic_s !== undefined && <span className="prg-note">pozostało ~{Math.max(0, run.wait.until_monotonic_s - run.server_monotonic_s).toFixed(1)} s</span>}</div>}
        {run && <div className="prg-run-body">
            {run.mode === "execute" && active && <div className="prg-run-cursors">
                <span className="prg-cursor robot">◆ robot</span><button className="link" disabled={!run.robot_cursor} onClick={() => run.robot_cursor && onBlock(run.robot_cursor.block_id)}>
                    {run.robot_cursor ? `${run.robot_cursor.block_id} · ${phaseLabels[run.robot_cursor.phase] ?? run.robot_cursor.phase}${run.robot_cursor.blend ? ` → ${run.robot_cursor.blend.to}` : ""}` : "—"}</button>
                <span className="prg-cursor program">▷ program</span><button className="link" disabled={!run.program_cursor.block_id} onClick={() => run.program_cursor.block_id && onBlock(run.program_cursor.block_id)}>
                    {run.program_cursor.block_id ?? "—"} · {({ computing: "liczy", waiting_robot: "czeka na robota", waiting_sync: "czeka", done: "koniec" } as Record<string, string>)[run.program_cursor.state]}</button>
                {run.chain && <span className="prg-note">ciąg {run.chain.index + 1}{run.chains ? `/${run.chains}` : ""} · {run.chain.elapsed_s.toFixed(1)} / {run.chain.duration_s.toFixed(1)} s{run.next_chain?.state === "planning" ? " · następny: liczenie" : ""}</span>}
            </div>}
            {timeline.length > 0 && <div className="prg-timeline" aria-label="Oś czasu programu">
                {bars.map((bar, k) => <div key={k} className={`prg-tl ${bar.kind}`} style={{ left: `${bar.start / total * 100}%`, width: `${Math.max(0.3, (bar.end - bar.start) / total * 100)}%` }}
                    title={`${bar.block ?? "przerwa między ciągami"}${bar.kind === "wait" ? " · czekanie" : bar.kind === "operator" ? " · czeka na operatora" : ""} · ${bar.start.toFixed(2)}–${bar.end.toFixed(2)} s`} onClick={() => bar.block && onBlock(bar.block)} />)}
                {marker !== null && <div className="prg-tl-marker" style={{ left: `${marker / total * 100}%` }} />}
            </div>}
            {timeline.length > 0 && <div className="prg-note">{timeline.filter(e => !e.kind).length} ruchów · {run.chains} ciąg(i) · ~{(run.total_s ?? total).toFixed(1)} s
                <span className="prg-legend"><i className="move" />ruch <i className="blend" />przejście <i className="stop" />postój <i className="wait" />czekanie <i className="gap" />przerwa</span></div>}
            {run.message && <div className={`prg-run-msg ${tone}`}>{run.message}</div>}
            {run.warnings.length > 0 && <div className="prg-issues">{run.warnings.slice(0, 20).map((w, k) =>
                <button key={k} onClick={() => w.block_id && onBlock(w.block_id)}>⚠ {w.block_id ? `${w.block_id}: ` : ""}{w.text}</button>)}</div>}
        </div>}
    </div>;
}
function Preview3D({ program, selected, values, ranges }: { program: Program; selected: string | null; values: Record<string, unknown> | null; ranges: JointRange[] }) {
    const host = useRef<HTMLDivElement>(null), scene = useRef<ReturnType<typeof import("./programPreviewScene").createProgramScene> | null>(null);
    const [error, setError] = useState(""), [ready, setReady] = useState(false);
    const pose = program.positions.find(p => p.name === selected)?.joints_deg ?? null;
    const outside = useMemo(() => new Set(program.positions.filter(p => violations(p.joints_deg, ranges).length).map(p => p.name)), [program.positions, ranges]);
    const bad = pose ? violations(pose, ranges) : [];
    useEffect(() => { let closed = false;
        void import("./programPreviewScene").then(engine => {
            if (closed || !host.current) return;
            try { scene.current = engine.createProgramScene(host.current); setReady(true); }
            catch { setError("Podgląd 3D niedostępny — sprawdź obsługę WebGL"); }
        }).catch(() => !closed && setError("Nie udało się wczytać podglądu 3D"));
        return () => { closed = true; scene.current?.dispose(); scene.current = null; }; }, []);
    useEffect(() => { if (ready) scene.current?.setGeometry(values); }, [ready, values]);
    useEffect(() => { scene.current?.setPositions(program.positions, selected, outside); scene.current?.setPose(pose); scene.current?.setLimits(poseLimits(ranges, pose)); });
    return <div className="prg-panel"><div className="prg-panel-head">Podgląd 3D <small>{selected ? `pozycja ${selected}` : "poza zerowa"}</small>
        {bad.length > 0 && <span className="badge error" title={describeViolations(bad)}><span className="dot" />poza zakresem: {bad.map(v => `J${v.axis}`).join(", ")}</span>}</div>
        <div className="prg-viewport" ref={host}>{error && <p className="prg-overlay">{error}</p>}</div>
        <div className="prg-note prg-pad">Wybierz pozycję w zakładce „Pozycje”, aby ustawić model. Przebieg programu: panel „Wykonanie” (oś czasu, kursory na blokach).</div></div>;
}
function WatchPanel({ program, issues, directory, onIssue }: { program: Program; issues: { block?: string; text: string }[]; directory?: string; onIssue: (id?: string) => void }) {
    return <div className="prg-panel prg-watch"><div className="prg-panel-head">Podgląd zmiennych <small>wartości bieżące pojawią się podczas symulacji</small></div>
        <table className="prg-table"><thead><tr><th>Zmienna</th><th>Typ</th><th>Początkowa</th><th>Bieżąca</th></tr></thead><tbody>
            {program.variables.map(v => <tr key={v.name} title={v.note}><td>ⓥ {v.name}</td><td className="prg-type">{v.type}</td><td>{initText(v)}</td><td className="prg-note">—</td></tr>)}
            {!program.variables.length && <tr><td colSpan={4} className="prg-note">Brak zmiennych programu</td></tr>}</tbody></table>
        <div className="prg-panel-head">Sprawdzenie <small>{issues.length ? `${issues.length} uwag` : "bez uwag"}</small></div>
        <div className="prg-issues">{issues.length ? issues.slice(0, 30).map((i, k) => <button key={k} onClick={() => onIssue(i.block)}>⚠ {i.text}</button>) : <span className="prg-note">✓ Wszystkie pola uzupełnione, odwołania poprawne</span>}</div>
        <div className="prg-note prg-pad">Plik: <code>{directory ? `${directory}/programs/${program.id}.json` : "—"}</code></div></div>;
}
function NameDialog({ title, initial, onCancel, onOk }: { title: string; initial: string; onCancel: () => void; onOk: (name: string) => void }) {
    const [name, setName] = useState(initial);
    return <div className="prg-modal-bg" onMouseDown={e => { if (e.target === e.currentTarget) onCancel(); }}><div className="prg-modal" role="dialog" aria-label={title}>
        <h3>{title}</h3><div className="prg-form"><label>Nazwa programu</label><input autoFocus value={name} maxLength={80} onChange={e => setName(e.target.value)} onKeyDown={e => e.key === "Enter" && name.trim() && onOk(name.trim())} /></div>
        <div className="prg-modal-actions"><button className="secondary-button" onClick={onCancel}>Anuluj</button><button className="primary-button" disabled={!name.trim()} onClick={() => onOk(name.trim())}>Utwórz</button></div>
    </div></div>;
}
