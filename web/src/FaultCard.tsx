import { useEffect, useMemo, useRef, useState } from "react";
import { createPortal } from "react-dom";
import { faultDownloadUrl, fetchFaultEvents, fetchFaultSummary, fetchFaultTrace, type FaultEvent, type FaultListItem, type FaultMini, type FaultSummary, type FaultTrace } from "./faultSnapshot";
import { useActivity } from "./robotActivity";

// Karta „Awaria w skrócie” przy czerwonym komunikacie (makieta temp/komunikaty/v2d): robot z podświetloną osią,
// kilka próbek przed i po awarii (pomiar + plan), zdarzenia CAN, które wywołały błąd, i szczegółowa analiza.
// Ciężkie elementy (scena 3D, pobieranie) powstają tylko przy otwartym panelu.
const fmt = (value: number, digits = 1) => value.toFixed(digits).replace(".", ",");
const planLabel = { reference: "plan: cel planowany", command: "plan: cel F5", hold: "plan: pozycja trzymania" } as const;

export function FaultCard({ snapshot, pending, visible }: { snapshot: FaultListItem | null; pending: boolean; visible: boolean }) {
    const [summary, setSummary] = useState<FaultSummary | null>(null), [error, setError] = useState(""), [dialog, setDialog] = useState(false);
    useEffect(() => {
        if (!visible || !snapshot) return;
        let closed = false;
        fetchFaultSummary(snapshot.id).then(value => { if (!closed) { setSummary(value); setError(""); } }).catch(() => { if (!closed) setError("Nie udało się odczytać migawki awarii."); });
        return () => { closed = true; };
    }, [visible, snapshot]);
    if (!snapshot) return pending ? <div className="fc fc-pending"><div className="fc-h">Awaria w skrócie<span>migawka w przygotowaniu…</span></div>
        <p className="fc-note">Nagranie 10 s przed i 5 s po awarii zapisuje się automatycznie; karta uzupełni się za kilka sekund.</p></div> : null;
    if (!visible) return null;
    if (error) return <div className="fc"><div className="fc-h">Awaria w skrócie</div><p className="fc-note">{error}</p></div>;
    if (!summary) return <div className="fc"><div className="fc-h">Awaria w skrócie<span>migawka {snapshot.id}</span></div><div className="fc-skeleton">Wczytywanie migawki…</div></div>;
    const axis = summary.axis;
    return <div className="fc" data-snapshot={summary.id}>
        <div className="fc-h">Awaria w skrócie<span>{summary.mini ? "" : `przebieg niedostępny: ${summary.mini_missing} · `}migawka {summary.id} · okno −{summary.window.pre_s} s … +{summary.window.post_s} s</span></div>
        {summary.hint && <p className="fc-hint"><b>Z nagrania (automatyczna analiza):</b> {summary.hint.text.replace(/^Z nagrania: /, "")}</p>}
        {(axis || summary.mini) && <div className={`fc-grid${axis && summary.mini ? "" : " one"}`}>
            {axis && <FaultRobot axis={axis} geometry={summary.geometry} inferred={summary.axis_source === "event"} />}
            {summary.mini && <MiniChart mini={summary.mini} />}
        </div>}
        <div className="fc-can"><div className="fc-cap">Migawka CAN — zdarzenia, które wywołały błąd</div>
            {summary.excerpt.length ? summary.excerpt.map((row, i) => <EventLine key={i} row={row} />) : <p className="fc-note">Brak zdarzeń kluczowych w oknie migawki.</p>}</div>
        <div className="fc-links">
            <button type="button" className="fc-link can" onClick={() => setDialog(true)}>Szczegółowa analiza <small>przebieg · {summary.key_events} zdarzeń kluczowych · {summary.rows.toLocaleString("pl-PL")} wierszy</small></button>
            <a className="fc-link" href={faultDownloadUrl(summary.id)} download>Pobierz migawkę <small>.jsonl</small></a>
            {!summary.has_shaft && <span className="fc-note">Starsza migawka: przebieg z ramek CAN (bez planu ruchu).</span>}
        </div>
        {dialog && createPortal(<div className="app-shell fd-portal"><FaultDialog summary={summary} onClose={() => setDialog(false)} /></div>, document.body)}
    </div>;
}

function EventLine({ row }: { row: FaultEvent }) {
    return <div className={`fc-row lvl-${row.level || "none"}`}><span className="fc-t">{row.t_ms > 0 ? "+" : ""}{fmt(row.t_ms)} ms</span><span className="fc-ev">{row.event}</span>
        <span>{row.axis ? `J${row.axis}` : "—"}</span><span className="fc-cmd">{row.command ? `${row.command}h` : ""}</span><span className="fc-det" title={row.detail}>{row.detail}</span></div>;
}

// Robot z podświetloną osią: istniejąca scena podglądu (geometria z konfiguracji w chwili awarii), poza z bieżącego odczytu.
function FaultRobot({ axis, geometry, inferred }: { axis: number; geometry: Record<string, unknown> | null; inferred: boolean }) {
    const host = useRef<HTMLDivElement>(null), scene = useRef<ReturnType<typeof import("./programPreviewScene").createProgramScene> | null>(null);
    const [ready, setReady] = useState(false), [failed, setFailed] = useState(false);
    const activity = useActivity();
    const pose = useMemo(() => activity.axes.map(a => a.joint && a.position !== null ? a.position : 0), [activity.axes]);
    useEffect(() => {
        let closed = false;
        void import("./programPreviewScene").then(engine => {
            if (closed || !host.current) return;
            try { scene.current = engine.createProgramScene(host.current, { maxFps: 20 }); setReady(true); } catch { setFailed(true); }
        }).catch(() => !closed && setFailed(true));
        return () => { closed = true; scene.current?.dispose(); scene.current = null; };
    }, []);
    useEffect(() => { if (ready) { scene.current?.setGeometry(geometry); scene.current?.setHighlight(axis); } }, [ready, geometry, axis]);
    useEffect(() => { if (ready) scene.current?.setPose(pose); }, [ready, pose]);
    return <div className="fc-robot"><div className="fc-robot-view" ref={host} aria-label={`Model robota z podświetloną osią J${axis}`} />
        <span className="fc-axis">J{axis} · CAN ID {axis}{inferred ? " · oś z pierwszego zdarzenia" : ""}</span>
        {failed && <span className="fc-note fc-robot-fail">Podgląd 3D niedostępny — oś J{axis}.</span>}</div>;
}

function MiniChart({ mini }: { mini: FaultMini }) {
    const W = 900, H = 290, x0 = 58, x1 = W - 12, y0 = 18, y1 = H - 30;
    const values = [...mini.samples.map(s => s[1]), ...mini.plan.map(p => p[1])];
    const span = Math.max(.5, Math.max(...values) - Math.min(...values)), lo = Math.min(...values) - span * .12, hi = Math.max(...values) + span * .12;
    const tx = (t: number) => x0 + (t - mini.start_ms) / Math.max(1, mini.end_ms - mini.start_ms) * (x1 - x0), ty = (v: number) => y1 - (v - lo) / (hi - lo) * (y1 - y0);
    const step = niceStep(hi - lo), ticks: number[] = [];
    for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) ticks.push(v);
    let line = "";
    mini.samples.forEach((s, i) => { const jump = i > 0 && mini.gaps.some(([a, b]) => mini.samples[i - 1][0] <= a && s[0] >= b); line += `${i && !jump ? "L" : "M"}${tx(s[0]).toFixed(1)},${ty(s[1]).toFixed(1)}`; });
    return <div className="fc-chart"><div className="fc-cap">J{mini.axis} · pozycja [{mini.unit}]<span><i className="lg-m" />pomiar{mini.plan_kind && <><i className="lg-p" />{planLabel[mini.plan_kind]}</>}<i className="lg-e" />po awarii</span></div>
        <svg viewBox={`0 0 ${W} ${H}`} role="img" aria-label={`Kilka próbek J${mini.axis} przed i po awarii`}>
            {ticks.map(v => <g key={v}><line x1={x0} x2={x1} y1={ty(v)} y2={ty(v)} className="grid" /><text x={x0 - 5} y={ty(v) + 3} textAnchor="end">{fmt(v, step < 1 ? 1 : 0)}</text></g>)}
            {mini.gaps.map(([a, b], i) => { const left = tx(Math.max(a, mini.start_ms)), right = tx(Math.min(b, mini.end_ms));
                return <g key={i}><rect x={left} y={y0} width={Math.max(2, right - left)} height={y1 - y0} className="gap" />
                    <text x={right - left < 70 ? left - 4 : (left + right) / 2} y={y0 + 14} textAnchor={right - left < 70 ? "end" : "middle"} className="gap-t">brak odpowiedzi {fmt((b - a) / 1000, 2)} s</text></g>; })}
            {mini.plan.length > 1 && <polyline points={mini.plan.map(p => `${tx(p[0])},${ty(p[1])}`).join(" ")} className="plan" />}
            <line x1={tx(0)} x2={tx(0)} y1={y0} y2={y1} className="zero" /><text x={tx(0)} y={H - 8} textAnchor="middle" className="zero-t">0 · awaria</text>
            <text x={x0} y={H - 8}>{fmt(mini.start_ms / 1000, 2)} s</text><text x={x1} y={H - 8} textAnchor="end">+{fmt(mini.end_ms / 1000, 2)} s</text>
            <path d={line} className="meas" />
            {mini.samples.map((s, i) => <circle key={i} cx={tx(s[0])} cy={ty(s[1])} r={4} className={s[0] >= 0 ? "after" : "before"} />)}
        </svg></div>;
}
function niceStep(range: number) {
    const raw = range / 4, power = 10 ** Math.floor(Math.log10(raw)), unit = raw / power;
    return (unit < 1.5 ? 1 : unit < 3.5 ? 2 : unit < 7.5 ? 5 : 10) * power;
}

const KINDS = [["key", "Zdarzenia kluczowe"], ["can", "CAN"], ["bridge", "Mostek"], ["log", "Dzienniki"], ["shaft", "Przebieg"], ["all", "Wszystko"]] as const;
function FaultDialog({ summary, onClose }: { summary: FaultSummary; onClose: () => void }) {
    const [trace, setTrace] = useState<FaultTrace | null>(null), [kind, setKind] = useState<string>("key"), [onlyAxis, setOnlyAxis] = useState(false), [query, setQuery] = useState("");
    const [rows, setRows] = useState<FaultEvent[]>([]), [total, setTotal] = useState(0), [error, setError] = useState("");
    useEffect(() => { fetchFaultTrace(summary.id).then(setTrace).catch(() => setError("Nie udało się odczytać przebiegu.")); }, [summary.id]);
    useEffect(() => {
        let closed = false;
        const timer = setTimeout(() => fetchFaultEvents(summary.id, kind, onlyAxis ? summary.axis : null, query, 0)
            .then(page => { if (!closed) { setRows(page.rows); setTotal(page.total); } }).catch(() => !closed && setError("Nie udało się odczytać zdarzeń.")), 200);
        return () => { closed = true; clearTimeout(timer); };
    }, [summary.id, summary.axis, kind, onlyAxis, query]);
    useEffect(() => { const key = (event: KeyboardEvent) => { if (event.key === "Escape") { event.stopPropagation(); onClose(); } }; window.addEventListener("keydown", key, true); return () => window.removeEventListener("keydown", key, true); }, [onClose]);
    const more = () => fetchFaultEvents(summary.id, kind, onlyAxis ? summary.axis : null, query, rows.length).then(page => setRows(current => [...current, ...page.rows]));
    const main = trace?.axes.find(a => a.axis === trace.axis) ?? trace?.axes[0];
    return <div className="fd-bg" onClick={event => { if (event.target === event.currentTarget) onClose(); }}>
        <div className="fd" role="dialog" aria-modal="true" aria-label={`Migawka awarii ${summary.id}`}>
            <div className="fd-head"><span className="mb-sev sev-error">BŁĄD</span><h2>Migawka awarii{summary.axis ? ` · J${summary.axis}` : ""}<small>{summary.time} · {summary.id}</small></h2>
                <span className="mb-space" /><a className="fc-link" href={faultDownloadUrl(summary.id)} download>Pobierz .jsonl</a>
                <button type="button" className="mb-icon" aria-label="Zamknij szczegółową analizę" onClick={onClose}><svg viewBox="0 0 24 24" aria-hidden="true"><path d="M6 6l12 12M18 6L6 18" /></svg></button></div>
            <div className="fd-meta">{summary.triggers.map((t, i) => <span key={i}><b>{t.event}</b> {t.detail}</span>)}
                <span>Okno <b>−{summary.window.pre_s} s … +{summary.window.post_s} s</b></span>
                {Object.entries(summary.counts).map(([name, count]) => <span key={name}>{name} <b>{count.toLocaleString("pl-PL")}</b></span>)}</div>
            <div className="fd-chart">{main ? <TraceChart axis={main} keyEvents={rows.filter(r => r.key && r.level === "error")} /> : <div className="fc-skeleton">{error || "Wczytywanie przebiegu…"}</div>}</div>
            <div className="fd-bar">{KINDS.map(([id, label]) => <button key={id} type="button" className={`mb-chip${kind === id ? " on" : ""}`} aria-pressed={kind === id} onClick={() => setKind(id)}>{label}</button>)}
                {summary.axis && <label className="fd-axis"><input type="checkbox" checked={onlyAxis} onChange={event => setOnlyAxis(event.target.checked)} />tylko J{summary.axis}</label>}
                <input type="search" aria-label="Szukaj w migawce" placeholder="Szukaj: zdarzenie, opis…" value={query} onChange={event => setQuery(event.target.value)} />
                <span className="mb-arch-count">{rows.length} z {total.toLocaleString("pl-PL")}</span></div>
            <div className="fd-table" role="table" aria-label="Zdarzenia migawki">
                <div className="fd-row hd" role="row"><span>t [ms]</span><span>czas</span><span>źródło</span><span>zdarzenie</span><span>oś</span><span>kom.</span><span>dane</span><span>opis</span></div>
                {rows.map((row, i) => <div key={i} role="row" className={`fd-row lvl-${row.level || "none"}${Math.abs(row.t_ms) < .05 ? " zero" : ""}`}>
                    <span className="fc-t">{row.t_ms > 0 ? "+" : ""}{fmt(row.t_ms)}</span><span className="fd-time">{row.time}</span><span className="fd-src">{row.source}</span><span className="fc-ev">{row.event}</span>
                    <span>{row.axis ? `J${row.axis}` : "—"}</span><span className="fc-cmd">{row.command ? `${row.command}h` : ""}</span><span className="fd-data">{row.data}</span><span className="fc-det" title={row.detail}>{row.detail}</span></div>)}
                {rows.length < total && <button type="button" className="fd-more" onClick={() => void more()}>Pokaż kolejne ({(total - rows.length).toLocaleString("pl-PL")})</button>}
            </div>
        </div></div>;
}

function TraceChart({ axis, keyEvents }: { axis: FaultTrace["axes"][number]; keyEvents: FaultEvent[] }) {
    const W = 1100, H = 200, x0 = 56, x1 = W - 12, y0 = 16, y1 = H - 24;
    const all = [...axis.samples, ...axis.plan];
    const t0 = Math.min(...all.map(p => p[0])), t1 = Math.max(...all.map(p => p[0]));
    const vs = all.map(p => p[1]), span = Math.max(.5, Math.max(...vs) - Math.min(...vs)), lo = Math.min(...vs) - span * .08, hi = Math.max(...vs) + span * .08;
    const tx = (t: number) => x0 + (t - t0) / Math.max(1, t1 - t0) * (x1 - x0), ty = (v: number) => y1 - (v - lo) / (hi - lo) * (y1 - y0);
    const path = (points: [number, number][]) => points.map((p, i) => `${i && p[0] - points[i - 1][0] < 400 ? "L" : "M"}${tx(p[0]).toFixed(1)},${ty(p[1]).toFixed(1)}`).join("");
    const seconds: number[] = []; for (let s = Math.ceil(t0 / 1000); s <= t1 / 1000; s++) seconds.push(s);
    return <svg viewBox={`0 0 ${W} ${H}`} className="fd-svg" role="img" aria-label={`Przebieg J${axis.axis} w oknie migawki`}>
        {seconds.map(s => <g key={s}><line x1={tx(s * 1000)} x2={tx(s * 1000)} y1={y0} y2={y1} className="grid" /><text x={tx(s * 1000)} y={H - 8} textAnchor="middle">{s > 0 ? "+" : ""}{s} s</text></g>)}
        {[lo + (hi - lo) * .1, (lo + hi) / 2, hi - (hi - lo) * .1].map(v => <text key={v} x={x0 - 6} y={ty(v) + 3} textAnchor="end">{fmt(v, 0)}°</text>)}
        {axis.gaps.map(([a, b], i) => <rect key={i} x={tx(a)} y={y0} width={Math.max(2, tx(b) - tx(a))} height={y1 - y0} className="gap" />)}
        {keyEvents.map((e, i) => <line key={i} x1={tx(e.t_ms)} x2={tx(e.t_ms)} y1={y0} y2={y1} className="mark" />)}
        <line x1={tx(0)} x2={tx(0)} y1={y0} y2={y1} className="zero" />
        {axis.plan.length > 1 && <path d={path(axis.plan)} className="plan" />}<path d={path(axis.samples)} className="meas" />
        <text x={x0 + 6} y={y0 + 10} className="legend">J{axis.axis} · pomiar{axis.plan_kind ? ` i ${axis.plan_kind === "reference" ? "cel planowany" : "cel F5"}` : ""} [° wału] · źródło: {axis.source === "shaft" ? "zapis wałów" : "ramki 31h"}</text>
    </svg>;
}
