import { useCallback, useEffect, useMemo, useRef, useState, type KeyboardEvent as ReactKeyboardEvent } from "react";
import { ControlAnnouncements, Explanation } from "./ControlMessageCenter";
import { FaultCard } from "./FaultCard";
import { matchSnapshot, ROBOT_FAULT_SOURCES, SNAPSHOT_PENDING_MS, useFaultSnapshots, type FaultListItem } from "./faultSnapshot";
import { controlSourceLabel } from "./controlMessages";
import type { ConsoleEntry } from "./eventLog";
import { messageModule, messageModules, useMessageCenter, useMessages, type ActiveMessage, type MessageModule } from "./messageCenter";
import { useActivity, type ActivityKind, type AxisActivityState } from "./robotActivity";

// Pasek komunikatów nad stroną (makieta temp/komunikaty/v2d). Po lewej czynność robota, w środku przewijane bieżące
// komunikaty, po prawej kwitowanie i liczniki. Kliknięcie rozwija panel z góry pod nagłówkiem strony — nagłówek z STOP
// zostaje odsłonięty. Treść „Czynności robota” powstaje tylko przy otwartej zakładce.
export const OPEN_MESSAGES_EVENT = "robot-messages:open";
type Tab = "now" | "act" | "arch";
const SEV_LABEL = { error: "BŁĄD", warning: "UWAGA", info: "INFO" } as const;
const clock = (timestamp: number, ms = false) => new Date(timestamp).toLocaleTimeString("pl-PL", { hour12: false, hour: "2-digit", minute: "2-digit", second: "2-digit", ...(ms ? { fractionalSecondDigits: 3 } : {}) });
const duration = (from: number, now: number) => { const minutes = Math.floor(Math.max(0, now - from) / 60000); return minutes < 1 ? "teraz" : minutes < 60 ? `trwa ${minutes} min` : `trwa ${Math.floor(minutes / 60)} h ${minutes % 60} min`; };
const plural = (count: number, one: string, few: string, many: string) => `${count} ${count === 1 ? one : count % 10 >= 2 && count % 10 <= 4 && (count % 100 < 12 || count % 100 > 14) ? few : many}`;
const activityTone: Record<ActivityKind, string> = { move: "ok", operation: "plan", idle: "info", fault: "error", recovery: "warning", unknown: "info" };
const axisLabel: Record<AxisActivityState, [string, string]> = { moving: ["W RUCHU", "ok"], waiting: ["CZEKA", "info"], disabled: ["WYŁĄCZONA", "info"], unknown: ["BRAK DANYCH", "info"] };

function Check() { return <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M4 12l5 5L20 6" /></svg>; }
function Chevron() { return <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M6 9l6 6 6-6" /></svg>; }

export function MessageBar() {
    const center = useMessageCenter(), messages = useMessages(), activity = useActivity();
    const [open, setOpen] = useState(false), [pinned, setPinned] = useState(false), [tab, setTab] = useState<Tab>("now");
    const [tick, setTick] = useState(0), [hover, setHover] = useState(false), [top, setTop] = useState(38), [flash, setFlash] = useState(false);
    const [now, setNow] = useState(Date.now());
    const stripRef = useRef<HTMLDivElement>(null);
    const { active, entries, unacknowledged, worst } = messages;
    const errors = active.filter(m => m.severity === "error"), warnings = active.filter(m => m.severity === "warning");
    const current = active.length ? active[tick % active.length] : null;
    // Migawki awarii: lista przy otwartym panelu; co 2 s, dopóki świeży błąd robota czeka na swoje nagranie.
    const waiting = errors.some(m => ROBOT_FAULT_SOURCES.has(m.source) && Date.now() - m.since < SNAPSHOT_PENDING_MS);
    const { snapshots } = useFaultSnapshots(open, open && waiting);

    // Panel opada spod nagłówka strony: nagłówek (STOP, akcje) pozostaje dostępny.
    const measure = useCallback(() => {
        const strip = stripRef.current, workspace = strip?.parentElement;
        if (!strip || !workspace) return;
        const header = workspace.querySelector(".app-content header");
        const offset = header ? header.getBoundingClientRect().bottom - workspace.getBoundingClientRect().top + 4 : 0;
        setTop(Math.max(strip.offsetHeight, Math.round(offset)));
    }, []);
    const show = useCallback((next: boolean, nextTab?: Tab) => { if (next) measure(); if (nextTab) setTab(nextTab); setOpen(next); }, [measure]);
    useEffect(() => {
        const openEvent = (event: Event) => show(true, (event as CustomEvent<{ tab?: Tab }>).detail?.tab ?? "now");
        const key = (event: KeyboardEvent) => { if (event.key === "Escape") setOpen(false); };
        const close = () => setOpen(false);
        window.addEventListener(OPEN_MESSAGES_EVENT, openEvent); window.addEventListener("keydown", key); window.addEventListener("hashchange", close);
        return () => { window.removeEventListener(OPEN_MESSAGES_EVENT, openEvent); window.removeEventListener("keydown", key); window.removeEventListener("hashchange", close); };
    }, [show]);
    useEffect(() => { if (!open) return; window.addEventListener("resize", measure); return () => window.removeEventListener("resize", measure); }, [open, measure]);
    // Karuzela: jeden timer co 4 s, wstrzymany przy otwartym panelu, kursorze na pasku i karcie w tle.
    useEffect(() => {
        if (open || hover || active.length < 2) return;
        const timer = setInterval(() => { if (!document.hidden) setTick(value => value + 1); }, 4000);
        return () => clearInterval(timer);
    }, [open, hover, active.length]);
    // Czas trwania warunków odświeżany co 30 s tylko przy otwartym panelu.
    useEffect(() => { if (!open) return; setNow(Date.now()); const timer = setInterval(() => setNow(Date.now()), 30000); return () => clearInterval(timer); }, [open]);
    // Nowy, niepokwitowany błąd: krótki błysk paska.
    const unackErrors = errors.filter(m => !m.acknowledged).length, previousErrors = useRef(unackErrors);
    useEffect(() => {
        if (unackErrors > previousErrors.current) { setFlash(true); const timer = setTimeout(() => setFlash(false), 1300); previousErrors.current = unackErrors; return () => clearTimeout(timer); }
        previousErrors.current = unackErrors;
    }, [unackErrors]);

    const tabKey = (event: ReactKeyboardEvent) => {
        if (event.key !== "ArrowRight" && event.key !== "ArrowLeft") return;
        const order: Tab[] = ["now", "act", "arch"], next = order[(order.indexOf(tab) + (event.key === "ArrowRight" ? 1 : 2)) % 3];
        setTab(next); document.getElementById(`message-tab-${next}`)?.focus();
    };
    const tone = activityTone[activity.kind];
    return <>
        <div ref={stripRef} className={`mb-strip sev-${worst}${open ? " open" : ""}${flash ? " flash" : ""}`} onMouseEnter={() => setHover(true)} onMouseLeave={() => setHover(false)}>
            <button type="button" className="mb-toggle" aria-expanded={open} aria-controls="message-panel" title={open ? "Zwiń komunikaty" : "Rozwiń komunikaty"} onClick={() => show(!open)}>
                <span className={`mb-activity tone-${tone}${activity.kind === "fault" && unackErrors ? " unack" : ""}`} aria-label="Czynność robota">
                    <span className="mb-activity-icon" aria-hidden="true" /><b>{activity.label}</b><span className="mb-activity-detail">{activity.detail}</span>
                    {activity.since !== null && activity.kind !== "unknown" && <time>{activity.kind === "fault" ? clock(activity.since) : `od ${clock(activity.since)}`}</time>}
                </span>
                <span className="mb-ticker" aria-hidden="true">
                    {current ? <span key={`${current.uid}-${tick}`} className="mb-tick">
                        {current.acknowledged ? <i className="mb-acked">✓</i> : current.fresh ? <i className="mb-new">NOWY</i> : null}
                        <span className={`mb-sev sev-${current.severity}`}>{SEV_LABEL[current.severity]}</span><span className="mb-module">{current.module}</span>
                        <b>{current.title ?? current.message}</b>{current.title && current.message !== current.title && <span className="mb-short">— {current.message}</span>}
                    </span> : <span key="ok" className="mb-tick"><span className="mb-sev sev-ok">OK</span><b>Brak aktywnych problemów</b>
                        {entries.length > 0 && <span className="mb-short">· ostatnie: {entries.at(-1)!.title ?? entries.at(-1)!.message} ({clock(entries.at(-1)!.timestamp)})</span>}</span>}
                </span>
                {active.length > 1 && <span className="mb-pager" aria-hidden="true">{active.slice(0, 8).map((m, i) => <i key={m.uid} className={i === tick % active.length ? "on" : ""} />)}<small>{(tick % active.length) + 1}/{active.length}</small></span>}
            </button>
            <div className="mb-right">
                {unacknowledged > 0 && <button type="button" className="mb-ack-strip" title="Kwituj wszystkie nowe komunikaty" onClick={() => center.acknowledgeAll()}><Check />Kwituj {unacknowledged}</button>}
                <span className={`mb-count e${errors.length ? " has" : ""}${errors.some(m => !m.acknowledged) ? " unack" : ""}`}>{errors.length === 0 ? "0 błędów" : plural(errors.length, "błąd", "błędy", "błędów")}</span>
                <span className={`mb-count w${warnings.length ? " has" : ""}${warnings.some(m => !m.acknowledged) ? " unack" : ""}`}>{warnings.length === 0 ? "0 uwag" : plural(warnings.length, "uwaga", "uwagi", "uwag")}</span>
                <button type="button" className="mb-chevron" aria-label={open ? "Zwiń komunikaty" : "Rozwiń komunikaty"} aria-expanded={open} onClick={() => show(!open)}><Chevron /></button>
            </div>
        </div>
        <div className={`mb-backdrop${open ? " open" : ""}`} style={{ top }} onClick={() => { if (!pinned) setOpen(false); }} />
        <section id="message-panel" className={`mb-drop message-panel sev-${worst}${open ? " open" : ""}`} style={{ top }} aria-label="Panel komunikatów" aria-hidden={!open}>
            <div className="mb-head">
                <div role="tablist" aria-label="Komunikaty i czynności robota" onKeyDown={tabKey}>
                    {([["now", "Aktualne", active.length], ["act", "Czynności robota", null], ["arch", "Archiwum", entries.length]] as const).map(([id, label, count]) =>
                        <button key={id} type="button" role="tab" id={`message-tab-${id}`} aria-selected={tab === id} aria-controls={`message-pane-${id}`} tabIndex={tab === id ? 0 : -1}
                            className={`mb-tab${tab === id ? " on" : ""}`} onClick={() => setTab(id)}>{label}{count !== null && <em>{count}</em>}
                            {id === "now" && <span className={`mb-dot sev-${worst}`} />}</button>)}
                </div>
                <span className="mb-space" />
                <button type="button" className="mb-ack-all" disabled={!unacknowledged} onClick={() => center.acknowledgeAll()}><Check />{unacknowledged ? `Kwituj wszystkie (${unacknowledged})` : "Wszystko pokwitowane"}</button>
                <button type="button" className={`mb-icon${pinned ? " on" : ""}`} aria-pressed={pinned} title="Przypnij: panel nie zamknie się po kliknięciu obok" onClick={() => setPinned(value => !value)}>
                    <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M9 3h6l-1 6 4 4H6l4-4z" /><path d="M12 13v8" /></svg></button>
                <button type="button" className="mb-icon" aria-label="Zwiń panel komunikatów" title="Zwiń (Esc)" onClick={() => setOpen(false)}>
                    <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M6 6l12 12M18 6L6 18" /></svg></button>
            </div>
            <div className="mb-body">
                <div className="mb-pane mb-now" role="tabpanel" id="message-pane-now" aria-labelledby="message-tab-now" hidden={tab !== "now"}>
                    <CurrentMessages active={active} now={now} open={open && tab === "now"} snapshots={snapshots} onAck={uid => center.acknowledge(uid)} onView={uid => center.markViewed(uid)} />
                </div>
                <div className="mb-pane" role="tabpanel" id="message-pane-act" aria-labelledby="message-tab-act" hidden={tab !== "act"}>
                    {open && tab === "act" && <ActivityPane />}
                </div>
                <div className="mb-pane mb-arch" role="tabpanel" id="message-pane-arch" aria-labelledby="message-tab-arch" hidden={tab !== "arch"}>
                    <Archive entries={entries} open={open && tab === "arch"} snapshots={snapshots} />
                </div>
            </div>
            <div className="mb-foot">
                <span className="mb-legend"><i className="sev-error" />Błąd</span><span className="mb-legend"><i className="sev-warning" />Uwaga</span>
                <span className="mb-legend"><i className="sev-info" />Info</span><span className="mb-legend"><i className="sev-ok" />Ruch / OK</span>
                <span className="mb-legend"><i className="mb-new">NOWY</i>nieobejrzany (zdejmuje najechanie)</span>
                <span className="mb-foot-hint">Kliknij pasek albo <kbd>Esc</kbd>, aby zwinąć · nagłówek strony ze STOP pozostaje dostępny</span>
            </div>
        </section>
        <ControlAnnouncements entries={entries} active={active} />
    </>;
}

function CurrentMessages({ active, now, open, snapshots, onAck, onView }: { active: ActiveMessage[]; now: number; open: boolean; snapshots: FaultListItem[];
    onAck: (uid: string) => void; onView: (uid: string) => void }) {
    if (!active.length) return <div className="mb-empty"><span>✓</span><h3>Brak aktywnych problemów</h3><p>Zakończone i pokwitowane komunikaty są w Archiwum.</p></div>;
    const groups = [["error", "Błędy"], ["warning", "Uwagi"], ["info", "Informacje"]] as const;
    return <>{groups.map(([severity, name]) => {
        const items = active.filter(m => m.severity === severity);
        if (!items.length) return null;
        return <section key={severity} className={`mb-group sev-${severity}`} aria-label={name}>
            <h2>{name} <em>{items.length}</em></h2>
            {items.map(m => <MessageCard key={m.uid} m={m} now={now} open={open} snapshots={snapshots} onAck={onAck} onView={onView} />)}
        </section>;
    })}</>;
}

// Karta komunikatu ma dwie formy: zwiniętą (nazwa i czego dotyczy) i rozwiniętą (pełna treść, migawka, kwitowanie);
// stan rozwinięcia przypięty do uid (klucz karty). Treść zwiniętej karty zostaje w DOM (inert), rozwijanie animowane
// wysokością; migawka awarii pobierana po rozwinięciu. Najechanie (250 ms),
// fokus lub rozwinięcie zdejmuje NOWY — bez kwitowania.
const VIEW_DWELL_MS = 250, NEW_LEAVE_MS = 560;
function MessageCard({ m, now, open, snapshots, onAck, onView }: { m: ActiveMessage; now: number; open: boolean; snapshots: FaultListItem[];
    onAck: (uid: string) => void; onView: (uid: string) => void }) {
    const [expanded, setExpanded] = useState(false), [closing, setClosing] = useState(false), [leaving, setLeaving] = useState(false);
    // Zdjęcie NOWY: znacznik zostaje na czas animacji wyjazdu, reszta wiersza wjeżdża z prawej na jego miejsce.
    const wasFresh = useRef(m.fresh);
    useEffect(() => {
        const was = wasFresh.current; wasFresh.current = m.fresh;
        if (m.fresh) { setLeaving(false); return; }
        if (!was) return;
        setLeaving(true); const timer = setTimeout(() => setLeaving(false), NEW_LEAVE_MS); return () => clearTimeout(timer);
    }, [m.fresh]);
    const dwell = useRef<ReturnType<typeof setTimeout> | null>(null), bodyId = `mb-card-body-${m.uid}`;
    const cancel = () => { if (dwell.current) { clearTimeout(dwell.current); dwell.current = null; } };
    useEffect(() => cancel, []);
    useEffect(() => { if (!open) cancel(); }, [open]);
    const view = () => { cancel(); if (m.fresh) onView(m.uid); };
    // Migawka zostaje zamontowana do końca animacji zwijania (zegar zamiast transitionend: ukryty panel nie animuje).
    useEffect(() => { if (!closing) return; const timer = setTimeout(() => setClosing(false), 320); return () => clearTimeout(timer); }, [closing]);
    const toggle = () => { view(); setClosing(expanded); setExpanded(value => !value); };
    return <article id={`control-issue-${m.source}`} data-source={m.source}
        className={`control-issue issue-${m.severity} mb-card${m.acknowledged ? " acked" : ""}${expanded ? " expanded" : ""}`}
        onMouseEnter={() => { if (open && m.fresh && !dwell.current) dwell.current = setTimeout(view, VIEW_DWELL_MS); }} onMouseLeave={cancel}>
        <div className="mb-card-head" onClick={toggle}>
            {(m.fresh || leaving) && <span className={`mb-new-slot${m.fresh ? "" : " leaving"}`}><span><i className="mb-new">NOWY</i></span></span>}
            <span className={`mb-sev sev-${m.severity}`}>{SEV_LABEL[m.severity]}</span>
            <h3>{m.title ?? m.message}</h3>
            <span className="mb-card-about">{m.module} · {m.context ?? controlSourceLabel(m.source)}</span>
            {m.unobservedSince !== undefined && <span className="mb-stale" title={`Moduł zamknięty — stan z ${clock(m.unobservedSince)}; odświeży się, gdy moduł znów go sprawdzi`}>stan z {clock(m.unobservedSince)}</span>}
            <button type="button" className="mb-card-toggle" aria-expanded={expanded} aria-controls={bodyId} onFocus={() => { if (open) view(); }}
                aria-label={expanded ? "Zwiń komunikat" : "Rozwiń komunikat"} title={expanded ? "Zwiń" : "Rozwiń szczegóły"}><Chevron /></button>
        </div>
        <div className="mb-card-body" id={bodyId} inert={!expanded} aria-hidden={!expanded}>
            <div className="mb-card-inner">
                <div className="mb-card-top">
                    <span className="mb-module">{m.module}</span>
                    <small>{m.context ?? controlSourceLabel(m.source)} · {m.kind === "condition" ? "warunek" : "wynik operacji"}</small>
                    <time dateTime={new Date(m.since).toISOString()}>{clock(m.since)}{m.kind === "condition" ? ` · ${duration(m.since, now)}` : ""}</time>
                </div>
                <div className="mb-explain"><Explanation notice={m} expanded={m.severity === "error"} /></div>
                {m.severity === "error" && <FaultCard snapshot={matchSnapshot(m, m.since, snapshots)} visible={open && (expanded || closing)}
                    pending={ROBOT_FAULT_SOURCES.has(m.source) && Date.now() - m.since < SNAPSHOT_PENDING_MS} />}
                <div className="mb-card-foot">
                    {m.acknowledged ? <span className="mb-acked-note">✓ pokwitowany {clock(m.acknowledgedAt!)}{m.kind === "condition" ? " · nadal aktywny, zniknie po usunięciu przyczyny" : ""}</span>
                        : <><span className="mb-kind-note">{m.unobservedSince !== undefined ? `Źródło komunikatu (moduł ${m.module}) zostało zamknięte — stan z ${clock(m.unobservedSince)}. Odświeży się, gdy moduł znów go sprawdzi; pokwitowanie usuwa komunikat.`
                            : m.kind === "condition" ? "Po pokwitowaniu zostaje na liście, dopóki przyczyna nie zniknie." : "Pokwitowanie przenosi wynik do archiwum."}</span>
                            <button type="button" className="mb-ack" onClick={() => onAck(m.uid)}><Check />Kwituj</button></>}
                </div>
            </div>
        </div>
    </article>;
}

function ActivityPane() {
    const activity = useActivity(), tone = activityTone[activity.kind];
    const caption = activity.kind === "fault" ? "Stan osi po zatrzymaniu" : "Stan osi teraz";
    return <div className="mb-activity-pane">
        <div className="mb-act-now">
            <div className={`mb-now-card tone-${tone}`}><span>{activity.kind === "fault" ? "Zatrzymany · ruch zablokowany" : activity.kind === "move" ? "Robot fizyczny · w ruchu" : activity.kind === "unknown" ? "Brak danych" : "Robot fizyczny"}</span>
                <h3>{activity.label} · {activity.detail}</h3>
                {activity.since !== null && <p>{activity.kind === "fault" ? `Zatrzymanie o ${clock(activity.since)}.` : `Od ${clock(activity.since)}.`}
                    {activity.kind === "fault" && activity.log[1] ? ` Wcześniej: ${activity.log[1].label.toLowerCase()} · ${activity.log[1].detail}.` : ""}</p>}
            </div>
            <div className="mb-caption">{caption} · odczyt co 1 s</div>
            <div className="mb-axes">{activity.axes.map(axis => { const [label, axisTone] = axisLabel[axis.state];
                return <div key={axis.axis} className={`mb-axis tone-${axisTone}${axis.state === "moving" ? " moving" : ""}`}>
                    <span className="mb-axis-state">{label}</span>
                    <div><b>J{axis.axis}</b><strong>{axis.position === null ? "—" : `${axis.position.toFixed(2)}°`}</strong>
                        <small>{axis.position !== null && !axis.joint ? "wał · " : ""}{axis.speedRpm === null ? "—" : `${axis.speedRpm.toFixed(1)} obr./min`}</small></div>
                </div>; })}</div>
            <p className="mb-note">Stan programu i praca automatyczna mają osobne ekrany. Tu tylko to, co robot fizycznie robi lub robił.</p>
        </div>
        <div className="mb-act-log"><h4>Ostatnie czynności (ta sesja panelu)</h4>
            {activity.log.length ? activity.log.map((event, index) => <div key={`${event.at}-${index}`} className={`mb-event tone-${activityTone[event.kind]}${index === 0 ? " current" : ""}`}>
                <time>{clock(event.at)}</time><span>{event.label}<small>{event.detail}</small></span></div>) : <p className="mb-note">Brak zarejestrowanych czynności.</p>}
        </div>
    </div>;
}

function Archive({ entries, open, snapshots }: { entries: ConsoleEntry[]; open: boolean; snapshots: FaultListItem[] }) {
    const [severity, setSeverity] = useState<"all" | "error" | "warning" | "info">("all"), [module, setModule] = useState<MessageModule | "">(""), [query, setQuery] = useState("");
    const rows = useMemo(() => {
        const text = query.trim().toLowerCase();
        return [...entries].reverse().filter(entry => (severity === "all" || entry.severity === severity) && (!module || messageModule(entry) === module) &&
            (!text || `${entry.title ?? ""} ${entry.message} ${entry.context ?? ""}`.toLowerCase().includes(text)));
    }, [entries, severity, module, query]);
    return <>
        <div className="mb-arch-bar">
            {([["all", "Wszystkie"], ["error", "Błędy"], ["warning", "Uwagi"], ["info", "Info"]] as const).map(([id, label]) =>
                <button key={id} type="button" className={`mb-chip sev-${id}${severity === id ? " on" : ""}`} aria-pressed={severity === id} onClick={() => setSeverity(id)}>{id !== "all" && <i />}{label}</button>)}
            <select aria-label="Moduł" value={module} onChange={event => setModule(event.target.value as MessageModule | "")}><option value="">Moduł: wszystkie</option>
                {messageModules.map(name => <option key={name}>{name}</option>)}</select>
            <input type="search" aria-label="Szukaj w archiwum" placeholder="Szukaj w archiwum…" value={query} onChange={event => setQuery(event.target.value)} />
            <span className="mb-arch-count">{rows.length} z {entries.length} · ta sesja panelu</span>
        </div>
        <div className="mb-arch-list control-history-log" role="log" aria-label="Historia komunikatów sterowania" aria-live="off" tabIndex={0}>
            {rows.map(entry => <ArchiveEntry key={entry.id} entry={entry} open={open} snapshots={snapshots} />)}
            {!rows.length && <div className="mb-empty small">Brak wpisów dla tego filtra.</div>}
        </div>
    </>;
}

// Wpis archiwum; migawka awarii (dla błędów) pobierana dopiero po rozwinięciu wpisu.
function ArchiveEntry({ entry, open, snapshots }: { entry: ConsoleEntry; open: boolean; snapshots: FaultListItem[] }) {
    const [expanded, setExpanded] = useState(false);
    return <details className={`history-entry issue-${entry.severity}`} onToggle={event => setExpanded((event.currentTarget as HTMLDetailsElement).open)}><summary>
                <time dateTime={new Date(entry.timestamp).toISOString()}>{clock(entry.timestamp, true)}</time>
                <span className={`mb-sev sev-${entry.severity}`}>{SEV_LABEL[entry.severity]}</span>
                <span className="mb-module">{messageModule(entry)}</span>
                <span className="mb-arch-context">{entry.context ?? controlSourceLabel(entry.source)} · {controlSourceLabel(entry.source)}</span>
                <strong>{entry.title ?? entry.message}</strong>{entry.repetitions && entry.repetitions > 1 ? <small>×{entry.repetitions}</small> : null}</summary>
                <div className="mb-arch-detail"><Explanation notice={entry} expanded={entry.severity === "error"} />
                    {entry.severity === "error" && expanded && <FaultCard snapshot={matchSnapshot(entry, entry.timestamp, snapshots)} visible={open} pending={false} />}</div></details>;
}
