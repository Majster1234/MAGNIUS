import { createRoot } from "react-dom/client";
import { useEffect, useMemo, useState } from "react";
import { useCanStatus } from "./canStatus";
import { dataIsStale, numeric, type Component, type State } from "./status";
import { useStatus } from "./useStatus";
import { ManualPage } from "./ManualPage";
import { ConfigurationPage } from "./ConfigurationPage";
import { AppShell, emptyManualHeader, pageFromHash } from "./AppShell";
import { DiagnosticsPage } from "./DiagnosticsPage";
import { DiagnosticsHub, type DiagnosticsTab } from "./DiagnosticsHub";
import { ProgrammingPage } from "./ProgrammingPage";
import { PageHeader } from "./PageHeader";
import { ProblemList, useOverview } from "./ProcessPanel";
import { architectureRows } from "./architectureData";
import { axisPresence, busStateLabel } from "./dashboardData";
import { buildInfo, buildLabel } from "./buildInfo";
import { navResources } from "./navStatus";
import { driveCells, freshCan, pulseSections } from "./systemPulse";
import "./style.css";
import "./theme.css";
import "./messageBar.css";
import { PendantPage } from "./PendantPage";
import { initAnimations } from "./animations";
import { defaultMessageCenter } from "./messageCenter";
import { defaultActivityStore, useRobotFeed } from "./robotActivity";
import { useSystemMessages } from "./systemMessages";

const labels: Record<State, string> = { ok: "Aktywny", warn: "Uwaga", error: "Błąd", stale: "Brak aktualnych danych", not_configured: "Brak integracji" };
function Badge({ state, text }: { state: State; text?: string }) {
    return <span className={`badge ${state}`}><span className="dot" />{text ?? labels[state]}</span>;
}
function Metric({ label, value, unit = "" }: { label: string; value: unknown; unit?: string }) {
    const number = numeric(value);
    return <div className="metric"><span>{label}</span>
        <strong>{number === null ? "—" : number.toLocaleString("pl-PL", { maximumFractionDigits: 1 })}<small>{number === null ? "" : unit}</small></strong></div>;
}
function Card({ title, index, component, stale, children, statusText, link }: {
    title: string; index: string; component?: Component; stale: boolean; children?: React.ReactNode; statusText?: string; link?: [string, string];
}) {
    const state = stale || !component ? "stale" : component.state;
    return <article className="card">
        <div className="card-heading"><span className="index">{index}</span><h3>{title}</h3>
            <Badge state={state} text={statusText ?? (component?.id === "controllers" && state === "not_configured" ? "Nieuruchomione" : undefined)} /></div>
        <p className="description">{component?.message ?? "Oczekiwanie na pierwszy pomiar"}</p>
        {children}
        <div className="sample">Pomiar: {component?.sampled_at ? new Date(component.sampled_at).toLocaleTimeString("pl-PL") : "—"}
            {link && <a className="card-link" href={link[0]}>{link[1]} →</a>}</div>
    </article>;
}
const diagnosticsTabFromHash = (hash: string): DiagnosticsTab | null => hash === "#can" ? "can" : hash === "#architecture" ? "proc" : null;

function App() {
    const [view, setView] = useState(location.hash);
    const [tracePinnedAxis, setTracePinnedAxis] = useState<number | null>(null);
    const [diagTab, setDiagTab] = useState<DiagnosticsTab>(() => diagnosticsTabFromHash(location.hash) ?? "trace");
    const [highlightAxis, setHighlightAxis] = useState<number | null>(null);
    const [traceHost, setTraceHost] = useState<HTMLDivElement | null>(null);
    const pageId = pageFromHash(view);
    const [manualHeader, setManualHeader] = useState(emptyManualHeader);
    // Panel stanu w menu potrzebuje diagnostyki CAN na każdym widoku.
    const canStatus = useCanStatus(true);
    useEffect(() => {
        const update = () => {
            // Dawne adresy #can i #architecture otwierają odpowiednią zakładkę Diagnostyki.
            const tab = diagnosticsTabFromHash(location.hash);
            if (tab) { setDiagTab(tab); history.replaceState(null, "", "#diagnostics"); }
            setManualHeader(emptyManualHeader); setView(location.hash);
        };
        update();
        window.addEventListener("hashchange", update); return () => window.removeEventListener("hashchange", update);
    }, []);
    const { snapshot, connection, receivedAt, now } = useStatus();
    const stale = connection !== "online" || dataIsStale(snapshot, receivedAt, now);
    const c = snapshot?.components;
    const host = c?.host.values;
    const can = c?.can.values;
    const online = connection === "online" && now - receivedAt < 3000;
    // Pasek komunikatów: stan robota co 1 s na każdej stronie i problemy podzespołów systemu.
    useRobotFeed(defaultMessageCenter, defaultActivityStore);
    useSystemMessages(defaultMessageCenter, snapshot, online, !stale);
    const controllers = c?.controllers.values.controllers;
    const age = snapshot?.age_seconds == null ? null : snapshot.age_seconds + Math.max(0, now - receivedAt) / 1000;
    const presence = axisPresence(canStatus.snapshot, canStatus.connection === "online", canStatus.receivedAt, now);
    const drivesFresh = presence.every(value => value !== null), driveCount = presence.filter(Boolean).length;
    const driveComponent: Component = { id: "drives", state: !drivesFresh ? "stale" : driveCount === 6 ? "ok" : "warn",
        message: drivesFresh ? "Dostępność z diagnostyki CAN; potwierdzenie ENABLE i konfiguracji odbywa się oddzielnie." : "Oczekiwanie na aktualną diagnostykę CAN.",
        sampled_at: canStatus.snapshot?.updated_at ?? null, values: {} };
    const canOnline = canStatus.connection === "online", canFresh = freshCan(canStatus.snapshot, canOnline, canStatus.receivedAt, now);
    const sections = pulseSections({ status: snapshot, statusOnline: online, statusFresh: !stale, statusConnecting: connection === "connecting",
        can: canStatus.snapshot, canOnline, canFresh, build: buildLabel });
    const navStatus = { sections, drives: driveCells(canStatus.snapshot, canFresh), resources: navResources(snapshot, !stale, canStatus.snapshot, canFresh) };
    const overviewView = pageId === "overview";
    const { overview, error: overviewError } = useOverview(overviewView);
    const rows = useMemo(() => overviewView ? architectureRows({ status: snapshot, statusFresh: !stale, statusOnline: online, can: canStatus.snapshot, canFresh, canOnline,
        overview, overviewError, build: buildInfo }) : [], [overviewView, snapshot, stale, online, canStatus.snapshot, canFresh, canOnline, overview, overviewError]);
    const openProcess = () => { setDiagTab("proc"); location.hash = "diagnostics"; };
    const cores = Array.isArray(host?.cpu_per_core_percent) ? host.cpu_per_core_percent.length : null;
    return (
        <AppShell view={pageId} manual={manualHeader} online={online} connecting={connection === "connecting"} status={navStatus}
            serverUrl={snapshot?.network.url} subnet={snapshot?.network.subnet}>
            {pageId === "config" ? <ConfigurationPage /> : pageId === "manual" ? <ManualPage onHeaderStatus={setManualHeader} /> : pageId === "programming" ? <ProgrammingPage />
                : pageId === "diagnostics" ? <DiagnosticsHub status={snapshot} statusFresh={!stale} statusOnline={online} can={canStatus.snapshot} canConnection={canStatus.connection}
                    canFresh={canFresh} canOnline={canOnline} tab={diagTab} onTab={tab => { setDiagTab(tab); if (tab !== "trace") setHighlightAxis(null); }}
                    onTraceHost={setTraceHost} onAxis={setHighlightAxis} />
                : <section className="overview-screen" aria-label="Przegląd systemu">
                    <PageHeader icon="overview" title="Przegląd systemu"
                        info="Stan połączeń i podzespołów w jednym miejscu. Zużycie CPU, RAM i magistrali CAN pokazuje panel stanu w menu; szczegóły w Diagnostyce."
                        state={<Badge state={stale ? "stale" : "ok"} text={stale ? "Oczekiwanie na aktualne dane" : "Diagnostyka dostępna"} />}
                        actions={<span className="ph-meta">Wiek danych: <b>{age === null ? "—" : `${Math.floor(age)} s`}</b></span>} />
                    <section className="cards" aria-label="Statusy podzespołów">
                        <Card title="Backend API" index="01" component={c?.api} stale={!online}>
                            <div className="detail-row"><span>Adres panelu</span><code>{snapshot?.network.url ?? location.origin}</code></div>
                            <div className="detail-row"><span>Interfejs LAN</span><code>{snapshot?.network.interface ?? "—"}</code></div>
                        </Card>
                        <Card title="ROS 2 Humble" index="02" component={c?.ros} stale={stale}>
                            <div className="detail-row"><span>Źródło danych</span><code>/robot/system_status</code></div>
                            <div className="detail-row"><span>Komunikacja ROS</span><span>Lokalna na komputerze robota</span></div>
                        </Card>
                        <Card title="Komputer robota" index="03" component={c?.host} stale={stale}>
                            <div className="detail-row"><span>Wątki procesora</span><span>{cores ?? "—"}</span></div>
                            <div className="detail-row"><span>Pamięć RAM</span><span>{numeric(host?.memory_total_bytes) === null ? "—" : `${(Number(host?.memory_total_bytes) / 2 ** 30).toLocaleString("pl-PL", { maximumFractionDigits: 1 })} GB`}</span></div>
                            <div className="detail-row"><span>Czas pracy</span><span>{numeric(host?.uptime_seconds) === null ? "—"
                                : `${Math.floor(Number(host?.uptime_seconds) / 3600)} godz. ${Math.floor((Number(host?.uptime_seconds) % 3600) / 60)} min`}</span></div>
                        </Card>
                        <Card title="Magistrala CAN" index="04" component={c?.can} stale={stale} link={["#can", "Diagnostyka"]}>
                            <div className="detail-row"><span>Interfejs / stan</span><code>{String(can?.interface ?? "—")} · {busStateLabel(can?.bus_state)}</code></div>
                            <div className="detail-row"><span>Bitrate</span><span>{numeric(can?.bitrate)?.toLocaleString("pl-PL") ?? "—"} bit/s</span></div>
                            <div className="metrics compact"><Metric label="Ramki RX" value={can?.rx_packets} /><Metric label="Ramki TX" value={can?.tx_packets} /><Metric label="Błędy magistrali" value={can?.bus_errors} /></div>
                        </Card>
                        <Card title="Kontrolery ruchu" index="05" component={c?.controllers} stale={stale}>
                            {Array.isArray(controllers) && controllers.length > 0
                                ? <ul className="controller-list">{controllers.map((item, index) => <li key={index}>{String(item.name)}<code>{String(item.state)}</code></li>)}</ul>
                                : <div className="empty-state">Lista pojawi się po uruchomieniu ros2_control.</div>}
                        </Card>
                        <Card title="Napędy · 6 osi" index="06" component={driveComponent} stale={!drivesFresh} statusText={drivesFresh ? `${driveCount}/6 dostępnych` : undefined} link={["#manual", "ENABLE, zero i JOG"]}>
                            <div className="joints" aria-label="Sześć osi robota">{[1, 2, 3, 4, 5, 6].map((n) =>
                                <span key={n} className={`j${n}`} data-present={presence[n - 1] === null ? "unknown" : String(presence[n - 1])}
                                    title={presence[n - 1] === null ? "Brak aktualnych danych" : presence[n - 1] ? "Napęd odpowiada" : "Napęd niedostępny"}>J{n}<i /></span>)}</div>
                        </Card>
                    </section>
                    <section className="panel overview-problems" aria-label="Problemy systemu">
                        <div className="panel-head"><h2>Problemy systemu</h2><a className="secondary-button" href="#architecture">Procesy w Diagnostyce →</a></div>
                        <ProblemList rows={rows} onSelect={openProcess} />
                    </section>
                </section>}
            {(pageId === "diagnostics" || tracePinnedAxis !== null) && <DiagnosticsPage visible={pageId === "diagnostics" && diagTab === "trace"} host={traceHost}
                highlightAxis={highlightAxis} pinnedAxis={tracePinnedAxis} onPin={setTracePinnedAxis} onClose={() => setTracePinnedAxis(null)}
                onDock={() => { setTracePinnedAxis(null); setDiagTab("trace"); setView("#diagnostics"); location.hash = "diagnostics"; }} />}
        </AppShell>
    );
}

// #pendant: teach pendant na telefon — pełny ekran bez ramki aplikacji (bez zapytań panelu głównego).
function Root() {
    const [pendant, setPendant] = useState(location.hash === "#pendant");
    useEffect(() => { const update = () => setPendant(location.hash === "#pendant"); window.addEventListener("hashchange", update); return () => window.removeEventListener("hashchange", update); }, []);
    return pendant ? <PendantPage /> : <App />;
}

initAnimations();
createRoot(document.getElementById("root")!).render(<Root />);
