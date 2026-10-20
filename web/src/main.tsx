import { createRoot } from "react-dom/client";
import { useEffect, useState } from "react";
import { useCanStatus, type CanDevice } from "./canStatus";
import { dataIsStale, numeric, type Component, type State } from "./status";
import { useStatus } from "./useStatus";
import { ManualPage } from "./ManualPage";
import { ConfigurationPage } from "./ConfigurationPage";
import { AppShell, emptyManualHeader, pageFromHash } from "./AppShell";
import { ArchitecturePage } from "./ArchitecturePage";
import { axisPresence, busStateLabel } from "./dashboardData";
import "./style.css";

const labels: Record<State, string> = {
    ok: "Aktywny",
    warn: "Uwaga",
    error: "Błąd",
    stale: "Brak aktualnych danych",
    not_configured: "Brak integracji",
};
function Badge({ state, text }: { state: State; text?: string }) {
    return (
        <span className={`badge ${state}`}>
            <span className="dot" />
            {text ?? labels[state]}
        </span>
    );
}
function Metric({
    label,
    value,
    unit = "",
}: {
    label: string;
    value: unknown;
    unit?: string;
}) {
    const number = numeric(value);
    return (
        <div className="metric">
            <span>{label}</span>
            <strong>
                {number === null
                    ? "—"
                    : number.toLocaleString("pl-PL", {
                          maximumFractionDigits: 1,
                      })}
                <small>{number === null ? "" : unit}</small>
            </strong>
        </div>
    );
}
function Card({
    title,
    index,
    component,
    stale,
    children,
    statusText,
}: {
    title: string;
    index: string;
    component?: Component;
    stale: boolean;
    children?: React.ReactNode;
    statusText?: string;
}) {
    const state = stale || !component ? "stale" : component.state;
    return (
        <article className="card">
            <div className="card-heading">
                <span className="index">{index}</span>
                <h3>{title}</h3>
                <Badge
                    state={state}
                    text={statusText ?? (
                        component?.id === "controllers" &&
                        state === "not_configured"
                            ? "Nieuruchomione"
                            : undefined)
                    }
                />
            </div>
            <p className="description">
                {component?.message ?? "Oczekiwanie na pierwszy pomiar"}
            </p>
            {children}
            <div className="sample">
                Pomiar:{" "}
                {component?.sampled_at
                    ? new Date(component.sampled_at).toLocaleTimeString("pl-PL")
                    : "—"}
            </div>
        </article>
    );
}
function CanPage() {
    const { snapshot, connection } = useCanStatus(true);
    const [page, setPage] = useState(0);
    const [action, setAction] = useState<string | null>(null);
    const devices = snapshot?.devices ?? [];
    const totalPages = Math.max(1, Math.ceil(devices.length / 25));
    const visible = devices.slice(page * 25, page * 25 + 25);
    const summary = snapshot?.summary.values;
    const scan = snapshot?.scan.values;
    const stale = connection !== "online" || !snapshot?.fresh;
    useEffect(() => setPage((old) => Math.min(old, totalPages - 1)), [totalPages]);
    async function request(actionName: "scan" | "reset") {
        setAction(actionName);
        try {
            const response = await fetch(`/api/v1/can/${actionName}`, { method: "POST" });
            const body = await response.json();
            if (!response.ok) throw new Error(body.detail ?? "Operacja odrzucona");
        } catch (error) {
            window.alert(error instanceof Error ? error.message : "Błąd operacji CAN");
        } finally { setAction(null); }
    }
    const value = (name: string) => summary?.[name];
    return <section className="can-screen" aria-label="Magistrala CAN">
        <div className="page-title"><div><div className="eyebrow">DIAGNOSTYKA SOCKETCAN</div><h1>Magistrala CAN</h1><p>Odczyt identyfikacji i statusów MKS SERVO42D CAN. Bez poleceń ruchu.</p></div><div className="can-actions"><button className="secondary-button" disabled={action !== null} onClick={() => request("reset")}>{action === "reset" ? "Resetowanie…" : "Reset CAN"}</button><button className="primary-button" disabled={action !== null || Boolean(scan?.scanning)} onClick={() => request("scan")}>{action === "scan" ? "Uruchamianie…" : "SCAN"}</button></div></div>
        <section className={`can-progress ${stale ? "waiting" : ""}`}><Badge state={stale ? "stale" : snapshot?.summary.state ?? "stale"} text={snapshot?.scan.message ?? "Oczekiwanie na diagnostykę CAN"}/><span>{Boolean(scan?.scanning) ? `Adres ${String(scan?.checked ?? 0)} / ${String(scan?.total ?? 2047)}` : String(scan?.state ?? "—")}</span><span className="progress-error">{scan?.error ? String(scan.error) : ""}</span></section>
        <section className="can-strip" aria-label="Podsumowanie magistrali"><div className="strip-state"><strong>{String(value("bus_state") ?? "—")}</strong><small>{String(value("interface") ?? "can0")}</small></div><Metric label="Bitrate" value={value("bitrate")} unit=" bit/s"/><Metric label="Urządzenia" value={value("device_count")}/><Metric label="Obieg odczytu" value={value("full_round_ms")} unit=" ms"/><Metric label="Błędy" value={value("errors")}/></section>
        <section className="can-table-card"><div className="table-heading"><div><h2>Urządzenia bieżącej sesji</h2><p>Wykryte i utracone adresy. Puste adresy nie są pokazywane.</p></div><span>{devices.length} rekordów</span></div>{devices.length === 0 ? <div className="can-empty"><span>⇄</span><strong>Tabela jest pusta</strong><p>Wyniki będą pojawiać się w trakcie automatycznego skanowania.</p></div> : <><div className="table-scroll"><table><thead><tr><th>CAN ID</th><th>Oś robota</th><th>Model</th><th>Łączność</th><th>Stan napędu</th><th>ENABLE</th><th>RTT [ms]</th></tr></thead><tbody>{visible.map((device) => <DeviceRow key={device.id} device={device}/>)}</tbody></table></div>{totalPages > 1 && <div className="pagination"><button disabled={page === 0} onClick={() => setPage(page - 1)}>Poprzednie</button><span>{page + 1} / {totalPages}</span><button disabled={page + 1 >= totalPages} onClick={() => setPage(page + 1)}>Następne</button></div>}</>}</section>
        <details className="card can-bus-details"><summary>Szczegóły komunikacji</summary>
            <div className="metrics compact">
                <Metric label="Planowane obciążenie" value={value("scheduled_utilization_percent")} unit="%" />
                <Metric label="Jitter cyklu" value={value("cycle_jitter_ms")} unit=" ms" />
                <Metric label="Odstęp między ramkami" value={value("rx_interval_mean_ms")} unit=" ms" />
                <Metric label="Ramki RX / sesja" value={value("rx_frames")} />
                <Metric label="Ramki TX / sesja" value={value("tx_frames")} />
            </div>
            <p className="muted-note">Obciążenie jest oszacowaniem kosztu zapytań i odpowiedzi. Nie obejmuje retransmisji.</p>
        </details>
        <p className="can-note">Aktualizacja panelu: 5 Hz. Pozycje mają osobne znaczniki czasu. SCAN i Reset CAN są blokowane podczas aktywnego sterowania.</p>
    </section>;
}
function DeviceRow({ device }: { device: CanDevice }) {
    const [open, setOpen] = useState(false);
    const v = device.values;
    const enable = v.enable === true ? "Włączony" : v.enable === false ? "Wyłączony" : "—";
    return <>
        <tr className="device-row">
            <td><button className="link-button" aria-expanded={open} onClick={() => setOpen(!open)}>{open ? "▾" : "▸"} {device.id}</button></td>
            <td>{device.id >= 1 && device.id <= 6 ? `J${device.id}` : "—"}</td>
            <td>{String(v.model ?? "—")}</td>
            <td><Badge state={device.state} text={device.message} /></td>
            <td>{String(v.motor_state ?? "Niedostępny")}</td>
            <td>{enable}</td>
            <td>{numeric(v.rtt_ms)?.toLocaleString("pl-PL", { maximumFractionDigits: 2 }) ?? "—"}</td>
        </tr>
        {open && <tr className="device-details"><td colSpan={7}><div>
            <span>Tryb: <b>{String(v.mode ?? "—")}</b></span>
            <span>Pozycja enkodera: <b>{numeric(v.position_ticks)?.toLocaleString("pl-PL") ?? "—"} impulsów</b></span>
            <span>Wiek pozycji: <b>{numeric(v.position_age_seconds) === null ? "—" : `${(Number(v.position_age_seconds) * 1000).toFixed(1)} ms`}</b></span>
            <span>Wiek ENABLE: <b>{numeric(v.enable_age_seconds) === null ? "—" : `${(Number(v.enable_age_seconds) * 1000).toFixed(1)} ms`}</b></span>
            <span>Stall: <b>{v.stall === true ? "Tak" : v.stall === false ? "Nie" : "—"}</b></span>
            <span>RTT min / śr. / max: <b>{numeric(v.rtt_min_ms)?.toFixed(2) ?? "—"} / {numeric(v.rtt_mean_ms)?.toFixed(2) ?? "—"} / {numeric(v.rtt_max_ms)?.toFixed(2) ?? "—"} ms</b></span>
            <span>Timeouty / błędne: <b>{String(v.timeouts ?? 0)} / {String(v.invalid_responses ?? 0)}</b></span>
            <span>Ostatni poprawny odczyt: <b>{numeric(v.last_ok_age_seconds)?.toFixed(1) ?? "—"} s temu</b></span>
        </div></td></tr>}
    </>;
}
function App() {
    const [view, setView] = useState(location.hash);
    const pageId = pageFromHash(view);
    const canView = pageId === "can", manualView = pageId === "manual", configurationView = pageId === "config";
    const [manualHeader, setManualHeader] = useState(emptyManualHeader);
    const canStatus = useCanStatus(pageId === "overview");
    useEffect(() => { const update = () => { setManualHeader(emptyManualHeader); setView(location.hash); }; window.addEventListener("hashchange", update); return () => window.removeEventListener("hashchange", update); }, []);
    const { snapshot, connection, receivedAt, now } = useStatus();
    const stale =
        connection !== "online" || dataIsStale(snapshot, receivedAt, now);
    const c = snapshot?.components;
    const host = c?.host.values;
    const can = c?.can.values;
    const online = connection === "online" && now - receivedAt < 3000;
    const controllers = c?.controllers.values.controllers;
    const age =
        snapshot?.age_seconds == null
            ? null
            : snapshot.age_seconds + Math.max(0, now - receivedAt) / 1000;
    const presence = axisPresence(canStatus.snapshot, canStatus.connection === "online", canStatus.receivedAt, now);
    const drivesFresh = presence.every(value => value !== null), driveCount = presence.filter(Boolean).length;
    const driveComponent: Component = { id: "drives", state: !drivesFresh ? "stale" : driveCount === 6 ? "ok" : "warn",
        message: drivesFresh ? "Dostępność z diagnostyki CAN; potwierdzenie ENABLE i konfiguracji odbywa się oddzielnie." : "Oczekiwanie na aktualną diagnostykę CAN.",
        sampled_at: canStatus.snapshot?.updated_at ?? null, values: {} };
    return (
        <AppShell view={pageId} manual={manualHeader} online={online} connecting={connection === "connecting"}>
                {configurationView ? <ConfigurationPage /> : manualView ? <ManualPage onHeaderStatus={setManualHeader} /> : canView ? <CanPage /> : pageId === "architecture" ? <ArchitecturePage /> : <>
                <div className="page-title">
                    <div>
                        <div className="eyebrow">MONITOROWANIE SYSTEMU</div>
                        <h1>Przegląd systemu</h1>
                        <p>Stan połączeń i podzespołów w jednym miejscu.</p>
                    </div>
                    <a className="primary-button page-action" href="#manual">Sterowanie ręczne →</a>
                </div>
                <section
                    className={`summary ${stale ? "waiting" : ""}`}
                    aria-label="Świeżość danych"
                >
                    <div className="summary-icon">{stale ? "◷" : "✓"}</div>
                    <div>
                        <h2>
                            {stale
                                ? "Oczekiwanie na aktualne dane"
                                : "Diagnostyka systemu jest dostępna"}
                        </h2>
                        <p>
                            {stale
                                ? "Statusy mogą być nieaktualne. Połączenie zostanie wznowione automatycznie."
                                : "Odbieramy statusy z ROS 2. Gotowość napędów nie została jeszcze potwierdzona."}
                        </p>
                    </div>
                    <span className="age">
                        {age === null
                            ? "BRAK POMIARU"
                            : `WIEK DANYCH: ${Math.floor(age)} s`}
                    </span>
                </section>
                <div className="section-title">
                    <h2>Podzespoły i połączenia</h2>
                    <span>01 — DIAGNOSTYKA</span>
                </div>
                <section className="cards" aria-label="Statusy podzespołów">
                    <Card
                        title="Backend API"
                        index="01"
                        component={c?.api}
                        stale={!online}
                    >
                        <div className="detail-row">
                            <span>Adres panelu</span>
                            <code>
                                {snapshot?.network.url ?? location.origin}
                            </code>
                        </div>
                        <div className="detail-row">
                            <span>Interfejs LAN</span>
                            <code>{snapshot?.network.interface ?? "—"}</code>
                        </div>
                    </Card>
                    <Card
                        title="ROS 2 Humble"
                        index="02"
                        component={c?.ros}
                        stale={stale}
                    >
                        <div className="detail-row">
                            <span>Źródło danych</span>
                            <code>/robot/system_status</code>
                        </div>
                        <div className="detail-row">
                            <span>Komunikacja ROS</span>
                            <span>Lokalna na komputerze robota</span>
                        </div>
                    </Card>
                    <Card
                        title="Komputer robota"
                        index="03"
                        component={c?.host}
                        stale={stale}
                    >
                        <div className="metrics">
                            <Metric
                                label="Użycie CPU"
                                value={host?.cpu_percent}
                                unit="%"
                            />
                            <Metric
                                label="Pamięć RAM"
                                value={host?.memory_percent}
                                unit="%"
                            />
                        </div>
                        <div className="detail-row">
                            <span>Czas pracy</span>
                            <span>
                                {numeric(host?.uptime_seconds) === null
                                    ? "—"
                                    : `${Math.floor(Number(host?.uptime_seconds) / 3600)} godz. ${Math.floor((Number(host?.uptime_seconds) % 3600) / 60)} min`}
                            </span>
                        </div>
                    </Card>
                    <div className="card-anchor">
                        <Card
                            title="Magistrala CAN"
                            index="04"
                            component={c?.can}
                            stale={stale}
                        >
                            <div className="detail-row">
                                <span>Interfejs / stan</span>
                                <code>
                                    {String(can?.interface ?? "—")} · {busStateLabel(can?.bus_state)}
                                </code>
                            </div>
                            <div className="detail-row">
                                <span>Bitrate</span>
                                <span>
                                    {numeric(can?.bitrate)?.toLocaleString(
                                        "pl-PL",
                                    ) ?? "—"}{" "}
                                    bit/s
                                </span>
                            </div>
                            <div className="metrics compact">
                                <Metric
                                    label="Ramki RX"
                                    value={can?.rx_packets}
                                />
                                <Metric
                                    label="Ramki TX"
                                    value={can?.tx_packets}
                                />
                                <Metric
                                    label="Błędy magistrali"
                                    value={can?.bus_errors}
                                />
                            </div>
                            <div className="detail-row">
                                <span>Błędy RX / TX</span>
                                <span>
                                    {numeric(can?.rx_errors) ?? "—"} /{" "}
                                    {numeric(can?.tx_errors) ?? "—"}
                                </span>
                            </div>
                        </Card>
                    </div>
                    <Card
                        title="Kontrolery ruchu"
                        index="05"
                        component={c?.controllers}
                        stale={stale}
                    >
                        {Array.isArray(controllers) &&
                        controllers.length > 0 ? (
                            <ul className="controller-list">
                                {controllers.map((item, index) => (
                                    <li key={index}>
                                        {String(item.name)}
                                        <code>{String(item.state)}</code>
                                    </li>
                                ))}
                            </ul>
                        ) : (
                            <div className="empty-state">
                                <span>⌁</span>Lista pojawi się po uruchomieniu
                                ros2_control.
                            </div>
                        )}
                    </Card>
                    <Card
                        title="Napędy · 6 osi"
                        index="06"
                        component={driveComponent}
                        stale={!drivesFresh}
                        statusText={drivesFresh ? `${driveCount}/6 dostępnych` : undefined}
                    >
                        <div
                            className="joints"
                            aria-label="Sześć osi robota"
                        >
                            {[1, 2, 3, 4, 5, 6].map((n) => (
                                <span key={n} data-present={presence[n - 1] === null ? "unknown" : String(presence[n - 1])} title={presence[n - 1] === null ? "Brak aktualnych danych" : presence[n - 1] ? "Napęd odpowiada" : "Napęd niedostępny"}>
                                    J{n}
                                    <i />
                                </span>
                            ))}
                        </div>
                        <p className="muted-note">
                            Pełny robot wymaga konfiguracji osi. <a href="#manual">ENABLE, zero i JOG silników →</a>
                        </p>
                    </Card>
                </section>
                <section className="architecture">
                    <div className="section-title">
                        <h2>Przepływ diagnostyki</h2>
                        <span>02 — ARCHITEKTURA</span>
                    </div>
                    <div className="flow">
                        <div>
                            <span>01</span>
                            <strong>System Linux</strong>
                            <small>CPU · RAM · CAN</small>
                        </div>
                        <b aria-hidden="true">→</b>
                        <div>
                            <span>02</span>
                            <strong>Monitor ROS 2</strong>
                            <small>Pomiary co sekundę</small>
                        </div>
                        <b aria-hidden="true">→</b>
                        <div>
                            <span>03</span>
                            <strong>Backend API</strong>
                            <small>Python · WebSocket</small>
                        </div>
                        <b aria-hidden="true">→</b>
                        <div>
                            <span>04</span>
                            <strong>Przeglądarka</strong>
                            <small>Panel w sieci LAN</small>
                        </div>
                    </div>
                    <p>
                        Diagnostyka działa niezależnie od przyszłej pętli
                        sterowania ruchem.
                    </p>
                </section>
                <footer>
                    <span>
                        ROBOT 6-DOF <span className="divider">/</span> Panel
                        diagnostyczny
                    </span>
                    <span>
                        {snapshot?.network.subnet ?? "LAN"} · Bez sterowania
                        napędami
                    </span>
                </footer>
                </>}
        </AppShell>
    );
}

createRoot(document.getElementById("root")!).render(<App />);
