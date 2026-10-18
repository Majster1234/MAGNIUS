import { createRoot } from "react-dom/client";
import { dataIsStale, numeric, type Component, type State } from "./status";
import { useStatus } from "./useStatus";
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
}: {
    title: string;
    index: string;
    component?: Component;
    stale: boolean;
    children?: React.ReactNode;
}) {
    const state = stale || !component ? "stale" : component.state;
    return (
        <article className="card">
            <div className="card-heading">
                <span className="index">{index}</span>
                <h3>{title}</h3>
                <Badge
                    state={state}
                    text={
                        component?.id === "controllers" &&
                        state === "not_configured"
                            ? "Nieuruchomione"
                            : undefined
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
function App() {
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
    return (
        <div className="shell">
            <aside className="sidebar">
                <a className="brand" href="#overview">
                    <img src="/robot.svg" alt="" />
                    <span>
                        ROBOT
                        <span className="brand-detail">6-DOF / STANOWISKO</span>
                    </span>
                </a>
                <div className="nav-label">PRZESTRZEŃ ROBOCZA</div>
                <nav aria-label="Sekcje panelu">
                    <a className="selected" href="#overview">
                        <span>◫</span>Przegląd systemu
                    </a>
                    <a href="#can">
                        <span>⇄</span>Magistrala CAN
                    </a>
                    <a href="#architecture">
                        <span>⌘</span>Architektura
                    </a>
                </nav>
                <div className="sidebar-bottom">
                    <span className="dot" />
                    PANEL DIAGNOSTYCZNY<p>Dostęp lokalny · Tylko odczyt</p>
                </div>
            </aside>
            <main id="overview">
                <header className="topbar">
                    <span>
                        Stanowisko robota <span className="divider">/</span>{" "}
                        Diagnostyka
                    </span>
                    <span className="local-tag">SIEĆ LOKALNA</span>
                </header>
                <div className="page-title">
                    <div>
                        <div className="eyebrow">MONITOROWANIE SYSTEMU</div>
                        <h1>Przegląd systemu</h1>
                        <p>Stan połączeń i podzespołów w jednym miejscu.</p>
                    </div>
                    <div
                        className="connection"
                        role="status"
                        aria-live="polite"
                    >
                        <Badge
                            state={online ? "ok" : "stale"}
                            text={
                                online
                                    ? "Połączono z backendem"
                                    : connection === "connecting"
                                      ? "Łączenie z backendem"
                                      : "Backend niedostępny"
                            }
                        />
                        <span>Odświeżanie co 1 s</span>
                    </div>
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
                    <div id="can" className="card-anchor">
                        <Card
                            title="Magistrala CAN"
                            index="04"
                            component={c?.can}
                            stale={stale}
                        >
                            <div className="detail-row">
                                <span>Interfejs / stan</span>
                                <code>
                                    {String(can?.interface ?? "—")} /{" "}
                                    {String(can?.bus_state ?? "—")}
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
                        component={c?.drives}
                        stale={stale}
                    >
                        <div
                            className="joints"
                            aria-label="Sześć osi bez integracji"
                        >
                            {[1, 2, 3, 4, 5, 6].map((n) => (
                                <span key={n}>
                                    J{n}
                                    <i />
                                </span>
                            ))}
                        </div>
                        <p className="muted-note">
                            Oczekuje na sterownik MKS i konfigurację osi.
                        </p>
                    </Card>
                </section>
                <section id="architecture" className="architecture">
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
            </main>
        </div>
    );
}

createRoot(document.getElementById("root")!).render(<App />);
