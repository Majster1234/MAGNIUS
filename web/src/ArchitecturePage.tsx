const stages = [
    ["Przeglądarka UI", "React · TypeScript", "Widoki operatora, WebSocket i podgląd cyfrowego bliźniaka."],
    ["Backend API", "FastAPI · :8080", "Wspólny snapshot ROS, odczyt REST i strumienie WebSocket."],
    ["ROS 2 Humble", "/robot/system_status", "Monitor systemu, diagnostyka CAN i osobne usługi ręcznego sterowania."],
    ["Planowanie i sterowanie", "KDL · TOTG · ros2_control", "Planowanie PTP/LIN służy podglądowi. Fizyczne wykonanie TCP pozostaje zablokowane."],
    ["SocketCAN", "can0 · 500 kbit/s", "Linux → BIGTREETECH U2C v2.1 → protokół producenta MKS."],
    ["Napędy MKS SERVO42D CAN", "CAN ID 1–6 ↔ J1–J6", "Stałe przypisanie osi. Dostępność napędów pokazuje diagnostyka bieżącej sesji."],
];
export function ArchitecturePage() {
    return <section className="architecture-screen" aria-label="Architektura">
        <header className="page-title"><div><div className="eyebrow">TOPOLOGIA SYSTEMU</div><h1>Architektura</h1><p>Przepływ danych od interfejsu operatora do napędów robota.</p></div></header>
        <div className="architecture-flow" aria-label="Przepływ danych">UI <span>↔</span> API :8080 <span>↔</span> ROS 2 Humble <span>↔</span> SocketCAN can0 <span>↔</span> MKS CAN ID 1–6</div>
        <div className="cards architecture-cards">{stages.map(([title, detail, description], index) => <article className="card" key={title}>
            <div className="card-heading"><span className="index">{String(index + 1).padStart(2, "0")}</span><h2>{title}</h2></div>
            <code>{detail}</code><p className="description">{description}</p>
        </article>)}</div>
        <p className="architecture-note">Monitorowanie i podgląd planów działają oddzielnie od ręcznych poleceń ENABLE, zerowania i JOG. Gotowość fizycznego robota wymaga potwierdzonej konfiguracji oraz odbioru sprzętu.</p>
    </section>;
}
