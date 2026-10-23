import { useEffect, useState, type ReactNode } from "react";

export const pages = [
    { id: "overview", label: "Przegląd systemu", icon: "◫" },
    { id: "can", label: "Magistrala CAN", icon: "⇄" },
    { id: "diagnostics", label: "Diagnostyka", icon: "∿" },
    { id: "manual", label: "Sterowanie ręczne", icon: "↔" },
    { id: "config", label: "Konfiguracja", icon: "⚙" },
    { id: "architecture", label: "Architektura", icon: "⌘" },
] as const;
export type PageId = typeof pages[number]["id"];
export function pageFromHash(hash: string): PageId {
    if (hash === "#configuration") return "config";
    return pages.find(page => `#${page.id}` === hash)?.id ?? "overview";
}
export type ManualHeaderState = { connected: boolean; fresh: boolean; planning: boolean; recording: boolean };
export const emptyManualHeader: ManualHeaderState = { connected: false, fresh: false, planning: false, recording: false };

export function AppShell({ view, manual, online, connecting, children }: {
    view: PageId; manual: ManualHeaderState; online: boolean; connecting: boolean; children: ReactNode;
}) {
    const [collapsed, setCollapsed] = useState(view === "manual");
    useEffect(() => { setCollapsed(view === "manual"); }, [view]);
    useEffect(() => {
        // Po zatwierdzeniu układu scena odczytuje aktualny rozmiar kanwy.
        const frame = requestAnimationFrame(() => window.dispatchEvent(new Event("resize")));
        return () => cancelAnimationFrame(frame);
    }, [collapsed, view]);
    const isManual = view === "manual", connected = isManual ? manual.connected : online;
    const fresh = isManual ? manual.fresh : online;
    const connectionText = isManual ? !connected ? "Brak połączenia" : fresh ? "Połączono · 20 Hz" : "Połączono · dane nieaktualne" :
        online ? "Połączono z backendem" : connecting ? "Łączenie z backendem" : "Backend niedostępny";
    return <div className={`shell app-shell ${collapsed ? "sidebar-collapsed" : "sidebar-expanded"} ${isManual ? "is-manual" : ""}`}>
        <aside className="sidebar" aria-label="Nawigacja aplikacji">
            <a className="brand" href="#overview" title="ROBOT 6-DOF / STANOWISKO" aria-label="ROBOT 6-DOF / STANOWISKO">
                <img src="/robot.svg" alt="" /><span className="sidebar-label">ROBOT<span className="brand-detail">6-DOF / STANOWISKO</span></span>
            </a>
            <div className="nav-label sidebar-label">PRZESTRZEŃ ROBOCZA</div>
            <nav aria-label="Sekcje panelu">{pages.map(page => <a key={page.id} href={`#${page.id}`} title={page.label}
                aria-label={page.label} aria-current={view === page.id ? "page" : undefined} className={view === page.id ? "active selected" : ""}>
                <span className="nav-icon" aria-hidden="true">{page.icon}</span><span className="sidebar-label">{page.label}</span>
            </a>)}</nav>
            <div className="sidebar-bottom"><div className="sidebar-label"><span className="dot" /> PANEL ROBOTA<p>Dostęp lokalny · CAN</p></div>
                <button type="button" className="sidebar-toggle" aria-label={collapsed ? "Rozwiń menu" : "Zwiń menu"}
                    title={collapsed ? "Rozwiń menu" : "Zwiń menu"} aria-expanded={!collapsed} onClick={() => setCollapsed(value => !value)}>{collapsed ? "»" : "«"}</button>
            </div>
        </aside>
        <main className="app-workspace">
            <header className="topbar app-topbar"><div className="breadcrumbs">Stanowisko robota <span className="divider">/</span><span>{pages.find(page => page.id === view)!.label}</span></div>
                <div className="topbar-statuses" role="status" aria-live="polite">
                    {isManual && manual.planning && <span className="badge planning-badge"><span className="dot" />PLANOWANIE · bliźniak</span>}
                    {isManual && manual.recording && <span className="badge warn">Nagrywanie diagnostyki</span>}
                    <span className={`badge ${connected && fresh ? "ok" : "stale"}`}><span className="dot" />{connectionText}</span>
                    <span className="local-tag">SIEĆ LOKALNA</span>
                </div>
            </header>
            <div className="app-content">{children}</div>
        </main>
    </div>;
}
