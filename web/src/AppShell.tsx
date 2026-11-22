import { useEffect, useState, type ReactNode } from "react";
import { DriveStartupDialog } from "./DriveStartupDialog";
import { Icon, type IconName } from "./Icon";
import { MessageBar } from "./MessageBar";
import { NavStatus } from "./NavStatus";
import { PendantLink } from "./PendantLink";

export const pages = [
    { id: "overview", label: "Przegląd systemu", icon: "overview" },
    { id: "diagnostics", label: "Diagnostyka", icon: "diag" },
    { id: "manual", label: "Sterowanie ręczne", icon: "manual" },
    { id: "programming", label: "Programowanie", icon: "program" },
    { id: "config", label: "Konfiguracja", icon: "config" },
] as const satisfies readonly { id: string; label: string; icon: IconName }[];
export type PageId = typeof pages[number]["id"];
// Dawne moduły Magistrala CAN i Architektura są zakładkami Diagnostyki.
export function pageFromHash(hash: string): PageId {
    if (hash === "#configuration") return "config";
    if (hash === "#can" || hash === "#architecture") return "diagnostics";
    return pages.find(page => `#${page.id}` === hash)?.id ?? "overview";
}
export type ManualHeaderState = { connected: boolean; fresh: boolean; planning: boolean; recording: boolean };
export const emptyManualHeader: ManualHeaderState = { connected: false, fresh: false, planning: false, recording: false };

const COLLAPSED_KEY = "robot-nav-collapsed-v1";
const NAV_ANIMATION_MS = 400;   // nieco dłużej niż --nav-dur w theme.css
function storedCollapsed() {
    try { return localStorage.getItem(COLLAPSED_KEY) === "1"; } catch { return false; }
}

export function AppShell({ view, manual, online, connecting, status, children, serverUrl, subnet }: {
    view: PageId; manual: ManualHeaderState; online: boolean; connecting: boolean; children: ReactNode;
    status?: Omit<React.ComponentProps<typeof NavStatus>, "collapsed">;
    serverUrl?: string; subnet?: string;   // adres panelu w LAN (/api/v1/status network) — kod QR teach pendanta
}) {
    // Stan rozwinięcia menu zmienia tylko operator; wybór strony go nie zmienia (pamiętany w tej przeglądarce).
    const [collapsed, setCollapsed] = useState(storedCollapsed);
    useEffect(() => { try { localStorage.setItem(COLLAPSED_KEY, collapsed ? "1" : "0"); } catch { /* Menu działa bez localStorage. */ } }, [collapsed]);
    useEffect(() => {
        // Menu zmienia szerokość płynnie (theme.css, ok. 0,34 s): przez czas animacji scena 3D i wykresy
        // odczytują aktualny rozmiar w każdej klatce; po zmianie strony wystarcza jedno zdarzenie.
        const until = performance.now() + NAV_ANIMATION_MS;
        let frame = requestAnimationFrame(function tick(now) {
            window.dispatchEvent(new Event("resize"));
            if (now < until) frame = requestAnimationFrame(tick);
        });
        return () => cancelAnimationFrame(frame);
    }, [collapsed, view]);
    const isManual = view === "manual", connected = isManual ? manual.connected : online;
    const fresh = isManual ? manual.fresh : online;
    const connectionText = isManual ? !connected ? "Brak połączenia" : fresh ? "Połączono · 20 Hz" : "Połączono · dane nieaktualne" :
        online ? "Połączono z backendem" : connecting ? "Łączenie z backendem" : "Backend niedostępny";
    return <div className={`shell app-shell ${collapsed ? "sidebar-collapsed" : "sidebar-expanded"} ${isManual ? "is-manual" : ""} ${view === "programming" ? "is-programming" : ""}`}>
        <aside className="sidebar" aria-label="Nawigacja aplikacji">
            <a className="brand" href="#overview" title="ROBOT 6-DOF / STANOWISKO" aria-label="ROBOT 6-DOF / STANOWISKO">
                <Icon name="robot" /><span className="brand-text">ROBOT<small>6-DOF · stanowisko</small></span>
            </a>
            <nav aria-label="Sekcje panelu">{pages.map(page => <a key={page.id} href={`#${page.id}`} title={page.label}
                aria-label={page.label} aria-current={view === page.id ? "page" : undefined} className={view === page.id ? "active selected" : ""}>
                <Icon name={page.icon} /><span className="nav-text">{page.label}</span>
            </a>)}</nav>
            {status && <NavStatus {...status} collapsed={collapsed} />}
            <button type="button" className="sidebar-toggle" aria-label={collapsed ? "Rozwiń menu" : "Zwiń menu"}
                title={collapsed ? "Rozwiń menu" : "Zwiń menu"} aria-expanded={!collapsed} onClick={() => setCollapsed(value => !value)}>
                <Icon name="collapse" /><span className="nav-text">Zwiń menu</span></button>
        </aside>
        <main className="app-workspace">
            <MessageBar />
            <div className="app-content">{children}</div>
            <footer className="app-statusbar">
                <div className="breadcrumbs">Stanowisko robota <span className="divider">/</span><span>{pages.find(page => page.id === view)!.label}</span></div>
                <div className="topbar-statuses" role="status" aria-live="polite">
                    {isManual && manual.planning && <span className="badge planning-badge"><span className="dot" />PLANOWANIE · bliźniak</span>}
                    {isManual && manual.recording && <span className="badge planning-badge"><span className="dot" />Nagrywanie diagnostyki</span>}
                    <span className={`badge ${connected && fresh ? "ok" : "stale"}`}><span className="dot" />{connectionText}</span>
                    <PendantLink baseUrl={serverUrl ?? location.origin} subnet={subnet} />
                </div>
            </footer>
        </main>
        <DriveStartupDialog />
    </div>;
}
