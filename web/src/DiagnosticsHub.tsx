import { useMemo, useState } from "react";
import { architectureRows, groups, isProblem, type Row } from "./architectureData";
import { buildInfo } from "./buildInfo";
import { CanPanel, useCanActions } from "./CanPanel";
import type { CanSnapshot } from "./canStatus";
import { Icon, type IconName } from "./Icon";
import { PageHeader } from "./PageHeader";
import { ProblemList, ProcessPanel, rowBadge, useOverview } from "./ProcessPanel";
import { numeric, type Snapshot } from "./status";
import { driveCells } from "./systemPulse";

export type DiagnosticsTab = "trace" | "can" | "proc" | "problems";
const tabs: [DiagnosticsTab, string, IconName][] = [["trace", "Przebiegi", "diag"], ["can", "Magistrala CAN", "can"], ["proc", "Procesy", "arch"], ["problems", "Problemy", "alert"]];
const groupIcon: Record<Row["group"], IconName> = { SYSTEM: "overview", ROS: "arch", BACKEND: "config", FRONTEND: "phone" };
const ms = (value: unknown) => { const n = numeric(value); return n === null ? "—" : `${n.toLocaleString("pl-PL", { maximumFractionDigits: 2 })} ms`; };

/** Diagnostyka: stała mapa systemu (magistrala, napędy, procesy) i zakładki Przebiegi · Magistrala CAN · Procesy · Problemy. */
export function DiagnosticsHub({ status, statusFresh, statusOnline, can, canConnection, canFresh, canOnline, tab, onTab, onTraceHost, onAxis }: {
    status: Snapshot | null; statusFresh: boolean; statusOnline: boolean;
    can: CanSnapshot | null; canConnection: string; canFresh: boolean; canOnline: boolean;
    tab: DiagnosticsTab; onTab: (tab: DiagnosticsTab) => void;
    onTraceHost: (element: HTMLDivElement | null) => void; onAxis: (axis: number) => void;
}) {
    const { overview, error } = useOverview(true);
    const rows = useMemo(() => architectureRows({ status, statusFresh, statusOnline, can, canFresh, canOnline, overview, overviewError: error, build: buildInfo }),
        [status, statusFresh, statusOnline, can, canFresh, canOnline, overview, error]);
    const [selected, setSelected] = useState<string | null>(null);
    const [mapSelection, setMapSelection] = useState<string | null>(null);
    const { action, request } = useCanActions();
    const problems = rows.filter(isProblem).length;
    const okCount = rows.filter(row => row.state === "ok" || row.state === "off").length;
    const drives = driveCells(can, canFresh);
    const summary = canFresh ? can?.summary.values ?? {} : {};
    const link = rows.find(row => row.id === "can-link");
    const selectRow = (id: string) => { setSelected(id); setMapSelection(id); onTab("proc"); };
    const node = (key: string, state: string, label: React.ReactNode, meta: React.ReactNode, onClick: () => void, title?: string) =>
        <button key={key} type="button" className={`map-node${mapSelection === key ? " sel" : ""}`} title={title} onClick={() => { setMapSelection(key); onClick(); }}>
            <i className={`map-dot ${state}`} /><span className="map-label">{label}</span><span className="map-meta">{meta}</span></button>;
    return <section className="diagnostics-hub" aria-label="Diagnostyka">
        <PageHeader icon="diag" title="Diagnostyka"
            info="Przebiegi wałów, magistrala CAN i procesy systemu (dawniej osobne moduły Magistrala CAN i Architektura). Kliknięcie elementu na mapie otwiera jego szczegóły. Bez poleceń ruchu; SCAN i Reset CAN są blokowane podczas aktywnego sterowania."
            state={<span className={`badge ${problems ? "warn" : "ok"}`}><span className="dot" />Problemy: {problems}</span>}
            actions={<>
                <button className="secondary-button" disabled={action !== null} onClick={() => void request("reset")}><Icon name="power" />{action === "reset" ? "Resetowanie…" : "Reset CAN"}</button>
                <button className="primary-button" disabled={action !== null || Boolean(can?.scan.values.scanning)} onClick={() => void request("scan")}><Icon name="scan" />{action === "scan" ? "Uruchamianie…" : "Skanuj"}</button>
            </>} />
        <div className="hub-body">
            <aside className="hub-map panel" aria-label="Mapa systemu">
                <div className="panel-head"><h2>Mapa systemu</h2><span className={`badge ${problems ? "warn" : "ok"}`}><span className="dot" />{okCount}/{rows.length} OK</span></div>
                <div className="map-scroll">
                    <div className="map-group"><Icon name="can" />Magistrala {String(status?.components.can.values.interface ?? "can0")}<span className="map-meta">RTT maks. {ms(summary.rtt_max_ms)}</span></div>
                    {link && node("can-link", rowBadge(link.state), link.name, String(status?.components.can.values.bus_state ?? "—"), () => onTab("can"), link.message)}
                    {drives.map(cell => {
                        const device = canFresh ? can?.devices.find(item => item.id === cell.axis) : undefined;
                        return node(`axis:${cell.axis}`, cell.state, <><span className={`axis-chip j${cell.axis}`}>J{cell.axis}</span> {String(device?.values.model ?? "napęd")}</>,
                            ms(device?.values.rtt_ms), () => { onTab("trace"); onAxis(cell.axis); }, cell.text);
                    })}
                    {groups.map(group => <div key={group.id}>
                        <div className="map-group"><Icon name={groupIcon[group.id]} />{group.label}</div>
                        {rows.filter(row => row.group === group.id && row.id !== "can-link").map(row =>
                            node(row.id, rowBadge(row.state), row.name, row.cpu === null || row.cpu === undefined ? "" : `${row.cpu.toLocaleString("pl-PL", { maximumFractionDigits: 1 })} %`,
                                () => selectRow(row.id), row.message))}
                    </div>)}
                </div>
            </aside>
            <div className="hub-main panel">
                <div className="hub-tabs" role="tablist" aria-label="Szczegóły diagnostyki">{tabs.map(([id, label, icon]) => <button key={id} type="button" role="tab"
                    aria-selected={tab === id} className={tab === id ? "on" : ""} onClick={() => onTab(id)}><Icon name={icon} />{label}
                    {id === "problems" && <span className="count">{problems}</span>}</button>)}</div>
                <div className="hub-pane" ref={onTraceHost} hidden={tab !== "trace"} />
                {tab === "can" && <div className="hub-pane"><CanPanel snapshot={can} connection={canConnection} /></div>}
                {tab === "proc" && <div className="hub-pane"><ProcessPanel rows={rows} selected={selected} onSelect={id => { setSelected(id); setMapSelection(id); }} error={error} /></div>}
                {tab === "problems" && <div className="hub-pane"><ProblemList rows={rows} onSelect={selectRow} /></div>}
            </div>
        </div>
    </section>;
}
