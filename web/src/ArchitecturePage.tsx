import { useEffect, useMemo, useState } from "react";
import { buildInfo } from "./buildInfo";
import type { CanSnapshot } from "./canStatus";
import type { Snapshot } from "./status";
import { architectureMeters, architectureRows, formatBytes, formatDuration, groups, isOverview, isProblem,
    type ArchitectureInput, type Group, type Overview, type Row, type RowState } from "./architectureData";
import { driveCells, pulseLabels, seconds, worst, type PulseSection, type PulseState } from "./systemPulse";

const stateText: Record<RowState, string> = { ok: "OK", warn: "UWAGA", stale: "BRAK", error: "BŁĄD", off: "OFF" };
const stateMark: Record<RowState, string> = { ok: "●", warn: "▲", stale: "○", error: "■", off: "○" };
type Filter = "ALL" | Group | "PROBLEMS";
const filters: { id: Filter; label: string }[] = [{ id: "ALL", label: "Wszystkie" }, ...groups, { id: "PROBLEMS", label: "Tylko problemy" }];
const cell = (value: number | null | undefined, digits = 1) => value === null || value === undefined ? "—" : value.toLocaleString("pl-PL", { maximumFractionDigits: digits });

// Przegląd procesów co 2 s tylko na tej stronie; endpoint jest tylko do odczytu.
function useOverview() {
    const [overview, setOverview] = useState<Overview | null>(null);
    const [error, setError] = useState<string | null>(null);
    useEffect(() => {
        let stopped = false, timer: ReturnType<typeof setTimeout>;
        const load = async () => {
            try {
                const response = await fetch("/api/v1/system/overview", { cache: "no-store" });
                if (response.status === 404) throw new Error("Backend bez przeglądu systemu — wymaga restartu usługi");
                if (!response.ok) throw new Error(`Przegląd systemu niedostępny (HTTP ${response.status})`);
                const value: unknown = await response.json();
                if (!isOverview(value)) throw new Error("Nieprawidłowy format przeglądu systemu");
                if (!stopped) { setOverview(value); setError(null); }
            } catch (reason) {
                if (!stopped) { setOverview(null); setError(reason instanceof Error ? reason.message : "Przegląd systemu niedostępny"); }
            }
            if (!stopped) timer = setTimeout(load, 2000);
        };
        void load();
        return () => { stopped = true; clearTimeout(timer); };
    }, []);
    return { overview, error };
}

export function ArchitecturePage({ status, statusFresh, statusOnline, can, canFresh, canOnline, sections }: {
    status: Snapshot | null; statusFresh: boolean; statusOnline: boolean;
    can: CanSnapshot | null; canFresh: boolean; canOnline: boolean; sections: PulseSection[];
}) {
    const { overview, error } = useOverview();
    const input: ArchitectureInput = { status, statusFresh, statusOnline, can, canFresh, canOnline, overview, overviewError: error, build: buildInfo };
    const rows = useMemo(() => architectureRows(input), [status, statusFresh, statusOnline, can, canFresh, canOnline, overview, error]);
    const meters = architectureMeters(input);
    const drives = driveCells(can, canFresh);
    const problems = rows.filter(isProblem);
    const [filter, setFilter] = useState<Filter>("ALL");
    const [selected, setSelected] = useState<string | null>(null);
    const visible = rows.filter(row => filter === "ALL" || (filter === "PROBLEMS" ? isProblem(row) : row.group === filter));
    const detail = rows.find(row => row.id === selected) ?? problems[0] ?? rows[0];
    const section = Object.fromEntries(sections.map(item => [item.id, item.state])) as Record<PulseSection["id"], PulseState>;
    const drivesState = worst(drives.map(item => item.state));
    const online = drives.filter(item => item.state === "ok" || item.state === "warn").length;
    const host = status?.components.host.values ?? {};
    const chain: [string, PulseState][] = [["Frontend", section.WEB], [`Backend :${status?.network.port ?? "—"}`, section.API], ["ROS 2", section.ROS],
        [String(status?.components.can.values.interface ?? "can0"), section.SYS], [`MKS ${online}/6`, drivesState]];
    return <section className="architecture-screen arch-console" aria-label="Architektura">
        <header className="page-title"><div><div className="eyebrow">TOPOLOGIA SYSTEMU</div><h1>Architektura</h1>
            <p>Procesy, połączenia i bieżące problemy czterech warstw systemu. Widok tylko do odczytu.</p></div>
            <div className="arch-chain" aria-label="Przepływ danych">{chain.map(([label, state], index) => <span key={label} className="arch-chain-step">
                {index > 0 && <span className={`arch-link ${chain[index - 1][1]} ${state}`} aria-hidden="true" />}
                <span className={`arch-node ${state}`} title={pulseLabels[state]}><i />{label}</span>
            </span>)}</div>
        </header>

        <div className="arch-strip">
            <span className={`arch-live ${statusFresh ? "on" : ""}`}><i />{statusFresh ? "NA ŻYWO" : "BRAK DANYCH"}</span>
            <span>host <b>{overview?.host.os ?? "—"} · {overview?.host.kernel ?? "—"}</b></span>
            <span>czas pracy <b>{formatDuration(typeof host.uptime_seconds === "number" ? host.uptime_seconds : null)}</b></span>
            <span>ros <b>{overview?.ros.distro ?? "—"} · {overview?.ros.overlay ?? "—"}</b></span>
            <span>api <b>:{status?.network.port ?? "—"}</b></span>
            <span>web <b>{buildInfo.revision}</b></span>
            <span>snapshot <b>{status?.age_seconds === null || status?.age_seconds === undefined ? "—" : `${seconds(status.age_seconds)} s`}</b></span>
            <span>problemy <b className={problems.length ? "warn" : "ok"}>{problems.length}</b></span>
        </div>

        <div className="arch-meters">{meters.map(meter => <div className="arch-meter" key={meter.label}>
            <label>{meter.label}</label>
            <div className="arch-track"><i className={meter.limit !== undefined && meter.value !== null && meter.value > meter.limit ? "warn" : meter.label === "RTT" && meter.value !== null && meter.value > meter.max * .8 ? "warn" : ""}
                style={{ width: `${meter.value === null ? 0 : Math.min(100, meter.value / meter.max * 100)}%` }} />
                {meter.limit !== undefined && <s style={{ left: `${meter.limit}%` }} title={`Limit ${meter.limit}%`} />}</div>
            <output>{meter.text}</output>
        </div>)}</div>

        <div className="arch-split">
            <section className="arch-box">
                <div className="arch-tabs" role="tablist" aria-label="Filtr sekcji">{filters.map(item => <button key={item.id} type="button" role="tab"
                    aria-selected={filter === item.id} className={filter === item.id ? "on" : ""} onClick={() => setFilter(item.id)}>{item.label}</button>)}</div>
                <div className="arch-scroll"><table className="arch-table">
                    <thead><tr><th>STAN</th><th>NAZWA</th><th>TYP</th><th className="r">PID</th><th className="r">CPU%</th><th className="r">RAM</th><th className="r">CZAS PRACY</th><th>KOMUNIKAT</th></tr></thead>
                    <tbody>{groups.map(group => {
                        const items = visible.filter(row => row.group === group.id);
                        return items.length ? [<tr className="arch-group" key={group.id}><td colSpan={8}>▾ {group.label.toUpperCase()}</td></tr>,
                            ...items.map(row => <ArchRow key={row.id} row={row} selected={detail?.id === row.id} onSelect={() => setSelected(row.id)} />)] : null;
                    })}</tbody>
                </table>{!visible.length && <p className="arch-empty">Brak pozycji w tym filtrze.</p>}</div>
            </section>

            <div className="arch-side">
                <section className="arch-box">
                    <div className="arch-box-title"><span>AKTYWNE PROBLEMY</span><b className={problems.length ? "warn" : "ok"}>{problems.length}</b></div>
                    <div className="arch-log">{problems.length ? problems.map(row => <button type="button" key={row.id} onClick={() => setSelected(row.id)}>
                        <span className={row.state}>{stateText[row.state]}</span><span>{groups.find(item => item.id === row.group)!.label} · {row.name}: {row.message}</span>
                    </button>) : <p className="ok">Brak aktywnych problemów.</p>}</div>
                </section>
                <section className="arch-box">
                    <div className="arch-box-title"><span>NAPĘDY MKS · {String(can?.interface ?? "can0")}</span><b className={drivesState}>{online}/6</b></div>
                    <table className="arch-drives">
                        <thead><tr><th>OŚ</th><th className="r">ID</th><th>ON</th><th className="r">EN</th><th className="r">RPM</th><th className="r">RTT</th><th className="r">WIEK</th></tr></thead>
                        <tbody>{drives.map(item => {
                            const device = canFresh ? can?.devices.find(entry => entry.id === item.axis) : undefined;
                            const v = device?.values ?? {};
                            return <tr key={item.axis} title={item.text}>
                                <td className={item.state === "ok" ? "" : item.state}>J{item.axis}</td><td className="r">{item.axis}</td>
                                <td className={item.state} aria-label={pulseLabels[item.state]}>{stateMark[item.state]}</td>
                                <td className="r">{v.enable === true ? "1" : v.enable === false ? "0" : "—"}</td>
                                <td className="r">{typeof v.reported_speed_rpm === "number" ? v.reported_speed_rpm : "—"}</td>
                                <td className="r">{typeof v.rtt_ms === "number" ? `${cell(v.rtt_ms)} ms` : "—"}</td>
                                <td className="r">{typeof v.last_ok_age_seconds === "number" ? `${Math.round(v.last_ok_age_seconds * 1000)} ms` : "—"}</td>
                            </tr>;
                        })}</tbody>
                    </table>
                </section>
                {detail && <section className="arch-box">
                    <div className="arch-box-title"><span>SZCZEGÓŁY · {detail.name}</span><b>{detail.pid ? `PID ${detail.pid}` : detail.type}</b></div>
                    <dl className="arch-details">
                        <dt>stan</dt><dd className={detail.state}>{stateText[detail.state]}</dd>
                        <dt>komunikat</dt><dd>{detail.message}</dd>
                        {detail.details.map(([key, value]) => value ? [<dt key={`k${key}`}>{key}</dt>, <dd key={`v${key}`}>{value}</dd>] : <dd key={key} className="wide">{key}</dd>)}
                    </dl>
                </section>}
            </div>
        </div>
        <p className="arch-foot">Widok tylko do odczytu · bez poleceń do napędów · przegląd procesów odświeżany co 2 s{error ? ` · ${error}` : ""}</p>
    </section>;
}

function ArchRow({ row, selected, onSelect }: { row: Row; selected: boolean; onSelect: () => void }) {
    return <tr className={selected ? "sel" : ""} onClick={onSelect} tabIndex={0} aria-selected={selected}
        onKeyDown={event => { if (event.key === "Enter" || event.key === " ") { event.preventDefault(); onSelect(); } }}>
        <td className={row.state}>{stateMark[row.state]} {stateText[row.state]}</td><td>{row.name}</td><td>{row.type}</td>
        <td className="r">{row.pid ?? "—"}</td><td className="r">{cell(row.cpu)}</td><td className="r">{formatBytes(row.ram)}</td>
        <td className="r">{formatDuration(row.uptime)}</td><td className="arch-msg">{row.message}</td>
    </tr>;
}
