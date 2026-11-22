import { useEffect, useRef, useState } from "react";
import { formatBytes, formatDuration, groups, isOverview, isProblem, type Group, type Overview, type Row, type RowState } from "./architectureData";

export const stateText: Record<RowState, string> = { ok: "OK", warn: "UWAGA", stale: "BRAK", error: "BŁĄD", off: "OFF" };
export const rowBadge = (state: RowState) => state === "off" ? "stale" : state;
type Filter = "ALL" | Group;
const filters: { id: Filter; label: string }[] = [{ id: "ALL", label: "Wszystkie" }, ...groups];
const cell = (value: number | null | undefined, digits = 1) => value === null || value === undefined ? "—" : value.toLocaleString("pl-PL", { maximumFractionDigits: digits });

// Przegląd procesów co 2 s, gdy aktywna jest Diagnostyka; endpoint jest tylko do odczytu.
export function useOverview(enabled: boolean) {
    const [overview, setOverview] = useState<Overview | null>(null);
    const [error, setError] = useState<string | null>(null);
    useEffect(() => {
        if (!enabled) return;
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
    }, [enabled]);
    return { overview, error };
}

/** Zakładka „Procesy”: tabela czterech warstw systemu (filtr grup) i szczegóły wybranego wiersza. */
export function ProcessPanel({ rows, selected, onSelect, error }: {
    rows: Row[]; selected: string | null; onSelect: (id: string) => void; error: string | null;
}) {
    const [filter, setFilter] = useState<Filter>("ALL");
    const visible = rows.filter(row => filter === "ALL" || row.group === filter);
    const detail = rows.find(row => row.id === selected) ?? rows.find(isProblem) ?? rows[0];
    const selectedRow = useRef<HTMLTableRowElement>(null);
    useEffect(() => { selectedRow.current?.scrollIntoView({ block: "nearest" }); }, [selected]);
    return <div className="process-panel">
        <div className="pane-scroll">
            <div className="filter-tabs" role="tablist" aria-label="Filtr warstw">{filters.map(item => <button key={item.id} type="button" role="tab"
                aria-selected={filter === item.id} className={filter === item.id ? "on" : ""} onClick={() => setFilter(item.id)}>{item.label}</button>)}</div>
            <table className="dense-table">
                <thead><tr><th>Stan</th><th>Nazwa</th><th>Typ</th><th className="r">PID</th><th className="r">CPU %</th><th className="r">RAM</th><th className="r">Czas pracy</th><th>Komunikat</th></tr></thead>
                <tbody>{groups.map(group => {
                    const items = visible.filter(row => row.group === group.id);
                    return items.length ? [<tr className="group-row" key={group.id}><td colSpan={8}>{group.label}</td></tr>,
                        ...items.map(row => <tr key={row.id} ref={detail?.id === row.id ? selectedRow : undefined} className={detail?.id === row.id ? "sel" : ""}
                            onClick={() => onSelect(row.id)} tabIndex={0} aria-selected={detail?.id === row.id}
                            onKeyDown={event => { if (event.key === "Enter" || event.key === " ") { event.preventDefault(); onSelect(row.id); } }}>
                            <td><span className={`badge ${rowBadge(row.state)}`}><span className="dot" />{stateText[row.state]}</span></td>
                            <td className="mono">{row.name}</td><td>{row.type}</td>
                            <td className="r">{row.pid ?? "—"}</td><td className="r">{cell(row.cpu)}</td><td className="r">{formatBytes(row.ram)}</td>
                            <td className="r">{formatDuration(row.uptime)}</td><td className="ellipsis">{row.message}</td>
                        </tr>)] : null;
                })}</tbody>
            </table>
            {!visible.length && <p className="muted-note">Brak pozycji w tym filtrze.</p>}
            <p className="muted-note">Widok tylko do odczytu · odświeżanie co 2 s{error ? ` · ${error}` : ""}</p>
        </div>
        {detail && <div className="pane-scroll process-detail">
            <div className="pane-section">Szczegóły · {detail.name}</div>
            <dl className="detail-list">
                <dt>Stan</dt><dd><span className={`badge ${rowBadge(detail.state)}`}><span className="dot" />{stateText[detail.state]}</span></dd>
                <dt>Typ</dt><dd>{detail.pid ? `${detail.type} · PID ${detail.pid}` : detail.type}</dd>
                <dt>Komunikat</dt><dd>{detail.message}</dd>
                {detail.details.map(([key, value]) => value ? [<dt key={`k${key}`}>{key}</dt>, <dd key={`v${key}`}>{value}</dd>] : <dd key={key} className="wide mono">{key}</dd>)}
            </dl>
        </div>}
    </div>;
}

/** Zakładka „Problemy”: bieżące ostrzeżenia, awarie i brak danych we wszystkich warstwach. */
export function ProblemList({ rows, onSelect }: { rows: Row[]; onSelect: (id: string) => void }) {
    const problems = rows.filter(isProblem);
    return <div className="pane-scroll problem-list">
        {problems.length ? problems.map(row => <button type="button" key={row.id} className={`problem ${rowBadge(row.state)}`} onClick={() => onSelect(row.id)}>
            <span className={`badge ${rowBadge(row.state)}`}><span className="dot" />{stateText[row.state]}</span>
            <b>{groups.find(item => item.id === row.group)!.label} · {row.name}</b><span>{row.message}</span>
        </button>) : <p className="problem-none">Brak aktywnych problemów we wszystkich warstwach systemu.</p>}
        <p className="muted-note">Lista pokazuje stan bieżący. Historia zdarzeń wymaga rejestru problemów w backendzie (osobne zadanie).</p>
    </div>;
}
