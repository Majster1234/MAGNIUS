import { useEffect, useState } from "react";
import type { CanDevice, CanSnapshot } from "./canStatus";
import { numeric, type State } from "./status";
import { moduleNotice, useCondition } from "./messageCenter";

const labels: Record<State, string> = { ok: "Aktywny", warn: "Uwaga", error: "Błąd", stale: "Brak aktualnych danych", not_configured: "Brak integracji" };
export function StateBadge({ state, text }: { state: State; text?: string }) {
    return <span className={`badge ${state}`}><span className="dot" />{text ?? labels[state]}</span>;
}
function Metric({ label, value, unit = "" }: { label: string; value: unknown; unit?: string }) {
    const number = numeric(value);
    return <div className="metric"><span>{label}</span>
        <strong>{number === null ? "—" : number.toLocaleString("pl-PL", { maximumFractionDigits: 1 })}<small>{number === null ? "" : unit}</small></strong></div>;
}

/** SCAN i Reset CAN (backend blokuje je podczas aktywnego sterowania). */
export function useCanActions() {
    const [action, setAction] = useState<"scan" | "reset" | null>(null);
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
    return { action, request };
}

/** Zakładka „Magistrala CAN” w Diagnostyce: postęp skanu, podsumowanie, urządzenia i szczegóły komunikacji. */
export function CanPanel({ snapshot, connection }: { snapshot: CanSnapshot | null; connection: string }) {
    const [page, setPage] = useState(0);
    const devices = snapshot?.devices ?? [];
    const totalPages = Math.max(1, Math.ceil(devices.length / 25));
    const visible = devices.slice(page * 25, page * 25 + 25);
    const summary = snapshot?.summary.values;
    const scan = snapshot?.scan.values;
    useCondition("can-scan", scan?.error ? moduleNotice({ source: "can-scan", severity: "warning", title: "Skanowanie magistrali CAN nie powiodło się", message: String(scan.error) }, "Diagnostyka · CAN", "#can") : null);
    const stale = connection !== "online" || !snapshot?.fresh;
    useEffect(() => setPage((old) => Math.min(old, totalPages - 1)), [totalPages]);
    const value = (name: string) => summary?.[name];
    return <div className="can-panel">
        <div className={`can-progress ${stale ? "waiting" : ""}`}><StateBadge state={stale ? "stale" : snapshot?.summary.state ?? "stale"} text={snapshot?.scan.message ?? "Oczekiwanie na diagnostykę CAN"}/>
            <span>{Boolean(scan?.scanning) ? `Adres ${String(scan?.checked ?? 0)} / ${String(scan?.total ?? 20)}` : String(scan?.state ?? "—")}</span>
            <span className="progress-error">{scan?.error ? String(scan.error) : ""}</span></div>
        <div className="can-strip" aria-label="Podsumowanie magistrali"><div className="strip-state"><span>Stan</span><strong>{String(value("bus_state") ?? "—")}</strong><small>{String(value("interface") ?? "can0")}</small></div>
            <Metric label="Bitrate" value={value("bitrate")} unit=" bit/s"/><Metric label="Urządzenia" value={value("device_count")}/>
            <Metric label="Obieg odczytu" value={value("full_round_ms")} unit=" ms"/><Metric label="Błędy" value={value("errors")}/></div>
        <div className="can-split">
            <div className="pane-scroll">
                <div className="pane-section">Urządzenia bieżącej sesji · {devices.length} rekordów</div>
                {devices.length === 0 ? <div className="can-empty"><strong>Tabela jest pusta</strong><p>Skan startowy lub przycisk Skanuj sprawdza adresy 1–20. Reset zachowuje znane urządzenia i odświeża ich stan.</p></div> : <>
                    <table className="dense-table"><thead><tr><th>CAN ID</th><th>Oś</th><th>Model</th><th>Łączność</th><th>Stan napędu</th><th>ENABLE</th><th className="r">RTT [ms]</th></tr></thead>
                        <tbody>{visible.map((device) => <DeviceRow key={device.id} device={device}/>)}</tbody></table>
                    {totalPages > 1 && <div className="pagination"><button className="secondary-button" disabled={page === 0} onClick={() => setPage(page - 1)}>Poprzednie</button><span>{page + 1} / {totalPages}</span><button className="secondary-button" disabled={page + 1 >= totalPages} onClick={() => setPage(page + 1)}>Następne</button></div>}</>}
            </div>
            <div className="pane-scroll">
                <div className="pane-section">Szczegóły komunikacji</div>
                <div className="can-metrics">
                    <Metric label="Planowane obciążenie" value={value("scheduled_utilization_percent")} unit="%" />
                    <Metric label="Limit planowany" value={value("planned_utilization_limit_percent")} unit="%" />
                    <Metric label="Cykl komend" value={value("command_period_ms")} unit=" ms" />
                    <Metric label="Okres enkoderów" value={value("encoder_period_ms")} unit=" ms" />
                    {value("motion_use_ack") === false ? <>
                        <div className="metric"><span>Tryb ruchu</span><strong>Streaming bez ACK</strong></div>
                        <Metric label="Wyprzedzenie pozycji" value={value("streaming_lookahead_ms")} unit=" ms" />
                    </> : <>
                        <Metric label="Margines ACK do terminu" value={value("ack_margin_min_ms")} unit=" ms" />
                        <Metric label="Spóźnienie SYNC / maksimum" value={value("sync_lateness_max_ms")} unit=" ms" />
                    </>}
                    <div className="metric"><span>STOP F7 → F6(0)</span><strong>{typeof value("stop_result") === "string" ? String(value("stop_result")) : "—"}</strong></div>
                    <Metric label="Jitter cyklu" value={value("cycle_jitter_ms")} unit=" ms" />
                    <Metric label="Odstęp między ramkami" value={value("rx_interval_mean_ms")} unit=" ms" />
                    <Metric label="Ramki RX / sesja" value={value("rx_frames")} />
                    <Metric label="Ramki TX / sesja" value={value("tx_frames")} />
                </div>
                <p className="muted-note">Obciążenie jest oszacowaniem kosztu zapytań i odpowiedzi; nie obejmuje retransmisji. Aktualizacja 5 Hz.</p>
            </div>
        </div>
    </div>;
}

function DeviceRow({ device }: { device: CanDevice }) {
    const [open, setOpen] = useState(false);
    const v = device.values;
    const enable = v.enable === true ? "Włączony" : v.enable === false ? "Wyłączony" : "—";
    return <>
        <tr className="device-row">
            <td><button className="link-button" aria-expanded={open} onClick={() => setOpen(!open)}>{open ? "▾" : "▸"} {device.id}</button></td>
            <td>{device.id >= 1 && device.id <= 6 ? <span className={`axis-chip j${device.id}`}>J{device.id}</span> : "—"}</td>
            <td>{String(v.model ?? "—")}</td>
            <td><StateBadge state={device.state} text={device.message} /></td>
            <td>{String(v.motor_state ?? "Niedostępny")}</td>
            <td><span className={`badge ${v.enable === true ? "ok" : "stale"}`}>{enable}</span></td>
            <td className="r">{numeric(v.rtt_ms)?.toLocaleString("pl-PL", { maximumFractionDigits: 2 }) ?? "—"}</td>
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
