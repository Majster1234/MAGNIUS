import { useCallback, useEffect, useRef, useState } from "react";
import { applyErrors, axisList, differenceRows, type DifferenceRow, type DriveStartupState } from "./driveStartup";
import { moduleNotice, useCondition } from "./messageCenter";

const POLL_MS = 2000, POLL_ACTIVE_MS = 500;

function Differences({ rows }: { rows: DifferenceRow[] }) {
    return <div className="drive-startup-table"><table>
        <thead><tr><th>Oś</th><th>Nastawa</th><th>Zapamiętana</th><th>W napędzie</th></tr></thead>
        <tbody>{rows.map(row => <tr key={row.key}><td>{row.axis}</td><td>{row.label}</td><td>{row.expected}</td><td>{row.actual}</td></tr>)}</tbody>
    </table></div>;
}

// Okno widoczne na każdej stronie panelu, gdy sprawdzenie nastaw po starcie napędów wymaga decyzji operatora.
export function DriveStartupDialog() {
    const [state, setState] = useState<DriveStartupState | null>(null);
    const [sending, setSending] = useState(false);
    const [error, setError] = useState("");
    const alive = useRef(true);
    const refresh = useCallback(async () => {
        try {
            const response = await fetch("/api/v1/drive-startup", { cache: "no-store" });
            if (!response.ok) { if (alive.current) setState(null); return; }
            const data = await response.json() as DriveStartupState;
            if (alive.current) setState(data);
        } catch { /* Brak połączenia pokazuje pasek stanu; okno wróci po kolejnym odczycie. */ }
    }, []);
    const busy = state?.phase === "applying" || state?.phase === "checking";
    useEffect(() => {
        alive.current = true; void refresh();
        const timer = window.setInterval(() => void refresh(), busy ? POLL_ACTIVE_MS : POLL_MS);
        return () => { alive.current = false; window.clearInterval(timer); };
    }, [refresh, busy]);
    // Stany wymagające działania także w pasku komunikatów (okno decyzji zostaje).
    const failedAxes = state?.failed_axes?.length ? axisList(state.failed_axes) : "";
    useCondition("drive-startup-axes", failedAxes ? moduleNotice({ source: "drive-startup-axes", severity: "error", title: `Inicjalizacja napędów niepotwierdzona · ${failedAxes}`,
        message: `Nie potwierdzono inicjalizacji osi ${failedAxes}. ENABLE tych osi jest zablokowane.`, remedy: "Sprawdź połączenie i zasilanie wskazanych osi, potem użyj „Sprawdź ponownie” w oknie startu napędów." }, "Start napędów", "#manual") : null);
    useCondition("drive-startup-check", state?.phase === "check_failed" ? moduleNotice({ source: "drive-startup-check", severity: "error", title: "Nie udało się sprawdzić nastaw napędów",
        message: state.message || "Sprawdzenie nastaw po starcie nie powiodło się." }, "Start napędów", "#manual") : null);
    useCondition("drive-startup-power", state?.phase === "applied" && state.last_apply?.requires_power_cycle ? moduleNotice({ source: "drive-startup-power", severity: "warning",
        title: "Wymagane wyłączenie zasilania napędów", message: "Zmieniono mikrokrok. Przed ruchem wyłącz i włącz zasilanie napędów." }, "Start napędów", "#manual") : null);
    if (!state?.attention) return null;

    const post = async (action: "apply" | "dismiss" | "recheck") => {
        setSending(true); setError("");
        try {
            const response = await fetch(`/api/v1/drive-startup/${action}`, { method: "POST", headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ generation: state.generation }) });
            if (!response.ok) {
                const detail = await response.json().catch(() => null) as { detail?: unknown } | null;
                setError(typeof detail?.detail === "string" ? detail.detail : `Polecenie odrzucone (HTTP ${response.status})`);
            }
            await refresh();
        } catch { setError("Brak połączenia z serwerem panelu"); }
        finally { setSending(false); }
    };

    const rows = differenceRows(state.differences), manual = differenceRows(state.manual_differences);
    const targets = axisList(Object.keys(state.differences)), errors = applyErrors(state);
    const applying = state.phase === "applying", applied = state.phase === "applied", failedCheck = state.phase === "check_failed";
    const title = applied ? "Nastawy zapisane w napędach" : failedCheck ? "Nie udało się sprawdzić nastaw napędów"
        : rows.length || manual.length || applying ? "Nastawy napędów różnią się od zapamiętanych" : "Inicjalizacja napędów niepełna";
    return <div className="prg-modal-bg drive-startup-bg"><div className="prg-modal drive-startup" role="alertdialog" aria-modal="true" aria-label={title}>
        <h3>{title}</h3>
        {state.failed_axes.length > 0 && <p className="prg-error">Nie potwierdzono inicjalizacji osi {axisList(state.failed_axes)}. ENABLE tych osi pozostaje zablokowane, a system ponawia wyłączenie. Sprawdź połączenie i stan napędu.</p>}
        {failedCheck && <p className="prg-error">{state.message}</p>}
        {applying && <p className="prg-note" role="status">{state.message}…</p>}
        {applied && <p className="prg-note">Odczyt kontrolny potwierdził zgodność nastaw z zapamiętanymi wartościami.</p>}
        {applied && state.last_apply?.requires_power_cycle && <p className="prg-error">Zmieniono mikrokrok. Przed ruchem wyłącz i włącz zasilanie napędów.</p>}
        {rows.length > 0 && <>
            <p className="prg-note">Napędy są wyłączone (ENABLE=0). Wysłanie zapisze zapamiętane wartości tylko w osiach {targets}, a każdy zapis zostanie sprawdzony odczytem. Stan obecny trafi najpierw do kopii <code>pre_test</code>.</p>
            <Differences rows={rows} />
        </>}
        {manual.length > 0 && <>
            <p className="prg-note">Tych nastaw system nie zapisuje automatycznie. Wymagają sprawdzenia przez operatora:</p>
            <Differences rows={manual} />
        </>}
        {errors.map(message => <p className="prg-error" key={message}>{message}</p>)}
        {error && <p className="prg-error">{error}</p>}
        <div className="prg-modal-actions">
            {failedCheck && <button className="secondary-button" disabled={sending} onClick={() => void post("recheck")}>Sprawdź ponownie</button>}
            {!applying && <button className={rows.length ? "secondary-button" : "primary-button"} disabled={sending} onClick={() => void post("dismiss")}>
                {applied || (!rows.length && !manual.length && !failedCheck) ? "Zamknij" : "Pomiń do następnego startu"}</button>}
            {rows.length > 0 && <button className="primary-button" disabled={sending || applying} onClick={() => void post("apply")}>{applying ? "Zapisywanie…" : `Wyślij właściwe nastawy do ${targets}`}</button>}
        </div>
    </div></div>;
}
