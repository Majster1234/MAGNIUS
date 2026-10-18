import { useEffect, useRef, useState } from "react";
import { JogControl, MANUAL_STATE_TIMEOUT_MS } from "./manualControl";

export function MotorPosition({ axis, position, stale = false }: { axis: number; position: number | null; stale?: boolean }) {
    const known = position !== null && Number.isFinite(position);
    const bounded = known ? Math.max(-3600, Math.min(3600, position)) : 0;
    return <div className={`motor-position ${stale ? "measurement-stale" : ""}`}>
        <span className="position-value"><span>{known ? `${position.toFixed(1)}°` : "—"}</span>{known && stale && <em>ostatni pomiar</em>}</span>
        <div className={`position-track ${known ? "" : "unknown"}`} role={known ? "meter" : "img"}
            aria-label={`Pozycja J${axis} względem limitów${known ? "" : ": brak danych"}`}
            aria-valuemin={known ? -3600 : undefined} aria-valuemax={known ? 3600 : undefined}
            data-stale={stale} aria-valuenow={known ? bounded : undefined} aria-valuetext={known ? `${stale ? "Ostatni pomiar: " : ""}${position.toFixed(1)}°` : undefined}>
            <span className="position-zero" />
            {known && <span className="position-thumb" style={{ left: `${(bounded + 3600) / 72}%` }} />}
        </div><div className="position-limits"><span>−3600°</span><span>+3600°</span></div>
    </div>;
}

type Axis = {
    axis: number; available: boolean; prepared: boolean; enable_known: boolean; enabled: boolean;
    position_known: boolean; position_degrees: number; speed_known: boolean; speed_rpm: number;
    can_jog: boolean; can_zero: boolean; motion: string;
};
type ManualState = { fresh: boolean; busy: boolean; fault: boolean; recording?: boolean; active_axis: number; max_rpm: number; operation: string; message: string; axes: Axis[] };

export function ManualPage() {
    const [state, setState] = useState<ManualState | null>(null);
    const [connected, setConnected] = useState(false);
    const [fresh, setFresh] = useState(false);
    const [error, setError] = useState("");
    const [pending, setPending] = useState(false);
    const [held, setHeld] = useState<{ axis: number; direction: number } | null>(null);
    const [speeds, setSpeeds] = useState<number[]>(Array(6).fill(30));
    const control = useRef<JogControl | null>(null);
    useEffect(() => {
        let closed = false, socket: WebSocket | null = null;
        let received = -Infinity;
        let recording = false, lastReported = -Infinity, signature = "", stateStamp = "";
        function report(event: string, axis = 0, reason = 0, gap = 0) {
            if (recording && socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify({type: "trace", event,
                client_epoch_ms: Date.now(), client_perf_ms: performance.now(), state_monotonic_ns: stateStamp,
                axis, reason, gap_ms: Number.isFinite(gap) ? gap : 0}));
        }
        let reconnect: ReturnType<typeof setTimeout> | undefined;
        function connect() {
            socket = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/api/v1/manual/ws`);
            const jog = new JogControl(socket, () => performance.now(), (reason, axis) =>
                report("client_stop", axis, ["release", "ack_timeout", "state_timeout", "socket_closed", "state_stale", "ack_rejected"].indexOf(reason)));
            control.current = jog;
            socket.onopen = () => { if (!closed) setConnected(true); };
            socket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (data.type === "state" && Array.isArray(data.state?.axes) && data.state.axes.length === 6) {
                        const now = performance.now(), gap = now - received;
                        recording = data.state.recording === true; stateStamp = data.state.state_monotonic_ns ?? "";
                        const nextSignature = JSON.stringify([data.state.fresh, data.state.fault, data.state.axes.map((axis: Axis) => axis.available)]);
                        if (gap >= 100 || now - lastReported >= 1000 || nextSignature !== signature) {
                            report(gap >= 100 && Number.isFinite(gap) ? "state_gap" : "state_received", 0, 0, gap);
                            lastReported = now; signature = nextSignature;
                        }
                        received = now; setState(data.state); setFresh(data.state.fresh);
                        jog.stateReceived(data.state.fresh);
                    } else if (data.type === "ack") {
                        jog.ack(data.sequence, data.accepted);
                        if (!data.accepted) setError(data.message);
                    }
                } catch { jog.stop(); setFresh(false); }
            };
            socket.onclose = () => {
                jog.stop();
                if (!closed) { setConnected(false); setFresh(false); reconnect = setTimeout(connect, 1000); }
            };
            socket.onerror = () => socket?.close();
        }
        const stop = () => { control.current?.stop(); setHeld(null); };
        const visibility = () => { report("visibility", 0, document.hidden ? 1 : 0); if (document.hidden) stop(); };
        window.addEventListener("blur", stop); window.addEventListener("pagehide", stop);
        window.addEventListener("pointerup", stop); window.addEventListener("pointercancel", stop);
        document.addEventListener("visibilitychange", visibility);
        const timer = setInterval(() => {
            control.current?.pulse();
            if (!control.current?.holding) setHeld(null);
            if (performance.now() - received >= MANUAL_STATE_TIMEOUT_MS) { control.current?.stop("state_timeout"); setFresh(false); }
        }, 50);
        connect();
        return () => {
            closed = true; clearInterval(timer); clearTimeout(reconnect); stop(); socket?.close(); control.current = null;
            window.removeEventListener("blur", stop); window.removeEventListener("pagehide", stop);
            window.removeEventListener("pointerup", stop); window.removeEventListener("pointercancel", stop);
            document.removeEventListener("visibilitychange", visibility);
        };
    }, []);
    async function operation(axis: number, name: "enable" | "zero", enabled?: boolean) {
        control.current?.stop(); setPending(true); setError("");
        try {
            const result = await fetch(`/api/v1/manual/axes/${axis}/${name}`, {
                method: "POST", headers: { "Content-Type": "application/json" },
                body: name === "enable" ? JSON.stringify({ enabled }) : undefined,
            });
            const body = await result.json();
            if (!result.ok) throw new Error(typeof body.detail === "string" ? body.detail : "Operacja odrzucona");
        } catch (failure) { setError(failure instanceof Error ? failure.message : "Brak odpowiedzi backendu"); }
        finally { setPending(false); }
    }
    function arrow(axis: Axis, direction: number) {
        return <button className="jog-arrow" aria-label={`JOG J${axis.axis} ${direction > 0 ? "prawo" : "lewo"}`}
            disabled={!(held?.axis === axis.axis && held.direction === direction) && (!fresh || !connected || !axis.can_jog || pending)}
            onPointerDown={(event) => {
                if (event.button !== 0) return;
                event.preventDefault(); event.currentTarget.setPointerCapture(event.pointerId);
                setError(""); if (control.current?.start(axis.axis, direction, speeds[axis.axis - 1])) setHeld({axis: axis.axis, direction});
            }}
            onPointerUp={() => control.current?.stop()} onPointerCancel={() => control.current?.stop()}
            onLostPointerCapture={() => control.current?.stop()}
            onContextMenu={(event) => event.preventDefault()}>{direction > 0 ? "→" : "←"}</button>;
    }
    const usable = fresh && connected;
    const axes = state?.axes ?? Array.from({ length: 6 }, (_, index) => ({ axis: index + 1 } as Axis));
    return <section className="manual-screen" aria-label="Sterowanie ręczne">
        <header className="manual-header">
            <h1>Sterowanie ręczne</h1>
            <div className="manual-statuses">{state?.recording && <span className="badge warn">Nagrywanie diagnostyki</span>}<span className={`badge ${usable ? "ok" : "stale"}`}>{!connected ? "Brak połączenia" : fresh ? "Połączono · 20 Hz" : "Połączono · dane nieaktualne"}</span></div>
            <a href="#overview">← Przegląd systemu</a>
        </header>
        <div className="manual-message" role="status">{usable ? state?.message : "JOG zablokowany — oczekiwanie na aktualne dane panelu"}</div>
        {error && <div className="manual-error" role="alert">{error}</div>}
        <div className="manual-table"><table><thead><tr><th>Oś / CAN</th><th>Połączenie</th><th>Pozycja wału</th><th>ENABLE</th><th>Napęd</th><th>Zero</th><th>JOG</th><th>Prędkość</th></tr></thead><tbody>
            {axes.map((axis) => <tr key={axis.axis} className={state?.active_axis === axis.axis ? "manual-active" : ""}>
                <td><strong>J{axis.axis}</strong><small>CAN {axis.axis}</small></td>
                <td><span className={`badge ${usable && axis.available ? "ok" : "stale"}`}>{!usable ? "Dane nieaktualne" : axis.available ? "Połączony" : "Niedostępny"}</span></td>
                <td><MotorPosition axis={axis.axis} position={axis.position_known ? axis.position_degrees : null} stale={!usable} /></td>
                <td><span className={`badge ${usable && axis.enable_known ? axis.enabled ? "ok" : "warn" : "stale"}`}>{usable && axis.enable_known ? axis.enabled ? "Włączony" : "Wyłączony" : "Nieznany"}</span></td>
                <td><div className="manual-enable"><button className="secondary-button" disabled={!usable || !axis.available || state?.busy || pending || (axis.enable_known && axis.enabled && axis.prepared)} onClick={() => operation(axis.axis, "enable", true)}>Załącz</button>
                    <button className="secondary-button" disabled={!usable || !axis.available || pending || (state?.busy && !(state.active_axis === axis.axis && (state.operation === "jog" || state.operation === "stopping")))} onClick={() => operation(axis.axis, "enable", false)}>Wyłącz</button></div></td>
                <td><button className="link-button" disabled={!usable || !axis.can_zero || pending} onClick={() => operation(axis.axis, "zero")}>Ustaw zero</button></td>
                <td><div className="jog-controls">{arrow(axis, -1)}<span className={`motion-indicator ${usable && axis.motion === "w prędkości" ? "moving" : ""}`} aria-live="polite">{usable ? axis.motion ?? "stan nieznany" : "stan nieznany"}</span>{arrow(axis, 1)}</div></td>
                <td><label className="jog-speed"><input aria-label={`Prędkość J${axis.axis}`} type="number" min={1} max={state?.max_rpm ?? 30} step={1} value={speeds[axis.axis - 1]} disabled={state?.busy || pending}
                    onChange={(event) => { const next = Math.max(1, Math.min(state?.max_rpm ?? 30, Math.round(Number(event.target.value) || 1))); setSpeeds((old) => old.map((value, i) => i === axis.axis - 1 ? next : value)); }}/><span>obr./min</span></label></td>
            </tr>)}
        </tbody></table></div>
        <p className="can-note">Przytrzymaj strzałkę, aby poruszać wałem; puszczenie hamuje. Limity ±3600° · Rampa acc=1 · → CW · Jedna oś naraz.</p>
    </section>;
}
