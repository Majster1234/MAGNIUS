import { useEffect, useRef, useState } from "react";
import { JogControl, MANUAL_STATE_TIMEOUT_MS } from "./manualControl";
import { RobotPreview } from "./RobotPreview";
import { EventConsole, useEventConsole } from "./EventConsole";
import type { Axis, ManualState } from "./manualTypes";

export function MotorPosition({ axis, position, stale = false, lower = -3600, upper = 3600 }: { axis: number; position: number | null; stale?: boolean; lower?: number; upper?: number }) {
    const known = position !== null && Number.isFinite(position);
    const bounded = known ? Math.max(lower, Math.min(upper, position)) : 0;
    return <div className={`motor-position ${stale ? "measurement-stale" : ""}`}>
        <span className="position-value"><span>{known ? `${position.toFixed(1)}°` : "—"}</span>{known && stale && <em>ostatni pomiar</em>}</span>
        <div className={`position-track ${known ? "" : "unknown"}`} role={known ? "meter" : "img"}
            aria-label={`Pozycja J${axis} względem limitów${known ? "" : ": brak danych"}`}
            aria-valuemin={known ? lower : undefined} aria-valuemax={known ? upper : undefined}
            data-stale={stale} aria-valuenow={known ? bounded : undefined} aria-valuetext={known ? `${stale ? "Ostatni pomiar: " : ""}${position.toFixed(1)}°` : undefined}>
            {lower <= 0 && upper >= 0 && <span className="position-zero" style={{left:`${-lower / (upper-lower)*100}%`}}/>}
            {known && <span className="position-thumb" style={{ left: `${(bounded - lower) / (upper - lower) * 100}%` }} />}
        </div><div className="position-limits"><span>{lower}°</span><span>{upper > 0 ? "+" : ""}{upper}°</span></div>
    </div>;
}


export function ManualPage() {
    const [state, setState] = useState<ManualState | null>(null);
    const [connected, setConnected] = useState(false);
    const [fresh, setFresh] = useState(false);
    const { entries, notice } = useEventConsole();
    const [pending, setPending] = useState(false);
    const [held, setHeld] = useState<{ axis: number; direction: number } | null>(null);
    const [speeds, setSpeeds] = useState<number[]>(Array(6).fill(30));
    const control = useRef<JogControl | null>(null);
    const speedProfile = state?.axes.map(axis => `${axis.joint_configured}/${axis.jog_min_velocity_degrees_s}/${axis.jog_max_velocity_degrees_s}`).join(";");
    useEffect(() => {
        setSpeeds(previous => previous.map((value, i) => {
            const axis = state?.axes[i], step = axis?.jog_min_velocity_degrees_s ?? 0, max = axis?.jog_max_velocity_degrees_s ?? 0;
            if (!axis?.joint_configured || step <= 0 || max < step) return value;
            return Number((Math.max(step, Math.floor(Math.min(value, max) / step + 1e-9) * step)).toPrecision(12));
        }));
    }, [speedProfile]);
    const wasConnected = useRef(false);
    useEffect(() => {
        notice({ source: "connection", severity: connected ? "info" : wasConnected.current ? "error" : "info",
            message: connected ? "Nawiązano połączenie z panelem sterowania." : wasConnected.current ? "Utracono połączenie z panelem. Ponawianie połączenia…" : "Łączenie z panelem sterowania…" });
        if (connected) wasConnected.current = true;
    }, [connected, notice]);
    useEffect(() => {
        if (!connected) return;
        notice({ source: "freshness", severity: fresh ? "info" : "warning", message: fresh ? "Dane sterowania są aktualne." : "Dane sterowania nieaktualne — JOG zablokowany do powrotu pomiarów." });
        if (!fresh || !state) return;
        notice({ source: "controller", severity: state.fault ? "error" : "info", message: state.fault ? `Awaria sterowania: ${state.message || "brak opisu"}` : state.message || "Sterowanie gotowe." });
        const available = state.axes.filter(axis => axis.available).map(axis => `J${axis.axis}`);
        const missing = state.axes.filter(axis => !axis.available).map(axis => `J${axis.axis}`);
        notice({ source: "axes", severity: missing.length ? "warning" : "info", message: `Dostępne napędy: ${available.join(", ") || "brak"}.${missing.length ? ` Niedostępne: ${missing.join(", ")}.` : " Wszystkie osie połączone."}` });
        for (const axis of state.axes) if (axis.configuration_message) notice({source:`profile-${axis.axis}`, severity:axis.joint_configured && (!axis.calibration_valid || !axis.prepared) ? "warning" : "info", message:`J${axis.axis}: ${axis.configuration_message}`});
    }, [state, connected, fresh, notice]);
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
                        if (!data.accepted) notice({ source: "jog", severity: "warning", message: data.message || "Komenda JOG odrzucona." });
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
    }, [notice]);
    async function operation(axis: number, name: "enable" | "zero", enabled?: boolean) {
        control.current?.stop(); setPending(true);
        try {
            const result = await fetch(`/api/v1/manual/axes/${axis}/${name}`, {
                method: "POST", headers: { "Content-Type": "application/json" },
                body: name === "enable" ? JSON.stringify({ enabled }) : undefined,
            });
            const body = await result.json();
            if (!result.ok) throw new Error(typeof body.detail === "string" ? body.detail : "Operacja odrzucona");
            notice({ source: `operation-${axis}`, severity: "info", message: `J${axis}: przyjęto ${name === "zero" ? "ustawienie zera" : enabled ? "załączenie napędu" : "wyłączenie napędu"}. Oczekiwanie na potwierdzenie sterownika.` });
        } catch (failure) { notice({ source: `operation-${axis}`, severity: "error", message: `J${axis}: ${failure instanceof Error ? failure.message : "Brak odpowiedzi backendu"}` }); }
        finally { setPending(false); }
    }
    function arrow(axis: Axis, direction: number) {
        return <button className="jog-arrow" aria-label={`JOG J${axis.axis} ${direction > 0 ? "prawo" : "lewo"}`}
            disabled={!(held?.axis === axis.axis && held.direction === direction) && (!fresh || !connected || !axis.can_jog || pending)}
            onPointerDown={(event) => {
                if (event.button !== 0) return;
                event.preventDefault(); event.currentTarget.setPointerCapture(event.pointerId);
                if (axis.joint_configured && !state?.configuration_revision) return;
                if (control.current?.start(axis.axis, direction, speeds[axis.axis - 1], axis.joint_configured ? state?.configuration_revision : undefined)) setHeld({axis: axis.axis, direction});
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
        <EventConsole entries={entries} />
        <RobotPreview state={state} usable={usable} onNotice={notice} />
        <div className="manual-table"><table><thead><tr><th>Oś / CAN</th><th>Połączenie</th><th>Pozycja osi / wału</th><th>ENABLE</th><th>Napęd</th><th>Zero wału</th><th>JOG</th><th>Prędkość</th></tr></thead><tbody>
            {axes.map((axis) => <tr key={axis.axis} className={state?.active_axis === axis.axis ? "manual-active" : ""}>
                <td><strong>J{axis.axis}</strong><small>CAN {axis.axis}</small></td>
                <td><span className={`badge ${usable && axis.available ? "ok" : "stale"}`}>{!usable ? "Dane nieaktualne" : axis.available ? "Połączony" : "Niedostępny"}</span></td>
                <td title={axis.joint_configured ? `Przegub; wał: ${axis.position_known ? axis.position_degrees.toFixed(1)+"°" : "brak danych"}` : "Wał silnika — brak pełnego profilu przegubu"}><MotorPosition axis={axis.axis}
                    position={axis.joint_configured ? axis.joint_position_known ? axis.joint_position_degrees! : null : axis.position_known ? axis.position_degrees : null}
                    lower={axis.joint_configured ? axis.joint_lower_degrees : -3600} upper={axis.joint_configured ? axis.joint_upper_degrees : 3600} stale={!usable}/></td>
                <td><span className={`badge ${usable && axis.enable_known ? axis.enabled ? "ok" : "warn" : "stale"}`}>{usable && axis.enable_known ? axis.enabled ? "Włączony" : "Wyłączony" : "Nieznany"}</span></td>
                <td><div className="manual-enable"><button className="secondary-button" disabled={!usable || !axis.available || state?.busy || pending || (axis.enable_known && axis.enabled && axis.prepared)} onClick={() => operation(axis.axis, "enable", true)}>Załącz</button>
                    <button className="secondary-button" disabled={!usable || !axis.available || pending || (state?.busy && !(state.active_axis === axis.axis && (state.operation === "jog" || state.operation === "stopping")))} onClick={() => operation(axis.axis, "enable", false)}>Wyłącz</button></div></td>
                <td><button className="link-button" title="Zeruje enkoder wału; wymaga ponownego zatwierdzenia offsetu skonfigurowanego przegubu" disabled={!usable || !axis.can_zero || pending} onClick={() => operation(axis.axis, "zero")}>{axis.joint_configured ? "Zeruj wał" : "Ustaw zero"}</button></td>
                <td><div className="jog-controls">{arrow(axis, -1)}<span className={`motion-indicator ${usable && axis.motion === "w prędkości" ? "moving" : ""}`} aria-live="polite">{usable ? axis.motion ?? "stan nieznany" : "stan nieznany"}</span>{arrow(axis, 1)}</div></td>
                <td><label className="jog-speed"><input aria-label={`Prędkość J${axis.axis}`} type="number" min={axis.joint_configured ? axis.jog_min_velocity_degrees_s || undefined : 1} max={axis.joint_configured ? axis.jog_max_velocity_degrees_s || undefined : state?.max_rpm ?? 30} step={axis.joint_configured ? "any" : 1} value={speeds[axis.axis - 1]} disabled={state?.busy || pending}
                    onChange={(event) => { const number=Number(event.target.value);const next=axis.joint_configured ? Math.max(0,Math.min(axis.jog_max_velocity_degrees_s ?? 0,Number.isFinite(number)?number:0)) : Math.max(1, Math.min(state?.max_rpm ?? 30, Math.round(number || 1))); setSpeeds((old) => old.map((value, i) => i === axis.axis - 1 ? next : value)); }}/><span>{axis.joint_configured ? "°/s przeg." : "obr./min wału"}</span></label></td>
            </tr>)}
        </tbody></table></div>
        <p className="can-note">Przytrzymaj strzałkę; puszczenie hamuje. Skonfigurowane osie: °/s przegubu, → kierunek dodatni. Pozostałe: obr./min wału, → CW. Limit wału ±3600° · acc=1 · Jedna oś naraz.</p>
    </section>;
}
