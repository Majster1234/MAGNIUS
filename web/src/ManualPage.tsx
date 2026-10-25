import { useCallback, useEffect, useRef, useState } from "react";
import { JogControl, MANUAL_STATE_TIMEOUT_MS } from "./manualControl";
import { RobotPreview } from "./RobotPreview";
import { PlanningJog } from "./planningJog";
import { MotionExecution } from "./MotionExecution";
import type { CartesianPlan } from "./cartesianTrajectory";
import { CartesianPlanning } from "./CartesianPlanning";
import { contextKey, usePlanningContext } from "./planningContext";
import { PlanningTcpJog } from "./planningTcpJog";
import type { TcpHeld, TcpAxis, TcpFrame } from "./planningTcpJog";
import { TcpControls } from "./TcpControls";
import { PlanningTcpStep } from "./planningTcpStep";
import type { MotionTrace } from "./previewMotionTrail";
import { EventConsole, useEventConsole } from "./EventConsole";
import type { Axis, ManualState } from "./manualTypes";
import type { ManualHeaderState } from "./AppShell";
import type { MotionObservation } from "./motionStatus";

function calibrationRepair(axis: Axis) {
    return axis.joint_configured && !axis.calibration_valid ?
        `J${axis.axis}: nieważna kalibracja przegubu po zerowaniu wału. Wyłącz napęd przyciskiem „Wyłącz”. W zakładce „Konfiguracja” sprawdź i ustaw poprawny „Offset zera przegubu” dla J${axis.axis}, a następnie kliknij „Zapisz do ROS”, aby ponownie zatwierdzić kalibrację. Po zapisie możesz przygotować napęd przyciskiem „Załącz”. Do tego czasu pozycja dotyczy wału, a JOG przegubu jest zablokowany.` : undefined;
}

export function MotorPosition({ axis, position, plannedPosition, stale = false, lower = -3600, upper = 3600, shaftFallback = false }: { axis: number; position: number | null; plannedPosition?: number | null; stale?: boolean; lower?: number; upper?: number; shaftFallback?: boolean }) {
    const known = position !== null && Number.isFinite(position);
    const bounded = known ? Math.max(lower, Math.min(upper, position)) : 0;
    const plannedKnown = plannedPosition !== null && plannedPosition !== undefined && Number.isFinite(plannedPosition);
    const plannedLabel = plannedKnown ? `Bliźniak J${axis}: ${plannedPosition.toFixed(1)}°` : "";
    return <div className={`motor-position ${stale ? "measurement-stale" : ""}`}>
        <span className="position-value"><span>{known ? `${position.toFixed(1)}°` : "—"}</span>{shaftFallback && <em>wał · brak kalibracji</em>}{known && stale && <em>ostatni pomiar</em>}</span>
        <div className={`position-track ${known ? "" : "unknown"}`} role={known ? "meter" : "img"}
            aria-label={`Pozycja ${shaftFallback ? "wału " : ""}J${axis} względem limitów${known ? "" : ": brak danych"}`}
            aria-valuemin={known ? lower : undefined} aria-valuemax={known ? upper : undefined}
            data-stale={stale} aria-valuenow={known ? bounded : undefined} aria-valuetext={known ? `${stale ? "Ostatni pomiar: " : ""}${position.toFixed(1)}°${plannedKnown ? `; ${plannedLabel}` : ""}` : undefined}>
            {lower <= 0 && upper >= 0 && <span className="position-zero" style={{left:`${-lower / (upper-lower)*100}%`}}/>}
            {known && <span className="position-thumb" style={{ left: `${(bounded - lower) / (upper - lower) * 100}%` }} />}
            {plannedKnown && <span className="position-planned-thumb" title={plannedLabel} aria-hidden="true" data-position={plannedPosition}
                style={{ left: `${(Math.max(lower, Math.min(upper, plannedPosition)) - lower) / (upper - lower) * 100}%` }} />}
        </div><div className="position-limits"><span>{Number(lower.toFixed(2))}°</span><span>{upper > 0 ? "+" : ""}{Number(upper.toFixed(2))}°</span></div>
    </div>;
}

export function AxisPosition({ axis, usable, planning = false, plannedPosition, planAvailable = false }: { axis: Axis; usable: boolean; planning?: boolean; plannedPosition?: number | null; planAvailable?: boolean }) {
    const shaftFallback = !!axis.joint_configured && !axis.calibration_valid;
    const joint = !!axis.joint_configured && !shaftFallback;
    const title = shaftFallback ? calibrationRepair(axis) :
        joint ? `Przegub; wał: ${axis.position_known ? axis.position_degrees.toFixed(1) + "°" : "brak danych"}` : "Wał silnika — brak pełnego profilu przegubu";
    return <div className="manual-position-reading" title={title}><MotorPosition axis={axis.axis}
        position={joint ? axis.joint_position_known ? axis.joint_position_degrees! : null : axis.position_known ? axis.position_degrees : null}
        lower={joint ? axis.joint_lower_degrees : -3600} upper={joint ? axis.joint_upper_degrees : 3600} stale={!usable} shaftFallback={shaftFallback}
        plannedPosition={planning && !shaftFallback ? plannedPosition : undefined}/>{planning && <small className={`planned-position ${planAvailable ? "" : "plan-unavailable"}`}>Plan: {plannedPosition == null ? "—" : `${plannedPosition.toFixed(1)}°`}</small>}</div>;
}


export function ManualPage({ onHeaderStatus }: { onHeaderStatus?: (status: ManualHeaderState) => void } = {}) {
    const [state, setState] = useState<ManualState | null>(null);
    const [connected, setConnected] = useState(false);
    const [fresh, setFresh] = useState(false);
    const { entries, notice } = useEventConsole();
    const [pending, setPending] = useState(false);
    const [held, setHeld] = useState<{ axis: number; direction: number } | null>(null);
    const [planning, setPlanning] = useState(false);
    const [showMotionFrames, setShowMotionFrames] = useState(true);
    useEffect(() => { onHeaderStatus?.({ connected, fresh, planning, recording: !!state?.recording }); }, [onHeaderStatus, connected, fresh, planning, state?.recording]);
    const [simulationPositions, setSimulationPositions] = useState<number[] | null>(null);
    const [motionTrace, setMotionTrace] = useState<MotionTrace | null>(null);
    const [execution, setExecution] = useState<MotionObservation | null>(null);
    const context = usePlanningContext();
    const simulationAllowed = !!context?.ignore_missing_drivers && connected;
    const [plan, setPlan] = useState(() => new PlanningJog().snapshot(performance.now()));
    const [previewModel, setPreviewModel] = useState<{ ready: boolean; revision?: string }>({ ready: false });
    const [speeds, setSpeeds] = useState<number[]>(Array(6).fill(30));
    const control = useRef<JogControl | null>(null);
    const planningControl = useRef(new PlanningJog());
    const [tcpHeld, setTcpHeld] = useState<TcpHeld | null>(null);
    const tcpPlannerReady = useRef(false);
    const tcpEnvironment = useRef({ allowed: false, key: "" });
    const tcpContextKey = useRef("");
    const tcpControl = useRef<PlanningTcpJog | null>(null);
    const tcpStepControl = useRef<PlanningTcpStep | null>(null);
    if (!tcpControl.current) tcpControl.current = new PlanningTcpJog(positions => {
        const applied = planningControl.current.preview(positions);
        if (applied) setPlan(planningControl.current.snapshot(performance.now()));
        return applied;
    }, message => notice({ source: "tcp", severity: "warning", message }));
    if (!tcpStepControl.current) tcpStepControl.current = new PlanningTcpStep(positions => {
        const applied = planningControl.current.preview(positions);
        if (applied) setPlan(planningControl.current.snapshot(performance.now()));
        return applied;
    }, message => notice({ source: "tcp-step", severity: "warning", message }));
    const beforePlanning = useCallback(async () => { await tcpStepControl.current?.settled(); }, []);
    const onEditingReady = useCallback((ready: boolean) => { tcpPlannerReady.current = ready; }, []);
    const [physicalPlanId, setPhysicalPlanId] = useState<string | null>(null);
    const [physicalMotionType, setPhysicalMotionType] = useState<"PTP" | "LIN" | null>(null);
    const receivePhysicalPlan = useCallback((id: string | null, type?: "PTP" | "LIN" | null) => {
        setPhysicalPlanId(id); setPhysicalMotionType(type ?? null);
    }, []);
    const preparedPreview = useRef<((plan: CartesianPlan) => void) | null>(null);
    const registerPreparedPreview = useCallback((callback: ((plan: CartesianPlan) => void) | null) => { preparedPreview.current = callback; }, []);
    const showPreparedPreview = useCallback((plan: CartesianPlan) => { preparedPreview.current?.(plan); }, []);
    const planningMode = useRef(false);
    const latestState = useRef<ManualState | null>(null);
    const modelStatus = useRef(previewModel); modelStatus.current = previewModel;
    const expectedRevision = context?.configuration_revision ?? state?.configuration_revision;
    const expectedModelRevision = useRef(expectedRevision); expectedModelRevision.current = expectedRevision;
    const modelReady = previewModel.ready && previewModel.revision === expectedRevision;
    const tcpAllowed = planning && connected && modelReady && plan.ready && !!context && !pending && !state?.busy &&
        plan.available.every(Boolean) && plan.positions.every(q => q !== null && Number.isFinite(q));
    tcpEnvironment.current = { allowed: tcpAllowed, key: contextKey(context) };
    const applyMotionPreview = useCallback((positions: number[] | null, trace?: MotionTrace | null) => {
        setSimulationPositions(planningMode.current ? positions : null);
        if (trace !== undefined) setMotionTrace(planningMode.current ? trace : null);
    }, []);
    useEffect(() => {
        if (contextKey(context) !== tcpContextKey.current) { tcpControl.current?.stop(); tcpStepControl.current?.stop(); setTcpHeld(null); }
        tcpContextKey.current = contextKey(context);
        planningControl.current.configure(context, performance.now());
        setPlan(planningControl.current.snapshot(performance.now()));
    }, [context]);
    const speedProfile = state?.axes.map(axis => `${axis.joint_configured}/${axis.jog_min_velocity_degrees_s}/${axis.jog_max_velocity_degrees_s}`).join(";");
    useEffect(() => {
        setSpeeds(previous => previous.map((value, i) => {
            const axis = state?.axes[i], step = axis?.jog_min_velocity_degrees_s ?? 0, max = axis?.jog_max_velocity_degrees_s ?? 0;
            if (!axis?.joint_configured || max <= 0) return value;
            if (step <= 0 || max < step) return Math.min(value, max);
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
                        received = now; latestState.current = data.state;
                        if (planningControl.current.receive(data.state, now)) notice({ source: "planning", severity: "warning", message: "Plan unieważniony przez zmianę konfiguracji. Skopiuj ponownie pozycję robota." });
                        setPlan(planningControl.current.snapshot(now)); setState(data.state); setFresh(data.state.fresh);
                        if (!planningMode.current) jog.stateReceived(data.state.fresh);
                    } else if (data.type === "ack") {
                        jog.ack(data.sequence, data.accepted);
                        if (!data.accepted) notice({ source: "jog", severity: "warning", message: data.message || "Komenda JOG odrzucona." });
                    }
                } catch { if (!planningMode.current) jog.stop(); tcpControl.current?.stop(); tcpStepControl.current?.stop(); setTcpHeld(null); planningControl.current.disconnect(); setPlan(planningControl.current.snapshot(performance.now())); setFresh(false); }
            };
            socket.onclose = () => {
                if (!planningMode.current) jog.stop();
                tcpControl.current?.stop(); tcpStepControl.current?.stop(); setTcpHeld(null);
                planningControl.current.disconnect(); setPlan(planningControl.current.snapshot(performance.now()));
                if (!closed) { setConnected(false); setFresh(false); reconnect = setTimeout(connect, 1000); }
            };
            socket.onerror = () => socket?.close();
        }
        const stop = (event?: Event) => {
            if (!planningMode.current) control.current?.stop(); tcpControl.current?.stop();
            if (event?.type !== "pointerup") tcpStepControl.current?.stop();
            setTcpHeld(tcpStepControl.current?.held ?? null); planningControl.current.stop(); setPlan(planningControl.current.snapshot(performance.now())); setHeld(null);
        };
        const visibility = () => { report("visibility", 0, document.hidden ? 1 : 0); if (document.hidden) stop(); };
        window.addEventListener("blur", stop); window.addEventListener("pagehide", stop);
        window.addEventListener("pointerup", stop); window.addEventListener("pointercancel", stop);
        document.addEventListener("visibilitychange", visibility);
        const timer = setInterval(() => {
            const now = performance.now();
            if (!planningMode.current) control.current?.pulse();
            if (document.hidden || !modelStatus.current.ready || modelStatus.current.revision !== expectedModelRevision.current) planningControl.current.stop();
            planningControl.current.tick(now); setPlan(planningControl.current.snapshot(now));
            const tcpKey = tcpEnvironment.current.key;
            tcpControl.current?.pulse(now, planningControl.current.snapshot(now).positions,
                () => tcpEnvironment.current.allowed && tcpEnvironment.current.key === tcpKey && !document.hidden &&
                    planningMode.current && planningControl.current.snapshot(performance.now()).available.every(Boolean) &&
                    modelStatus.current.ready && modelStatus.current.revision === expectedModelRevision.current,
                tcpPlannerReady.current);
            tcpStepControl.current?.pulse(now, tcpPlannerReady.current);
            setTcpHeld(tcpControl.current?.held ?? tcpStepControl.current?.held ?? null);
            if (!control.current?.holding && planningControl.current.activeAxis === 0) setHeld(null);
            if (now - received >= MANUAL_STATE_TIMEOUT_MS) { if (!planningMode.current) control.current?.stop("state_timeout"); setFresh(false); }
        }, 50);
        connect();
        return () => {
            closed = true; clearInterval(timer); clearTimeout(reconnect); stop(); planningControl.current.end(); socket?.close(); control.current = null;
            window.removeEventListener("blur", stop); window.removeEventListener("pagehide", stop);
            window.removeEventListener("pointerup", stop); window.removeEventListener("pointercancel", stop);
            document.removeEventListener("visibilitychange", visibility);
        };
    }, [notice]);
    function beginPlanning() {
        if (planningMode.current || !modelReady || !connected || (!simulationAllowed && (!latestState.current || !fresh)) || latestState.current?.busy || pending || control.current?.holding) return;
        if (!planningControl.current.begin(performance.now())) return;
        planningMode.current = true; setPlan(planningControl.current.snapshot(performance.now())); setPlanning(true); setHeld(null);
        notice({ source: "planning", severity: "info", message: "Włączono planowanie lokalne. JOG i TCP poruszają wyłącznie bliźniakiem 3D." });
    }
    function syncPlanning() {
        if (!planningMode.current || !modelReady || !connected || (!simulationAllowed && (!latestState.current || !fresh)) || latestState.current?.busy || pending) return;
        tcpControl.current?.stop(); tcpStepControl.current?.stop(); setTcpHeld(null);
        if (!planningControl.current.begin(performance.now())) return;
        setPlan(planningControl.current.snapshot(performance.now())); setHeld(null);
        notice({ source: "planning", severity: "info", message: "Bliźniak skopiował aktualne pozycje enkoderów." });
    }
    function endPlanning() {
        setMotionTrace(null);
        tcpControl.current?.stop(); tcpStepControl.current?.stop(); setTcpHeld(null);
        planningControl.current.end(); planningMode.current = false; setPlan(planningControl.current.snapshot(performance.now())); setPlanning(false); setHeld(null); setSimulationPositions(null);
        notice({ source: "planning", severity: "info", message: "Zakończono planowanie lokalne. Fizyczny JOG pozostaje zatrzymany." });
    }
    async function operation(axis: number, name: "enable" | "zero", enabled?: boolean) {
        if (planningMode.current) return;
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
        const locallyHeld = held?.axis === axis.axis && held.direction === direction;
        const disabled = planning ? !!tcpHeld || !modelReady || !plan.ready || !plan.available[axis.axis - 1] || pending ||
            (locallyHeld ? plan.activeAxis !== axis.axis : !planningControl.current.canStart(axis.axis, direction, speeds[axis.axis - 1], performance.now())) :
            !locallyHeld && (!fresh || !connected || !axis.can_jog || pending);
        const stopJog = () => { if (planningMode.current) planningControl.current.stop(); else control.current?.stop(); setPlan(planningControl.current.snapshot(performance.now())); setHeld(null); };
        return <button className="jog-arrow" aria-label={`JOG J${axis.axis} ${direction > 0 ? "prawo" : "lewo"}`}
            disabled={disabled}
            onPointerDown={(event) => {
                if (event.button !== 0) return;
                event.preventDefault(); event.currentTarget.setPointerCapture(event.pointerId);
                if (planningMode.current) {
                    if (tcpControl.current?.held || tcpStepControl.current?.busy) return;
                    if (modelReady && planningControl.current.start(axis.axis, direction, speeds[axis.axis - 1], performance.now())) {
                        setPlan(planningControl.current.snapshot(performance.now())); setHeld({axis: axis.axis, direction});
                    }
                } else {
                    if (axis.joint_configured && !state?.configuration_revision) return;
                    if (control.current?.start(axis.axis, direction, speeds[axis.axis - 1], axis.joint_configured ? state?.configuration_revision : undefined)) setHeld({axis: axis.axis, direction});
                }
            }}
            onPointerUp={stopJog} onPointerCancel={stopJog} onLostPointerCapture={stopJog}
            onContextMenu={(event) => event.preventDefault()}>{direction > 0 ? "→" : "←"}</button>;
    }
    function stopTcp() { tcpControl.current?.stop(); tcpStepControl.current?.stop(); setTcpHeld(null); }
    function startTcp(axis: TcpAxis, direction: number, frame: TcpFrame, speed: number, step?: number) {
        if (!tcpAllowed || !context || planningControl.current.activeAxis || control.current?.holding || tcpControl.current?.held || tcpStepControl.current?.busy) return;
        tcpPlannerReady.current = false;
        if (step !== undefined) {
            const key = contextKey(context);
            if (tcpStepControl.current?.start(context, axis, direction, frame, speed, step, plan.positions,
                () => tcpEnvironment.current.allowed && tcpEnvironment.current.key === key && !document.hidden && planningMode.current &&
                    planningControl.current.snapshot(performance.now()).available.every(Boolean), performance.now())) setTcpHeld(tcpStepControl.current.held);
            return;
        }
        if (tcpControl.current?.start(context, axis, direction, frame, speed, performance.now())) {
            setTcpHeld(tcpControl.current.held);
        }
    }
    function jogSpeed(axis: Axis) {
        const profile = planning ? planningControl.current.controlProfile(axis.axis) ?? axis : axis;
        const maximum = profile.joint_configured ? profile.jog_max_velocity_degrees_s || undefined : state?.max_rpm ?? 30;
        return <label className="jog-speed"><input aria-label={`Prędkość J${axis.axis}`} type="number"
            min={profile.joint_configured ? planning ? 0 : profile.jog_min_velocity_degrees_s || undefined : 1}
            max={maximum} step={profile.joint_configured ? "any" : 1} value={speeds[axis.axis - 1]} disabled={(planning ? plan.activeAxis !== 0 : state?.busy) || pending}
            onChange={event => { const number = Number(event.target.value);
                const next = profile.joint_configured ? Math.max(0, Math.min(maximum ?? 0, Number.isFinite(number) ? number : 0)) : Math.max(1, Math.min(maximum ?? 30, Math.round(number || 1)));
                setSpeeds(old => old.map((value, i) => i === axis.axis - 1 ? next : value)); }} />
            <span>{profile.joint_configured ? "°/s przeg." : "obr./min wału"}</span></label>;
    }
    const usable = fresh && connected;
    const axes = state?.axes ?? Array.from({ length: 6 }, (_, index) => ({ axis: index + 1 } as Axis));
    return <section className={`manual-screen ${planning ? "mode-planning" : "mode-live"}`} data-usable={usable} aria-label="Sterowanie ręczne">
        <header className="manual-header">
            <h1>Sterowanie ręczne</h1>
            <span className="manual-mode"><span aria-hidden="true" />{planning ? "PLANOWANIE · bliźniak" : "LIVE · robot fizyczny"}</span>
        </header>
        <div className="manual-workspace"><div className="manual-observation">
        <RobotPreview state={state} usable={usable} pending={pending} physicalJogActive={!planning && !!held} onNotice={notice} onModelStatus={setPreviewModel} planning={planning} plannedPositions={plan.positions} planningReady={plan.ready} onPlanningStart={beginPlanning} onPlanningSync={syncPlanning} onPlanningEnd={endPlanning}
            simulationPositions={simulationPositions} motionTrace={motionTrace} execution={execution} showMotionFrames={showMotionFrames} simulationAllowed={simulationAllowed} expectedRevision={expectedRevision} modelId={context?.model_id} />
        <div className="manual-console-dock"><h2>Zdarzenia <span>Historia sterowania</span></h2><EventConsole entries={entries} /></div>
        </div><div className="manual-controls-row"><div className="manual-table"><table><colgroup>
            <col className="manual-col-jog"/><col className="manual-col-axis"/><col/>
            <col className="manual-col-drive"/><col className="manual-col-zero"/><col className="manual-col-speed"/>
        </colgroup><thead><tr><th>JOG</th><th>Oś / CAN</th><th title="Pozycja osi / wału">Pozycja</th><th>Napęd / ENABLE</th><th>Zero wału</th><th>Prędkość</th></tr></thead><tbody>
            {axes.map((axis) => <tr key={axis.axis} title={calibrationRepair(axis)} className={[
                planning ? plan.activeAxis === axis.axis ? "manual-active manual-planned-active" : "" : state?.active_axis === axis.axis ? "manual-active" : "",
                calibrationRepair(axis) ? "manual-axis-error" : "",
            ].filter(Boolean).join(" ")}>
                <td><div className={`jog-controls ${planning ? "jog-planning" : ""}`}>{arrow(axis, -1)}{!planning && <span className={`motion-indicator ${usable && axis.motion === "w prędkości" ? "moving" : ""}`} aria-live="polite"><span>{usable ? axis.motion ?? "stan nieznany" : "stan nieznany"}</span></span>}{arrow(axis, 1)}</div></td>
                <td className="manual-axis"><strong>J{axis.axis}</strong><small>CAN {axis.axis}</small>
                    <span className={`badge ${usable && axis.available ? "ok" : "stale"}`}>{!usable ? "Dane nieaktualne" : axis.available ? "Połączony" : "Niedostępny"}</span></td>
                <td><AxisPosition axis={axis} usable={usable} planning={planning} plannedPosition={plan.positions[axis.axis - 1]} planAvailable={plan.available[axis.axis - 1]}/></td>
                <td className="manual-drive"><span className={`badge ${usable && axis.enable_known ? axis.enabled ? "ok" : "warn" : "stale"}`}>{usable && axis.enable_known ? axis.enabled ? "Włączony" : "Wyłączony" : "Nieznany"}</span>
                    <div className="manual-enable"><button className="secondary-button" disabled={planning || !usable || !axis.available || state?.busy || pending || (axis.enable_known && axis.enabled && axis.prepared)} onClick={() => operation(axis.axis, "enable", true)}>Załącz</button>
                    <button className="secondary-button" disabled={planning || !usable || !axis.available || pending || (state?.busy && !(state.active_axis === axis.axis && (state.operation === "jog" || state.operation === "stopping")))} onClick={() => operation(axis.axis, "enable", false)}>Wyłącz</button></div></td>
                <td><button className="link-button" title={calibrationRepair(axis) ?? "Zeruje enkoder wału; wymaga ponownego zatwierdzenia offsetu skonfigurowanego przegubu"} disabled={planning || !usable || !axis.can_zero || pending} onClick={() => operation(axis.axis, "zero")}>{axis.joint_configured ? "Zeruj wał" : "Ustaw zero"}</button></td>
                <td>{jogSpeed(axis)}</td>
            </tr>)}
        </tbody></table></div>
        <div className="manual-tools"><TcpControls allowed={tcpAllowed && plan.activeAxis === 0} planning={planning} held={tcpHeld} stepAvailable={!!context?.tcp_step_available}
            missing={axes.filter(axis => !usable || !axis.available).map(axis => `J${axis.axis}`)} start={startTcp} stop={stopTcp} />
        {planning && <div className="manual-simulation"><CartesianPlanning positions={plan.positions} context={context} editing={plan.activeAxis !== 0 || !!tcpHeld} usable={connected && modelReady && plan.ready && !!context} onMotion={applyMotionPreview} onEditingReady={onEditingReady} beforePlanning={beforePlanning}
            showMotionFrames={showMotionFrames} onShowMotionFrames={setShowMotionFrames} onPlanReady={receivePhysicalPlan} onPreparedPreview={registerPreparedPreview} /></div>}
        <div className="manual-simulation"><MotionExecution planId={physicalPlanId} motionType={physicalMotionType} disabled={!planning || !connected || !modelReady || plan.activeAxis !== 0 || !!tcpHeld} onPreview={showPreparedPreview} onStatus={setExecution} /></div>
        <aside className="can-note"><span className="manual-info-icon" aria-hidden="true">i</span><div><h2>{planning ? "Plan lokalny" : "Obsługa JOG"}</h2><p>{planning ? "JOG i TCP poruszają bliźniakiem. Fizyczny PTP/LIN wymaga osobnego przygotowania i zatwierdzenia. ENABLE i zerowanie są tu zablokowane. Brak wykrywania kolizji." : "Przytrzymaj strzałkę; puszczenie hamuje. Skonfigurowane osie: °/s przegubu, → kierunek dodatni. Pozostałe: obr./min wału, → CW. Limit wału ±3600° · acc=1 · Jedna oś naraz."}</p></div></aside>
        </div></div></div>
    </section>;
}
