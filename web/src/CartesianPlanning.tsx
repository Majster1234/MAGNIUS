import { useCallback, useEffect, useRef, useState } from "react";
import { pointTime, trajectoryPosition, validateCartesianPlan, type CartesianPlan } from "./cartesianTrajectory";
import { contextKey, planningRequest, type PreviewContext } from "./planningContext";
import type { MotionTrace } from "./previewMotionTrail";
import type { Notice } from "./eventLog";

type Start = { values: number[]; simulated: boolean[] };
export function CartesianPlanning({ positions, context, editing, usable, onMotion, onEditingReady, beforePlanning, showMotionFrames = true, onShowMotionFrames, onPlanReady, onPreparedPreview, onNotice, onCondition }: {
    positions: (number | null)[]; context: PreviewContext | null; editing: boolean; usable: boolean;
    onMotion: (positions: number[] | null, trace?: MotionTrace | null) => void;
    onEditingReady?: (ready: boolean) => void;
    beforePlanning?: () => Promise<void>;
    showMotionFrames?: boolean; onShowMotionFrames?: (show: boolean) => void;
    onPlanReady?: (id: string | null, type?: "PTP" | "LIN" | null) => void;
    onPreparedPreview?: (callback: ((plan: CartesianPlan) => void) | null) => void;
    onNotice?: (value:Notice)=>void;
    onCondition?: (key:string,value:Notice|null)=>void;
}) {
    const [planId, setPlanId] = useState<string | null>(null);
    const [choice, setChoice] = useState<"PTP" | "LIN" | null>(null), [start, setStart] = useState<Start | null>(null);
    function setMessage(message:string,severity:Notice["severity"]="info",code?:string) {
        const progress=/ · (\d+)%$/.exec(message);
        onNotice?.({source:"simulation",message:progress?message.slice(0,progress.index):message,severity,code,progress:progress?Number(progress[1]):undefined,context:"Symulacja"});
    }
    const [busy, setBusy] = useState(false), [playing, setPlaying] = useState(false), [serial, setSerial] = useState(0);
    const [overrides, setOverrides] = useState<string[]>(Array(6).fill(""));
    const [velocity, setVelocity] = useState("6"), [acceleration, setAcceleration] = useState("50");
    const [tcpVelocity, setTcpVelocity] = useState("50"), [tcpAcceleration, setTcpAcceleration] = useState("100"), [tcpAngularVelocity, setTcpAngularVelocity] = useState("10");
    const generation = useRef(0), animation = useRef(0), mounted = useRef(true), armed = useRef(false);
    const job = useRef<string | null>(null), cancellation = useRef<Promise<unknown>>(Promise.resolve());
    const work = useRef<Promise<unknown>>(Promise.resolve()), previousContext = useRef(contextKey(context));
    const key = contextKey(context), targetKey = positions.join("/");
    const maxPtpVelocity = Math.min(360, Math.max(...(context?.profiles.map(profile => profile.velocity) ?? [360])));
    const ptpParametersValid = velocity.trim() !== "" && acceleration.trim() !== "" &&
        Number.isFinite(Number(velocity)) && Number(velocity) >= .01 && Number(velocity) <= maxPtpVelocity &&
        Number.isFinite(Number(acceleration)) && Number(acceleration) >= .01 && Number(acceleration) <= 3600;
    const linParametersValid=[[tcpVelocity,10000],[tcpAcceleration,100000],[tcpAngularVelocity,360]].every(([text,max])=>String(text).trim()!=="" && Number.isFinite(Number(text)) && Number(text)>=.01 && Number(text)<=Number(max));
    const editingContext = useRef<string | null>(null);
    useEffect(()=>{onNotice?.({source:"simulation",severity:"info",context:"Symulacja",message:"Ustaw cel przez JOG lub TCP, następnie wybierz PTP lub LIN."});},[onNotice]);
    function cancelJob() {
        if (!job.current) return cancellation.current;
        const identifier = job.current; job.current = null;
        cancellation.current = planningRequest(`planning/jobs/${identifier}`, {}, "DELETE").catch(error => {
            if (!job.current) job.current = identifier;
            throw error;
        });
        void cancellation.current.catch(() => {});
        return cancellation.current;
    }
    function interrupt() {
        generation.current++; cancelAnimationFrame(animation.current); setPlaying(false); setBusy(false);
        void cancelJob();
    }
    function stop() { armed.current = false; interrupt(); setMessage("Podgląd zatrzymany. Wybierz PTP/LIN, aby uruchomić go ponownie."); }
    function changePtpParameter(setValue: (value: string) => void, value: string) {
        armed.current = false; interrupt(); setPlanId(null); onPlanReady?.(null, choice); onMotion(null, null);
        setValue(value);
        setMessage("Parametry PTP zmienione. Wybierz PTP, aby przeliczyć plan; potem przygotuj i zatwierdź ruch ponownie.");
    }
    function changeLinParameter(setValue: (value: string) => void, value: string) {
        interrupt(); setPlanId(null); onPlanReady?.(null, choice); onMotion(null, null); setValue(value);
        setMessage("Parametry LIN zmienione. Podgląd jest przeliczany; fizyczny program wymaga ponownego przygotowania i zatwierdzenia.");
    }
    useEffect(() => {
        mounted.current = true;
        const blur = () => { armed.current = false; interrupt(); setMessage("Podgląd zatrzymany po utracie fokusu. Wybierz PTP/LIN, aby wznowić."); };
        const hidden = () => { if (document.hidden) blur(); };
        window.addEventListener("blur", blur); document.addEventListener("visibilitychange", hidden);
        return () => {
            mounted.current = false; armed.current = false; generation.current++; cancelAnimationFrame(animation.current);
            void cancelJob(); onMotion(null);
            window.removeEventListener("blur", blur); document.removeEventListener("visibilitychange", hidden);
        };
    }, [onMotion]);
    useEffect(() => {
        if (previousContext.current !== key || !usable) {
            armed.current = false; interrupt(); onMotion(null, null);
            if (previousContext.current !== key) {
                setChoice(null); setStart(null); setOverrides(Array(6).fill(""));
                setMessage("Kontekst planowania zmienił się. Ustaw cel i wybierz PTP lub LIN.","warning","configuration");
            } else if (!usable) setMessage("Podgląd zatrzymany — brak aktualnego modelu lub połączenia.","warning","planner");
            previousContext.current = key;
        }
    }, [key, usable, onMotion]);
    function choose(type: "PTP" | "LIN") {
        if (!context || !usable || editing || (type === "PTP" ? !ptpParametersValid : !linParametersValid)) return;
        const values = context.preview_seed_degrees.map((q, i) => overrides[i].trim() ? Number(overrides[i]) : q);
        if (!values.every((q, i) => Number.isFinite(q) && q >= context.profiles[i].lower && q <= context.profiles[i].upper)) {
            setMessage("Start poza limitami. Popraw pozycje w edytorze Start symulacji.","warning","limits"); return;
        }
        setPlanId(null); interrupt(); armed.current = true; setChoice(type);
        setStart({ values, simulated: context.seed_sources.map((source, i) => source !== "measured" || !!overrides[i].trim()) });
        setSerial(old => old + 1);
    }
    useEffect(() => {
        if (editing) {
            if (editingContext.current === key && usable) return;
            editingContext.current = key;
            interrupt(); if (choice && start) armed.current = true;
            const current = generation.current;
            onMotion(null); onEditingReady?.(false);
            void work.current.catch(() => {}).then(() => cancellation.current).then(() => {
                if (mounted.current && generation.current === current) onEditingReady?.(true);
            }).catch(() => { if (mounted.current && generation.current === current) setMessage("Nie potwierdzono anulowania podglądu — TCP zablokowane.","error"); });
            return;
        }
        editingContext.current = null;
        if (!choice || !start || !usable || !context || !armed.current || (choice === "LIN" ? !linParametersValid : !ptpParametersValid)) return;
        interrupt(); const current = generation.current;
        const payload = { model_id: context.model_id, configuration_revision: context.configuration_revision,
            settings_revision: context.settings_revision, seed_degrees: start.values, simulated_start: start.simulated,
            target_degrees: positions.map((q, i) => q ?? context.preview_seed_degrees[i]), motion_type: choice,
            ...(choice === "LIN" ? { tcp_velocity_mm_s: Number(tcpVelocity), tcp_acceleration_mm_s2: Number(tcpAcceleration),
                tcp_angular_velocity_degrees_s: Number(tcpAngularVelocity) } : {
                preview_velocity_degrees_s: Number(velocity), preview_acceleration_degrees_s2: Number(acceleration) }) };
        const valid = () => mounted.current && current === generation.current && armed.current;
        setPlanId(null); setBusy(true); setMessage(`Obliczanie ${choice}…`); onMotion(start.values);
        work.current = work.current.catch(() => {}).then(async () => {
            try {
                await cancellation.current;
                await beforePlanning?.();
                if (!valid()) return;
                const started = await planningRequest("planning/moves", payload);
                job.current = started.id;
                if (!valid()) { await cancelJob(); return; }
                for (;;) {
                    const status = await planningRequest(`planning/jobs/${started.id}`);
                    if (!valid()) { await cancelJob(); return; }
                    setMessage(`${choice} · ${status.phase} · ${Math.round(status.progress * 100)}%`);
                    if (status.status === "succeeded") {
                        job.current = null;
                        const plan = await planningRequest(`planning/plans/${status.plan_id}`); validateCartesianPlan(plan);
                        if (!valid()) return;
                        if (plan.model_id !== payload.model_id || plan.configuration_revision !== payload.configuration_revision ||
                            plan.settings_revision !== payload.settings_revision || plan.motion_type !== choice || !plan.simulation_only || plan.execution_ready)
                            throw Error("Wynik dotyczy innego kontekstu symulacji; wybierz ruch ponownie.");
                        setPlanId(plan.id);
                        const total = pointTime(plan.trajectory.points.at(-1)!);
                        const epoch = performance.now(); let previous = epoch;
                        setBusy(false); setPlaying(true); setMessage(`${choice} · ${total.toFixed(2)} s · pętla · tylko symulacja`);
                        function frame(now: number) {
                            if (!valid()) return;
                            if (document.hidden || now - previous > 500) { armed.current = false; setPlaying(false); return; }
                            previous = now;
                            const elapsed = (now - epoch) / 1000, t = Math.min(total, elapsed % (total + .5));
                            onMotion(trajectoryPosition(plan, t), { plan, seconds: t }); animation.current = requestAnimationFrame(frame);
                        }
                        onMotion(trajectoryPosition(plan, 0), { plan, seconds: 0 }); animation.current = requestAnimationFrame(frame); return;
                    }
                    if (status.status !== "running") { job.current = null; throw Error(status.message || "Obliczenia anulowane"); }
                    await new Promise(resolve => setTimeout(resolve, 100));
                }
            } catch (error) {
                if (valid()) { armed.current = false; setMessage((error as Error).message,"error"); }
            } finally {
                if (!valid()) { try { await cancelJob(); } catch { /* Nowe obliczenia nie ruszą bez potwierdzenia anulowania. */ } }
                if (mounted.current && current === generation.current) setBusy(false);
            }
        });
    }, [choice, start, serial, targetKey, editing, usable, key, velocity, acceleration, tcpVelocity, tcpAcceleration, tcpAngularVelocity, onMotion, onEditingReady, beforePlanning]);
    const showPrepared = useCallback((plan: CartesianPlan) => {
        armed.current = false;generation.current++;cancelAnimationFrame(animation.current);
        setBusy(false);setPlaying(true);
        const current = generation.current, epoch = performance.now(), total = pointTime(plan.trajectory.points.at(-1)!);
        setMessage(`Podgląd przygotowanego ${plan.motion_type} · ${total.toFixed(2)} s · fizyczne wykonanie wymaga zatwierdzenia`);
        function frame(now: number) {
            if (!mounted.current || generation.current !== current) return;
            const seconds = Math.min(total, (now - epoch) / 1000);
            onMotion(trajectoryPosition(plan, seconds), { plan, seconds });
            if (seconds < total) animation.current = requestAnimationFrame(frame);else setPlaying(false);
        }
        animation.current = requestAnimationFrame(frame);
    }, [onMotion]);
    useEffect(() => { onPlanReady?.(planId, choice);return () => onPlanReady?.(null); }, [choice, planId, onPlanReady]);
    useEffect(() => { onPreparedPreview?.(showPrepared);return () => onPreparedPreview?.(null); }, [showPrepared, onPreparedPreview]);
    const invalidStart = context?.preview_seed_degrees.some((q, i) => {const value=overrides[i].trim()?Number(overrides[i]):q;return !Number.isFinite(value) || value < context.profiles[i].lower || value > context.profiles[i].upper;});
    useEffect(()=>{
        const invalid=choice==="LIN"?!linParametersValid:!ptpParametersValid;
        onCondition?.("simulation-parameters",invalid?{source:"simulation-parameters",severity:"warning",code:"limits",context:"Parametry podglądu",message:choice==="LIN"?"LIN: podaj prędkość TCP 0,01–10000 mm/s, przyspieszenie 0,01–100000 mm/s² i obrót 0,01–360°/s.":`PTP: podaj prędkość 0,01–${maxPtpVelocity}°/s i przyspieszenie 0,01–3600°/s².`}:null);
        onCondition?.("simulation-start",invalidStart?{source:"simulation-start",severity:"warning",code:"limits",context:"Start symulacji",message:"Start poza limitami. Popraw pozycje w edytorze Start symulacji."}:null);
    },[choice,linParametersValid,ptpParametersValid,invalidStart,maxPtpVelocity,onCondition]);
    useEffect(()=>()=>{onCondition?.("simulation-parameters",null);onCondition?.("simulation-start",null);},[onCondition]);
    return <div className="cartesian-planning motion-selection" aria-label="Podgląd ruchu PTP/LIN">
        <div className="cartesian-card">
            <header className="motion-stage-heading"><span className="motion-stage-number">1</span><div><h3>Podgląd PTP/LIN</h3><small>Cel ustawiany przez JOG lub TCP</small></div><span className="badge planning-badge">SYMULACJA</span></header>
            <div className="cartesian-actions">
                {(["PTP", "LIN"] as const).map(type => <button key={type} aria-pressed={choice === type}
                    disabled={!context?.planner_available || !usable || editing || (type === "PTP" ? !ptpParametersValid : !linParametersValid)} onClick={() => choose(type)}>{type}</button>)}
                <button disabled={!busy && !playing} onClick={stop}>Zatrzymaj podgląd</button>
            </div>
            {choice !== "LIN" && <fieldset className="ptp-parameters" disabled={editing || !usable}>
                <legend>Parametry ruchu PTP</legend>
                <div className="cartesian-target">
                    <label>Prędkość PTP °/s<input aria-label="Prędkość PTP °/s" type="number" min="0.01" max={maxPtpVelocity} step="any" value={velocity}
                        aria-errormessage="control-issue-simulation-parameters" aria-invalid={velocity.trim() === "" || !Number.isFinite(Number(velocity)) || Number(velocity) < .01 || Number(velocity) > maxPtpVelocity}
                        onChange={e => changePtpParameter(setVelocity, e.target.value)} /></label>
                    <label>Przyspieszenie PTP °/s²<input aria-label="Przyspieszenie PTP °/s²" type="number" min="0.01" max="3600" step="any" value={acceleration}
                        aria-errormessage="control-issue-simulation-parameters" aria-invalid={acceleration.trim() === "" || !Number.isFinite(Number(acceleration)) || Number(acceleration) < .01 || Number(acceleration) > 3600}
                        onChange={e => changePtpParameter(setAcceleration, e.target.value)} /></label>
                </div>
                <small>Wartości dotyczą przegubów i obowiązują w podglądzie oraz wykonaniu fizycznym. Każda oś zachowuje własne limity. Zmiana wymaga ponownego obliczenia i zatwierdzenia PTP.</small>
            </fieldset>}
            <label className="motion-frames-toggle"><input type="checkbox" checked={showMotionFrames} onChange={event => onShowMotionFrames?.(event.target.checked)} />Pokaż klatki pośrednie</label>
            <small>Przy osobliwości nadgarstka J4/J6 zwalniają do maks. 5°/s.</small>{" "}
            {start && <small>Stały start · symulowane odniesienie: {start.simulated.map((value, i) => value ? `J${i + 1}` : "").filter(Boolean).join(", ") || "brak"}</small>}
            <details open={invalidStart || undefined}><summary>Start symulacji</summary>
                <p>Pomiar → ostatni pomiar → 0° tylko w symulacji. Korekta obowiązuje przy następnym wyborze ruchu.</p>
                <div className="cartesian-seed">{Array.from({ length: 6 }, (_, i) => <label key={i}>J{i + 1} · {context?.seed_sources[i] === "measured" ? "pomiar" : context?.seed_sources[i] === "last" ? "ostatni" : "sym. 0°"}
                    <input type="number" step="any" aria-label={`Start symulacji J${i + 1}`} value={overrides[i]} aria-errormessage="control-issue-simulation-start" aria-invalid={!!context && (!Number.isFinite(Number(overrides[i])) || (overrides[i].trim() !== "" && (Number(overrides[i]) < context.profiles[i].lower || Number(overrides[i]) > context.profiles[i].upper)))} placeholder={String(context?.preview_seed_degrees[i] ?? 0)}
                        onChange={e => setOverrides(old => old.map((q, j) => j === i ? e.target.value : q))} /></label>)}</div>
            </details>
            {choice === "LIN" && <details><summary>Prędkość podglądu LIN</summary><div className="cartesian-target">
                    <label>Prędkość TCP mm/s<input type="number" min="0.01" max="10000" value={tcpVelocity} aria-errormessage="control-issue-simulation-parameters" aria-invalid={tcpVelocity.trim() === "" || !Number.isFinite(Number(tcpVelocity)) || Number(tcpVelocity) < .01 || Number(tcpVelocity) > 10000} onChange={e => changeLinParameter(setTcpVelocity, e.target.value)} /></label>
                    <label>Przysp. TCP mm/s²<input type="number" min="0.01" max="100000" value={tcpAcceleration} aria-errormessage="control-issue-simulation-parameters" aria-invalid={tcpAcceleration.trim() === "" || !Number.isFinite(Number(tcpAcceleration)) || Number(tcpAcceleration) < .01 || Number(tcpAcceleration) > 100000} onChange={e => changeLinParameter(setTcpAcceleration, e.target.value)} /></label>
                    <label>Obrót TCP °/s<input type="number" min="0.01" max="360" value={tcpAngularVelocity} aria-errormessage="control-issue-simulation-parameters" aria-invalid={tcpAngularVelocity.trim() === "" || !Number.isFinite(Number(tcpAngularVelocity)) || Number(tcpAngularVelocity) < .01 || Number(tcpAngularVelocity) > 360} onChange={e => changeLinParameter(setTcpAngularVelocity, e.target.value)} /></label>
            </div></details>}
        </div>
    </div>;
}
