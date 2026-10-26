import { useEffect, useLayoutEffect, useRef, useState } from "react";
import type { ManualState } from "./manualTypes";
import { PreviewPose } from "./previewPose";
import { previewLimits } from "./previewLimits";
import type { createPreviewScene } from "./robotPreviewScene";
import type { Notice } from "./eventLog";
import type { MotionTrace } from "./previewMotionTrail";
import type { MotionObservation } from "./motionStatus";

export function RobotPreview({ state, usable, pending, physicalJogActive, onNotice, onCondition, onModelStatus, planning, plannedPositions, planningReady, onPlanningStart, onPlanningSync, onPlanningEnd, simulationPositions = null, motionTrace = null, execution = null, simulationAllowed = false, expectedRevision, modelId, showMotionFrames = true }: {
    state: ManualState | null; usable: boolean; onNotice: (notice: Notice) => void; planning: boolean;
    onCondition?:(key:string,value:Notice|null,resolution?:string)=>void;
    pending: boolean; physicalJogActive: boolean; plannedPositions: (number | null)[]; planningReady: boolean; onPlanningStart: () => void; onPlanningSync: () => void; onPlanningEnd: () => void;
    onModelStatus: (status: { ready: boolean; revision?: string }) => void;
    showMotionFrames?: boolean;
    simulationPositions?: number[] | null; simulationAllowed?: boolean; expectedRevision?: string; modelId?: string;
    motionTrace?: MotionTrace | null;
    execution?: MotionObservation | null;
}) {
    const host = useRef<HTMLDivElement>(null);
    const scene = useRef<ReturnType<typeof createPreviewScene> | null>(null);
    const pose = useRef(new PreviewPose());
    const poseRevision = useRef(state?.configuration_revision);
    const input = useRef({ state, usable, planning, plannedPositions, simulationPositions, motionTrace, execution, expectedRevision, showMotionFrames });
    const sample = useRef<{ state: ManualState | null; receivedAt: number; key: string }>({ state: null, receivedAt: -Infinity, key: "" });
    const sampleSequence = useRef(0);
    // Scena czyta spójny, zatwierdzony stan razem ze znacznikiem próbki przed
    // następną klatką. useEffect zostawiał okno z nowym stanem i starym stampem.
    useLayoutEffect(() => {
        if (sample.current.state !== state)
            sample.current = { state, receivedAt: performance.now(), key: state?.state_monotonic_ns || String(++sampleSequence.current) };
        input.current = { state, usable, planning, plannedPositions, simulationPositions, motionTrace, execution, expectedRevision, showMotionFrames };
    });
    const [message, setMessage] = useState("Wczytywanie modelu z ROS…");
    const [modelKind, setModelKind] = useState("partial-demo");
    const [, refresh] = useState(0);
    useEffect(() => {
        const now = performance.now();
        if (poseRevision.current !== state?.configuration_revision) {
            pose.current = new PreviewPose(); poseRevision.current = state?.configuration_revision;
        }
        if (usable) state?.axes.forEach((axis, i) => {
            if (axis.available && axis.position_known && (!axis.joint_configured || axis.joint_position_known))
                pose.current.push(i, axis.joint_configured ? axis.joint_position_degrees! : axis.position_degrees, now - Math.max(0, axis.position_age_seconds ?? 0) * 1000);
        });
    }, [state, usable]);
    useEffect(() => {
        let closed = false, retry: ReturnType<typeof setTimeout> | undefined;
        const revisionToLoad = expectedRevision ?? state?.configuration_revision;
        onModelStatus({ ready: false, revision: revisionToLoad }); setMessage("Wczytywanie modelu z ROS…");
        const abort = new AbortController();
        async function load() {
            try {
                const response = await fetch('/api/v1/robot/description', { signal: abort.signal });
                if (!response.ok) throw Error("Oczekiwanie na opis modelu z ROS…");
                const xml = await response.text();
                const root = new DOMParser().parseFromString(xml, "application/xml").documentElement;
                const revision = root.getAttribute("configuration_revision"), required = input.current.expectedRevision ?? input.current.state?.configuration_revision;
                if ((revision && required && revision !== required) || (required && required !== "initial" && revision !== required)) throw Error("Oczekiwanie na model zgodny z rewizją sterowania ROS…");
                const engine = await import('./robotPreviewScene');
                if (closed || !host.current || (input.current.expectedRevision ?? input.current.state?.configuration_revision) !== revisionToLoad) return;
                try {
                    scene.current = engine.createPreviewScene(host.current, xml, () => {
                        const { state: current, usable: online, planning: activePlanning, plannedPositions: planned } = input.current, now = performance.now();
                        const valid = Array.from({ length: 6 }, (_, i) => !!(online && current?.axes[i]?.available && current.axes[i].position_known && (!current.axes[i].joint_configured || current.axes[i].joint_position_known) && pose.current.fresh(i, now)));
                        return { angles: valid.map((ok, i) => pose.current.angle(i, now, ok)), valid, active: current?.active_axis ?? 0,
                            limits: valid.map((ok, i) => previewLimits(current?.axes[i], pose.current.degrees(i), ok)), planning: activePlanning,
                            plannedAngles: planned.map(position => position === null ? null : position * Math.PI / 180),
                            motionAngles: input.current.simulationPositions?.map(value => value * Math.PI / 180) ?? null,
                            motionTrace: input.current.motionTrace, showMotionFrames: input.current.showMotionFrames,
                            executionFrame: { state: sample.current.state === current ? current : null, usable: online,
                                receivedAt: sample.current.receivedAt, sampleKey: sample.current.key,
                                execution: input.current.execution, revision: revisionToLoad } };
                    },(subject,result)=>{
                        const warning=result && result.level!=="clear";
                        onCondition?.(`singularity-${subject}`,warning?{source:`singularity-${subject}`,severity:"warning",code:"singularity",context:subject,title:result.level==="critical"?"Bardzo blisko osobliwości modelu":"Blisko osobliwości modelu",message:`${subject}: ${result.source==="wrist"?"nadgarstek J4–J6":"ramię"} · ${result.level}.`}:null,result?undefined:`${subject}: ocena osobliwości niedostępna — brak pełnej pozycji.`);
                    });
                    setMessage("");
                    onModelStatus({ ready: true, revision: revisionToLoad });
                    setModelKind(root.getAttribute("model_kind") ?? "partial-demo");
                    onNotice({ source: "model", severity: "info", message: "Model 3D wczytany z ROS. Skonfigurowane osie uwzględniają przełożenia i offsety; pozostałe mają odwzorowanie wału 1:1." });
                } catch {
                    host.current.replaceChildren(); setMessage("Podgląd 3D niedostępny");
                    onNotice({ source: "model", severity: "error", message: "Podgląd 3D niedostępny — sprawdź model lub obsługę WebGL." });
                }
            } catch (error) {
                if (closed) return;
                setMessage("Oczekiwanie na model z ROS…");
                onNotice({ source: "model", severity: "warning", message: error instanceof Error ? error.message : "Nie można wczytać modelu" });
                retry = setTimeout(load, 1000);
            }
        }
        void load();
        const timer = setInterval(() => { if (!document.hidden) refresh(value => value + 1); }, 100);
        return () => { closed = true; abort.abort(); clearTimeout(retry); clearInterval(timer); scene.current?.dispose(); scene.current = null; };
    }, [onNotice, onCondition, onModelStatus, state?.configuration_revision, expectedRevision, modelId]);
    const now = performance.now();
    const valid = Array.from({ length: 6 }, (_, i) => !!(usable && state?.axes[i]?.available && state.axes[i].position_known && (!state.axes[i].joint_configured || state.axes[i].joint_position_known) && pose.current.fresh(i, now)));
    const count = valid.filter(Boolean).length;
    const executionTrace = scene.current?.executionTrace();
    useEffect(()=>{onCondition?.("model-trace-partial",executionTrace?.partial?{source:"model-trace-partial",severity:"warning",code:"partial",context:"Ślad modelu",message:"Tor modelu — niepełne pomiary. Historia śladu zawiera niepełny zestaw pozycji."}:null);},[executionTrace?.partial,onCondition]);
    const canStartPlanning = !message && (usable && count > 0 || simulationAllowed) && !pending && !physicalJogActive && !state?.busy;
    return <section className="robot-preview" aria-label="Wizualizacja robota" aria-busy={!!message}>
        <header><h2>Wizualizacja robota</h2><span className="badge warn">{modelKind === "configured" ? "Geometria konfiguracji" : "Geometria demonstracyjna"} · {state?.axes.some(axis => axis.joint_configured) ? "przełożenia osi" : "wały 1:1"}</span>
            {planning && <span className="badge planning-badge">Planowanie — JOG / TCP</span>}
            <span className={`preview-completeness ${count === 6 ? "complete" : "partial"}`}>{count === 6 ? "Pozycja kompletna — 6/6 osi" : `Pozycja częściowa — ${count}/6 osi`}</span>
        </header>
        <div className="preview-trail-legend" aria-label="Ślady TCP">
            <span className="trail-planned">Plan PTP/LIN</span><span className="trail-executed">Wykonane PTP/LIN</span>
            <button type="button" disabled={!executionTrace?.points} title="Usuwa złotą historię w tym panelu. Historia obejmuje do 20 000 punktów."
                onClick={() => { scene.current?.clearExecutionTrace(); refresh(value => value + 1); }}>Wyczyść ślad</button>
        </div>
        <div className="preview-body"><div className="preview-viewport" aria-label="Model 3D robota">
            <div ref={host} className="preview-canvas" />{message && <span className="preview-loading" aria-hidden="true">3D</span>}
            <button className="preview-home" type="button" aria-label="Dopasuj widok" title="Dopasuj widok" onClick={() => scene.current?.view("fit")} disabled={!!message}>
                <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="M3 11 12 3l9 8M5 10v11h5v-7h4v7h5V10" /></svg>
            </button>
            {!planning ? <button className="preview-planning" type="button" onClick={onPlanningStart} disabled={!canStartPlanning} title="Lokalny JOG bez komend do robota">Planowanie</button> :
                <div className="preview-planning-controls"><button type="button" onClick={onPlanningSync} disabled={!canStartPlanning}>Od pozycji robota</button><button type="button" onClick={onPlanningEnd}>Zakończ planowanie</button></div>}
            {planning && <div className="preview-planning-legend"><span className="real">Robot: enkodery</span><span className="twin">Cel</span><span className="motion">Symulacja ruchu</span>{!planningReady && <span>Plan wymaga ponownej synchronizacji</span>}</div>}
        </div></div>
    </section>;
}
