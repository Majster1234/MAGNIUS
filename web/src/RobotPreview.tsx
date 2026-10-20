import { useEffect, useRef, useState } from "react";
import type { ManualState } from "./manualTypes";
import { PreviewPose } from "./previewPose";
import { previewLimits } from "./previewLimits";
import type { createPreviewScene } from "./robotPreviewScene";
import type { Notice } from "./eventLog";
import type { MotionTrace } from "./previewMotionTrail";

export function RobotPreview({ state, usable, pending, physicalJogActive, onNotice, onModelStatus, planning, plannedPositions, planningReady, onPlanningStart, onPlanningSync, onPlanningEnd, simulationPositions = null, motionTrace = null, simulationAllowed = false, expectedRevision, modelId, showMotionFrames = true }: {
    state: ManualState | null; usable: boolean; onNotice: (notice: Notice) => void; planning: boolean;
    pending: boolean; physicalJogActive: boolean; plannedPositions: (number | null)[]; planningReady: boolean; onPlanningStart: () => void; onPlanningSync: () => void; onPlanningEnd: () => void;
    onModelStatus: (status: { ready: boolean; revision?: string }) => void;
    showMotionFrames?: boolean;
    simulationPositions?: number[] | null; simulationAllowed?: boolean; expectedRevision?: string; modelId?: string;
    motionTrace?: MotionTrace | null;
}) {
    const host = useRef<HTMLDivElement>(null);
    const scene = useRef<ReturnType<typeof createPreviewScene> | null>(null);
    const pose = useRef(new PreviewPose());
    const poseRevision = useRef(state?.configuration_revision);
    const input = useRef({ state, usable, planning, plannedPositions, simulationPositions, motionTrace, expectedRevision, showMotionFrames }); input.current = { state, usable, planning, plannedPositions, simulationPositions, motionTrace, expectedRevision, showMotionFrames };
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
                            motionTrace: input.current.motionTrace, showMotionFrames: input.current.showMotionFrames };
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
    }, [onNotice, onModelStatus, state?.configuration_revision, expectedRevision, modelId]);
    const now = performance.now();
    const valid = Array.from({ length: 6 }, (_, i) => !!(usable && state?.axes[i]?.available && state.axes[i].position_known && (!state.axes[i].joint_configured || state.axes[i].joint_position_known) && pose.current.fresh(i, now)));
    const count = valid.filter(Boolean).length;
    const canStartPlanning = !message && (usable && count > 0 || simulationAllowed) && !pending && !physicalJogActive && !state?.busy;
    return <section className="robot-preview" aria-label="Wizualizacja robota">
        <header><h2>Wizualizacja robota</h2><span className="badge warn">{modelKind === "configured" ? "Geometria konfiguracji" : "Geometria demonstracyjna"} · {state?.axes.some(axis => axis.joint_configured) ? "przełożenia osi" : "wały 1:1"}</span>
            {planning && <span className="badge planning-badge">Planowanie — JOG / TCP</span>}
            <span className={`preview-completeness ${count === 6 ? "complete" : "partial"}`}>{count === 6 ? "Pozycja kompletna — 6/6 osi" : `Pozycja częściowa — ${count}/6 osi`}</span>
        </header>
        <div className="preview-body"><div className="preview-viewport" aria-label="Model 3D robota">
            <div ref={host} className="preview-canvas" />{message && <p className="preview-overlay" role="status">{message}</p>}
            <button className="preview-home" type="button" aria-label="Dopasuj widok" title="Dopasuj widok" onClick={() => scene.current?.view("fit")} disabled={!!message}>
                <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="M3 11 12 3l9 8M5 10v11h5v-7h4v7h5V10" /></svg>
            </button>
            {!planning ? <button className="preview-planning" type="button" onClick={onPlanningStart} disabled={!canStartPlanning} title="Lokalny JOG bez komend do robota">Planowanie</button> :
                <div className="preview-planning-controls"><button type="button" onClick={onPlanningSync} disabled={!canStartPlanning}>Od pozycji robota</button><button type="button" onClick={onPlanningEnd}>Zakończ planowanie</button></div>}
            {planning && <div className="preview-planning-legend"><span className="real">Robot: enkodery</span><span className="twin">Cel</span><span className="motion">Symulacja ruchu</span>{!planningReady && <span>Plan wymaga ponownej synchronizacji</span>}</div>}
        </div></div>
    </section>;
}
