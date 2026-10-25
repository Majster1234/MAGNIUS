import { useCallback, useEffect, useRef, useState } from "react";
import { validateCartesianPlan, type CartesianPlan } from "./cartesianTrajectory";
import { planningRequest } from "./planningContext";
import type { MotionObservation, MotionStatus } from "./motionStatus";

export function MotionExecution({ planId, motionType, disabled, onPreview, onStatus }: { planId: string | null; motionType: "PTP" | "LIN" | null; disabled: boolean; onPreview: (plan: CartesianPlan) => void; onStatus?: (value: MotionObservation | null) => void }) {
    const [status, setStatus] = useState<MotionStatus | null>(null), [message, setMessage] = useState("");
    const [preparing, setPreparing] = useState(false), [sending, setSending] = useState(false);
    const [confirmed, setConfirmed] = useState(false);
    const generation = useRef(0), key = useRef("");
    const statusEpoch = useRef(0);
    const receive = useCallback((value: MotionStatus) => {
        setStatus(value); onStatus?.({ status: value, receivedAt: performance.now() });
        setPreparing(value.preparation?.state === "planning");
        if (value.preparation?.state === "failed") setMessage(value.preparation.message || "Przygotowanie nie powiodło się");
    }, [onStatus]);
    useEffect(() => {
        let closed = false, pending = false;
        const update = async () => {
            if (pending) return; pending = true;
            const epoch = statusEpoch.current;
            try {
                const value = await planningRequest("motion/status");
                if (!closed && epoch === statusEpoch.current) receive(value);
            } catch (error) { if (!closed && epoch === statusEpoch.current) { setMessage((error as Error).message); onStatus?.(null); } }
            finally { pending = false; }
        };
        void update(); const timer = setInterval(() => void update(), 250);
        return () => { closed = true; clearInterval(timer); };
    }, [receive, onStatus]);
    useEffect(() => { generation.current++; setConfirmed(false); key.current = ""; }, [planId, motionType, status?.program_id]);
    async function prepare() {
        setSending(true); setConfirmed(false); setMessage("");
        try { await planningRequest("motion/prepare", { plan_id: planId }); setPreparing(true); }
        catch (error) { setMessage((error as Error).message); }
        finally { setSending(false); }
    }
    async function preview() {
        try {
            const result = await planningRequest("motion/prepared");validateCartesianPlan(result.plan);
            if (result.program_id !== status?.program_id || result.plan.simulation_only) throw Error("Podgląd programu jest nieaktualny");
            onPreview(result.plan);
        } catch (error) { setMessage((error as Error).message); }
    }
    async function execute() {
        if (!status || !confirmed) return;
        if (!key.current) key.current = Array.from(crypto.getRandomValues(new Uint8Array(16)), byte => byte.toString(16).padStart(2, "0")).join("");
        setSending(true);setMessage(""); statusEpoch.current++;
        try { receive(await planningRequest("motion/execute", { program_id: status.program_id, execution_key: key.current })); setConfirmed(false); }
        catch (error) { setMessage((error as Error).message + " Sprawdź stan programu przed ponowieniem."); }
        finally { setSending(false); }
    }
    async function stop() {
        setSending(true); statusEpoch.current++; onStatus?.(null);
        try { await planningRequest("motion/stop", {}); setConfirmed(false); }
        catch (error) { setMessage((error as Error).message); }
        finally { setSending(false); }
    }
    const stateLabels: Record<string, string> = {idle: "Oczekiwanie", ready: "Gotowy", arming: "Uzbrajanie", warming: "Odczyt pozycji startowej", staging: "Przygotowanie startu", running: "W ruchu", waiting_position: "Oczekiwanie na pozycję osi", waiting_feedback: "Oczekiwanie na pomiary", waiting_budget: "Oczekiwanie na budżet CAN", settling: "Potwierdzanie celu", succeeded: "Zakończony", failed: "Przerwany", invalidated: "Nieaktualny"};
    const ready = !disabled && status?.state === "ready" && !preparing && status.preparation?.preview_plan_id === planId && status.preparation?.state === "ready" && status.preparation?.program_id === status.program_id && status.motion_type === motionType && status.preparation.motion_type === motionType;
    const selectedType = motionType ?? "ruch", programType = status?.motion_type ?? "Ruch";
    return <section className="physical-motion" aria-label="Wykonanie fizycznego PTP/LIN">
        <strong>Wykonanie fizyczne PTP/LIN</strong>
        <div className="cartesian-actions">
            <button disabled={disabled || !planId || !motionType || preparing || sending || status?.busy} onClick={() => void prepare()}>Przygotuj {selectedType} robota</button>
            <button disabled={!ready || sending} onClick={() => void preview()}>Podgląd przygotowanego {selectedType}</button>
            <button disabled={!ready || !confirmed || sending || preparing} onClick={() => void execute()}>Wykonaj {selectedType}</button>
            <button className="danger" onClick={() => void stop()}>STOP robota</button>
        </div>
        {status?.program_id && <p><strong>{stateLabels[status.state] || status.state}</strong> · {programType} próbkowany · plan: {status.duration.toFixed(2)} s{typeof status.elapsed === "number" ? ` · czas: ${status.elapsed.toFixed(2)} s` : ""} · osie: {status.participating.map((yes, i) => yes ? `J${i + 1}` : "").filter(Boolean).join(", ")} · {Math.round(status.progress * 100)}%</p>}
        {status?.program_id && <p>Start → cel: {status.participating.map((yes, i) => yes ? `J${i + 1}: ${(status.start_positions[i] * 180 / Math.PI).toFixed(2)}° → ${(status.target_positions[i] * 180 / Math.PI).toFixed(2)}°` : "").filter(Boolean).join(" · ")}</p>}
        {status?.program_id && !status.tcp_verified && <p className="motion-warning" role="status">Niepełny zestaw napędów. {status.motion_type === "LIN" ? "Próba LIN — tor TCP nie jest potwierdzony. " : ""}Ruch obejmuje wyłącznie wymienione osie. Pozycja TCP i osiągnięcie celu TCP nie są potwierdzone.</p>}
        {ready && <label><input type="checkbox" checked={confirmed} onChange={event => setConfirmed(event.target.checked)} />Zatwierdzam fizyczny ruch wymienionych osi{status?.tcp_verified ? "." : " mimo niepotwierdzonej pozycji TCP."}</label>}
        <p role="status">{preparing ? "Przygotowywanie programu z aktualnej pozycji…" : message || status?.message || "Wybierz PTP lub LIN i oblicz podgląd."}</p>
        <small>Zatwierdzony ruch trwa również po zamknięciu panelu lub utracie LAN. Oczekiwanie na pozycję wydłuża czas wykonania. STOP przerywa program.</small>
    </section>;
}
