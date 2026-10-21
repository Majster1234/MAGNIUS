import { useEffect, useRef, useState } from "react";
import { validateCartesianPlan, type CartesianPlan } from "./cartesianTrajectory";
import { planningRequest } from "./planningContext";

type Status = { state: string; program_id: string; mode: string; message: string; busy: boolean; progress: number;
    duration: number; start_positions: number[]; target_positions: number[]; preparation?: { state: string; preview_plan_id?: string; program_id?: string }; participating: boolean[]; tcp_verified: boolean; execution_key: string };
export function MotionExecution({ planId, disabled, onPreview }: { planId: string | null; disabled: boolean; onPreview: (plan: CartesianPlan) => void }) {
    const [status, setStatus] = useState<Status | null>(null), [message, setMessage] = useState("");
    const [preparing, setPreparing] = useState(false), [sending, setSending] = useState(false);
    const [confirmed, setConfirmed] = useState(false);
    const generation = useRef(0), key = useRef("");
    useEffect(() => {
        let closed = false, pending = false;
        const update = async () => {
            if (pending) return; pending = true;
            try {
                const value = await planningRequest("motion/status");
                if (!closed) {
                    setStatus(value);
                    setPreparing(value.preparation?.state === "planning");
                    if (value.preparation?.state === "failed") setMessage(value.preparation.message);
                }
            } catch (error) { if (!closed) setMessage((error as Error).message); }
            finally { pending = false; }
        };
        void update(); const timer = setInterval(() => void update(), 250);
        return () => { closed = true; clearInterval(timer); };
    }, []);
    useEffect(() => { generation.current++; setConfirmed(false); key.current = ""; }, [planId]);
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
        setSending(true);setMessage("");
        try { await planningRequest("motion/execute", { program_id: status.program_id, execution_key: key.current }); setConfirmed(false); }
        catch (error) { setMessage((error as Error).message + " Sprawdź stan programu przed ponowieniem."); }
        finally { setSending(false); }
    }
    async function stop() {
        setSending(true);
        try { await planningRequest("motion/stop", {}); setConfirmed(false); }
        catch (error) { setMessage((error as Error).message); }
        finally { setSending(false); }
    }
    const stateLabels: Record<string, string> = {idle: "Oczekiwanie", ready: "Gotowy", arming: "Uzbrajanie", staging: "Przygotowanie startu", running: "W ruchu", settling: "Potwierdzanie celu", succeeded: "Zakończony", failed: "Przerwany", invalidated: "Nieaktualny"};
    const ready = status?.state === "ready" && !preparing && status.preparation?.preview_plan_id === planId && status.preparation?.state === "ready" && status.preparation?.program_id === status.program_id;
    return <section className="physical-motion" aria-label="Wykonanie fizycznego PTP">
        <strong>Wykonanie fizyczne PTP</strong>
        <div className="cartesian-actions">
            <button disabled={disabled || !planId || preparing || sending || status?.busy} onClick={() => void prepare()}>Przygotuj PTP robota</button>
            <button disabled={!ready || sending} onClick={() => void preview()}>Podgląd przygotowanego PTP</button>
            <button disabled={!ready || !confirmed || sending || preparing} onClick={() => void execute()}>Wykonaj PTP</button>
            <button className="danger" onClick={() => void stop()}>STOP robota</button>
        </div>
        {status?.program_id && <p><strong>{stateLabels[status.state] || status.state}</strong> · {status.mode === "native_ptp" ? "PTP natywny" : "PTP próbkowany"} · {status.duration.toFixed(2)} s · osie: {status.participating.map((yes, i) => yes ? `J${i + 1}` : "").filter(Boolean).join(", ")} · {Math.round(status.progress * 100)}%</p>}
        {status?.program_id && <p>Start → cel: {status.participating.map((yes, i) => yes ? `J${i + 1}: ${(status.start_positions[i] * 180 / Math.PI).toFixed(2)}° → ${(status.target_positions[i] * 180 / Math.PI).toFixed(2)}°` : "").filter(Boolean).join(" · ")}</p>}
        {status?.program_id && !status.tcp_verified && <p className="motion-warning" role="status">Niepełny zestaw napędów. Ruch obejmuje wyłącznie wymienione osie. Pozycja TCP i osiągnięcie celu TCP nie są potwierdzone.</p>}
        {ready && <label><input type="checkbox" checked={confirmed} onChange={event => setConfirmed(event.target.checked)} />Zatwierdzam fizyczny ruch wymienionych osi{status?.tcp_verified ? "." : " mimo niepotwierdzonej pozycji TCP."}</label>}
        <p role="status">{preparing ? "Przygotowywanie programu z aktualnej pozycji…" : message || status?.message || "Wybierz PTP i oblicz podgląd."}</p>
        <small>Zatwierdzony ruch trwa również po zamknięciu panelu lub utracie LAN. STOP przerywa program.</small>
    </section>;
}
