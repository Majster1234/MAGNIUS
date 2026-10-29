import { useCallback, useEffect, useRef, useState } from "react";
import { validateCartesianPlan, type CartesianPlan } from "./cartesianTrajectory";
import { planningRequest, PlanningRequestError } from "./planningContext";
import { MOTION_STATUS_MAX_AGE_MS, motionReady, motionStateLabels, motionStatusFresh, type MotionObservation, type MotionStatus } from "./motionStatus";
import type { Notice } from "./eventLog";

type Props={planId:string|null;motionType:"PTP"|"LIN"|null;disabled:boolean;onPreview:(plan:CartesianPlan)=>void;
    onStatus?:(value:MotionObservation|null)=>void;onNotice?:(value:Notice)=>void;
    onCondition?:(key:string,value:Notice|null)=>void;onStopReady?:(action:(()=>void)|null)=>void};
export function MotionExecution({planId,motionType,disabled,onPreview,onStatus,onNotice,onCondition,onStopReady}:Props) {
    const [status,setStatus]=useState<MotionStatus|null>(null),[preparing,setPreparing]=useState(false),[sending,setSending]=useState(false);
    const [confirmed,setConfirmed]=useState(false),[receivedAt,setReceivedAt]=useState(-Infinity),[clock,setClock]=useState(performance.now()),[connectionError,setConnectionError]=useState("");
    const [uncertain,setUncertain]=useState<{program:string;key:string}|null>(null);
    const uncertainty=useRef(uncertain);uncertainty.current=uncertain;
    const expected=useRef<string|null>(null),epoch=useRef(0),key=useRef(""),stopAck=useRef<number|null>(null);
    const operatorStopped=useRef<string|null>(null),planKey=useRef("");planKey.current=`${planId}/${motionType}`;
    const emit=useCallback((source:string,message:string,severity:Notice["severity"]="info",code?:string)=>onNotice?.({source,message,severity,code,context:"Robot fizyczny"}),[onNotice]);
    const receive=useCallback((value:MotionStatus,requestedAt=performance.now())=>{
        if(typeof value?.state!=="string" || typeof value.program_id!=="string" || typeof value.busy!=="boolean" ||
            !Number.isFinite(value.duration) || !Number.isFinite(value.progress) || !Array.isArray(value.participating) || value.participating.length!==6 || value.participating.some(v=>typeof v!=="boolean") ||
            ![value.start_positions,value.target_positions].every(q=>Array.isArray(q) && q.length===6 && q.every(Number.isFinite)))throw Error("Nieprawidłowy stan wykonawcy");
        const now=performance.now();setStatus(value);setReceivedAt(now);setClock(now);setConnectionError("");onStatus?.({status:value,receivedAt:now});
        const matching=!expected.current || value.preparation?.id===expected.current;
        if(matching) {
            setPreparing(value.preparation?.state==="planning");
            if(value.preparation?.state==="failed")emit("motion-prepare",value.preparation.message || "Przygotowanie programu nie powiodło się","error");
            else if(value.preparation?.state==="ready")emit("motion-prepare","Program przygotowany. Sprawdź osie, start i cel przed zatwierdzeniem.");
            if(value.preparation?.state==="planning")emit("motion-preparation",`Przygotowywanie fizycznego ${value.preparation.motion_type ?? "programu"} z aktualnych pomiarów…`);
            else if(value.preparation?.state)emit("motion-preparation",value.preparation.state==="ready"?"Przygotowanie zakończone":value.preparation.state==="failed"?"Przygotowanie odrzucone":"Oczekiwanie na przygotowanie");
        }
        emit("motion-state",!matching && !value.busy?"Trwa przygotowanie nowego programu":`${value.motion_type ?? "Program"} · ${motionStateLabels[value.state] ?? "Nieznany stan"}`);
        const pending=uncertainty.current;
        if(pending && ((value.program_id===pending.program && value.execution_key===pending.key && value.state!=="ready") || (value.program_id && value.program_id!==pending.program))) {
            setUncertain(null);emit("motion-execute","Odczytano potwierdzony stan programu po żądaniu startu.");
        }
        if(stopAck.current!==null && requestedAt>=stopAck.current && !value.busy && ["idle","failed","invalidated","succeeded"].includes(value.state)) {
            stopAck.current=null;operatorStopped.current=value.program_id;setUncertain(null);emit("motion-stop","Wykonawca potwierdził nieaktywny program. To status wykonawcy, nie pomiar prędkości wałów.");
            emit("motion-execute","Stan wykonawcy uzgodniony po STOP.");
        }
    },[emit,onStatus]);
    useEffect(()=>{
        let closed=false,pending=false;
        async function update() {
            if(pending)return;pending=true;const current=epoch.current,requestedAt=performance.now();
            try {const value=await planningRequest("motion/status");if(!closed && current===epoch.current)receive(value,requestedAt);}
            catch(error){if(!closed && current===epoch.current){setConnectionError((error as Error).message);setConfirmed(false);onStatus?.(null);}}
            finally{pending=false;}
        }
        void update();const timer=setInterval(()=>void update(),250),ticker=setInterval(()=>setClock(performance.now()),100);
        return()=>{closed=true;clearInterval(timer);clearInterval(ticker);};
    },[receive,onStatus]);
    const fresh=motionStatusFresh(receivedAt,clock,!!connectionError);
    useEffect(()=>{setConfirmed(false);key.current="";},[planId,motionType,status?.program_id,disabled]);
    useEffect(()=>{if(!fresh)setConfirmed(false);onCondition?.("motion-connection",fresh?null:{source:"motion-connection",severity:"warning",message:connectionError || "Brak statusu wykonawcy młodszego niż 750 ms",code:"executor",context:"Robot fizyczny"});},[fresh,connectionError,onCondition]);
    useEffect(()=>{
        if(fresh)for(const code of ["waiting_position","waiting_feedback","waiting_budget"])
            onCondition?.(`motion-${code}`,fresh && status?.state===code?{source:`motion-${code}`,severity:"warning",message:status.message,code,context:"Robot fizyczny"}:null);
        onCondition?.("motion-partial",status?.program_id && !status.tcp_verified?{source:"motion-partial",severity:"warning",code:"partial",title:status.motion_type==="LIN"?"Próba LIN — tor TCP nie jest potwierdzony":"Niepełny zestaw napędów programu",message:"Niepełny zestaw napędów programu.",description:`Ruch obejmuje wyłącznie osie: ${status.participating.map((yes,i)=>yes?`J${i+1}`:"").filter(Boolean).join(", ")}. Pozycja i osiągnięcie celu TCP nie są potwierdzone.`,context:"Robot fizyczny"}:null);
        if(fresh){onCondition?.("motion-failed",status?.state==="failed" && !stopAck.current && operatorStopped.current!==status.program_id?{source:"motion-failed",severity:"error",message:status.message,context:"Robot fizyczny",title:"Program został przerwany"}:null);
        onCondition?.("motion-invalidated",status?.state==="invalidated"?{source:"motion-invalidated",severity:"warning",message:status.message,code:"program",context:"Robot fizyczny"}:null);}
    },[fresh,status,onCondition]);
    async function prepare() {
        if(disabled || !fresh || performance.now()-receivedAt>=MOTION_STATUS_MAX_AGE_MS || !planId || !motionType || uncertain)return;
        const current=++epoch.current;expected.current=`pending-${current}`;setSending(true);setPreparing(true);setConfirmed(false);
        emit("motion-prepare","Wysłano żądanie przygotowania programu.");
        try {
            const result=await planningRequest("motion/prepare",{plan_id:planId});if(current!==epoch.current)return;
            if(typeof result.id!=="string")throw Error("Brak identyfikatora przygotowania programu");
            expected.current=result.id;setPreparing(result.state==="planning");
            emit("motion-preparation",`Przygotowywanie fizycznego ${motionType ?? "programu"} z aktualnych pomiarów…`);
        } catch(error){if(current===epoch.current){setPreparing(false);emit("motion-prepare",(error as Error).message,"error");}}
        finally{if(current===epoch.current)setSending(false);}
    }
    async function preview() {
        const program=status?.program_id,current=epoch.current,selected=planKey.current;
        try {
            const result=await planningRequest("motion/prepared");if(current!==epoch.current || selected!==planKey.current)return;validateCartesianPlan(result.plan);
            if(result.program_id!==program || result.plan.simulation_only)throw Error("Podgląd programu jest nieaktualny");
            onPreview(result.plan);emit("motion-preview","Wyświetlono przygotowany program. Podgląd nie uruchamia robota.");
        } catch(error){if(current===epoch.current)emit("motion-preview",(error as Error).message,"error","program");}
    }
    const ready=!disabled && !preparing && !sending && !uncertain && motionReady(status,planId,motionType,expected.current,fresh);
    async function execute() {
        if(!ready || !confirmed || !status || performance.now()-receivedAt>=MOTION_STATUS_MAX_AGE_MS)return;
        if(!key.current)key.current=Array.from(crypto.getRandomValues(new Uint8Array(16)),b=>b.toString(16).padStart(2,"0")).join("");
        const current=++epoch.current,attempt={program:status.program_id,key:key.current};setSending(true);setConfirmed(false);
        try {
            const value=await planningRequest("motion/execute",{program_id:attempt.program,execution_key:attempt.key});if(current!==epoch.current)return;
            receive(value);emit("motion-execute","Żądanie startu przyjęte. Obserwuj aktualny stan programu.");
        }catch(error){if(current===epoch.current){
            const rejected=error instanceof PlanningRequestError && error.status>=400 && error.status<500 && ![408,499].includes(error.status);
            if(rejected)expected.current=`rejected-${current}`;else setUncertain(attempt);
            emit("motion-execute",(error as Error).message,"error",rejected?undefined:"uncertain_start");
        }}
        finally{if(current===epoch.current)setSending(false);}
    }
    async function stop() {
        const current=++epoch.current;setSending(true);setConfirmed(false);setPreparing(false);onStatus?.(null);
        emit("motion-stop","Wysłano żądanie STOP. Oczekiwanie na odpowiedź wykonawcy.");
        try {
            const result=await planningRequest("motion/stop",{});if(current!==epoch.current)return;
            if(result.stopped!==true)throw Error("Brak potwierdzenia żądania STOP");
            stopAck.current=performance.now();emit("motion-stop","Żądanie STOP przyjęte. Oczekiwanie na aktualny stan wykonawcy.");
        }catch(error){if(current===epoch.current)emit("motion-stop",(error as Error).message,"error","stop");}
        finally{if(current===epoch.current)setSending(false);}
    }
    const selectedType=motionType ?? "ruch";
    const stopAction=useRef(stop);stopAction.current=stop;
    useEffect(()=>{onStopReady?.(()=>void stopAction.current());return()=>onStopReady?.(null);},[onStopReady]);
    return <section className="physical-motion" aria-label="Wykonanie fizycznego PTP/LIN">
        <header className="motion-stage-heading"><span className="motion-stage-number">2</span><div><h3>Przygotowanie robota</h3><small>Aktualne pomiary → program fizyczny</small></div><span className="badge warn">ROBOT</span></header>
        <div className="motion-prepare-actions"><button className="secondary-button" disabled={disabled || !fresh || !planId || !motionType || preparing || sending || !!uncertain || status?.busy} onClick={()=>void prepare()}>Przygotuj {selectedType} robota</button>
            <button className="secondary-button" disabled={!ready} onClick={()=>void preview()}>Podgląd przygotowanego {selectedType}</button></div>
        <div className="motion-summary" aria-label="Podsumowanie programu" data-program-id={status?.program_id ?? ""}>
            <span className={`badge ${fresh?status?.busy?"planning-badge":"ok":"stale"}`}>{!fresh?"Status nieaktualny":preparing?"Przygotowywanie":expected.current && status?.preparation?.id!==expected.current?"Oczekiwanie na aktualny program":motionStateLabels[status?.state ?? "idle"] ?? "Nieznany stan"}</span>
            {status?.program_id && <><dl><div><dt>Wysyłanie</dt><dd>{status.dispatch_mode === "synchronized" ? "Streaming synchroniczny" : "Streaming"}</dd></div><div><dt>Typ</dt><dd>{status.motion_type ?? "—"}</dd></div><div><dt>Plan</dt><dd>{status.duration.toFixed(2)} s</dd></div><div><dt>Czas</dt><dd>{(status.elapsed ?? 0).toFixed(2)} s</dd></div><div><dt>Osie</dt><dd>{status.participating.map((yes,i)=>yes?`J${i+1}`:"").filter(Boolean).join(", ") || "—"}</dd></div>{status.dispatch_mode === "synchronized" && <><div><dt>Cykl SYNC</dt><dd>{status.sync_cycle ?? 0}</dd></div><div><dt>Spóźnienie SYNC</dt><dd>{(status.sync_lateness_ms ?? 0).toFixed(3)} ms</dd></div></>}</dl>
                <div className="motion-progress"><progress aria-label="Postęp fizycznego programu" max={1} value={Math.max(0,Math.min(1,status.progress))}/><span>{Math.round(status.progress*100)}%</span></div>
                <details><summary>Pozycje osi · start → cel</summary><table><thead><tr><th>Oś</th><th>Start [°]</th><th>Cel [°]</th></tr></thead><tbody>{status.participating.map((yes,i)=>yes && <tr key={i}><th>J{i+1}</th><td>{(status.start_positions[i]*180/Math.PI).toFixed(2)}</td><td>{(status.target_positions[i]*180/Math.PI).toFixed(2)}</td></tr>)}</tbody></table></details></>}
        </div>
        <header className="motion-stage-heading"><span className="motion-stage-number">3</span><div><h3>Zatwierdzenie i wykonanie</h3><small>Sprawdź zakres i gotowość stanowiska</small></div></header>
        <label className="motion-confirm"><input type="checkbox" checked={confirmed} disabled={!ready} onChange={e=>setConfirmed(e.target.checked)}/>Zatwierdzam fizyczny ruch wymienionych osi{status?.tcp_verified?".":" mimo niepotwierdzonej pozycji TCP."}</label>
        <small className="motion-persistence">Program trwa po zamknięciu panelu i utracie LAN. Szczegóły stanu oraz rozwiązania problemów: Centrum komunikatów.</small>
        <div className="motion-action-bar"><button className="primary-button" disabled={!ready || !confirmed} onClick={()=>void execute()}>Wykonaj {selectedType}</button></div>
    </section>;
}
