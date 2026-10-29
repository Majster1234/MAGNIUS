import { useCallback, useState } from "react";
import { ControlMessages, controlSourceLabel, type CurrentMessage } from "./controlMessages";
import type { ConsoleEntry, Notice } from "./eventLog";

export function useControlMessages() {
    const [store]=useState(()=>new ControlMessages());const [,render]=useState(0);
    const notice=useCallback((value:Notice)=>{if(store.report(value))render(n=>n+1);},[store]);
    const condition=useCallback((key:string,value:Notice|null,resolution?:string)=>{if(store.condition(key,value,resolution))render(n=>n+1);},[store]);
    return {entries:store.log.entries,active:store.active,statuses:store.statuses,notice,condition};
}
function Explanation({notice}:{notice:Notice}) {
    const showReason=notice.description && notice.message.trim() && !notice.description.includes(notice.message);
    return <>{showReason && <p className="message-reason"><strong>Powód:</strong> {notice.message}</p>}<p>{notice.description ?? notice.message}</p>{notice.remedy && <p className="message-remedy"><strong>Co zrobić:</strong> {notice.remedy}</p>}
        {notice.href && <a className="message-link" href={notice.href}>Otwórz {notice.href==="#config"?"konfigurację":notice.href==="#can"?"magistralę CAN":"diagnostykę"} →</a>}
        {notice.details && <details><summary>Szczegóły techniczne</summary><pre>{notice.details}</pre></details>}</>;
}
export function ControlMessageCenter({entries,active,statuses}:{entries:ConsoleEntry[];active:CurrentMessage[];statuses:Notice[]}) {
    return <section className="control-message-center" aria-label="Centrum komunikatów">
        <header><div><h2>Centrum komunikatów</h2><small>Stan sterowania i wskazówki operatora</small></div><span className={`badge ${active.some(n=>n.severity==="error")?"error":active.length?"warn":"ok"}`}>{active.length?`${active.length} do sprawdzenia`:"Bez zgłoszonych problemów"}</span></header>
        <div className="control-current" aria-label="Aktualny stan sterowania">
            {statuses.filter(n=>n.severity==="info").map(n=><div className="control-state-line" key={n.source}><span>{n.context}</span><strong>{n.title}</strong>{typeof n.progress==="number" && <span aria-label="Postęp obliczeń">{n.progress}%</span>}</div>)}
        </div>
        <div className="control-active" aria-label="Aktualne problemy">
            {active.map(n=><article key={`${n.kind}/${n.source}`} id={`control-issue-${n.source}`} data-source={n.source} className={`control-issue issue-${n.severity}`}>
                <div className="issue-heading"><span className={`badge ${n.severity==="error"?"error":"warn"}`}>{n.severity==="error"?"BŁĄD":"UWAGA"}</span><small>{n.context} · {controlSourceLabel(n.source)} · {n.kind==="condition"?"Warunek do sprawdzenia":"Wynik ostatniej operacji"}</small></div>
                <h3>{n.title}</h3><Explanation notice={n}/>
            </article>)}
        </div>
        <span className="sr-only" role="alert" aria-live="assertive">{active.filter(n=>n.severity==="error").map(n=>`${n.context}: ${n.title}`).join(". ")}</span>
        <span className="sr-only" role="status" aria-live="polite">{entries.at(-1)?.severity!=="error"?entries.at(-1)?.title ?? entries.at(-1)?.message:""}</span>
        <details className="control-history"><summary>Historia zdarzeń <span>{entries.length}/100</span></summary>
            <div role="log" aria-label="Historia komunikatów sterowania" aria-live="off" tabIndex={0}>
                {[...entries].reverse().map(n=><details key={n.id} className={`history-entry issue-${n.severity}`}><summary>
                    <time dateTime={new Date(n.timestamp).toISOString()}>{new Date(n.timestamp).toLocaleTimeString("pl-PL",{hour12:false,hour:"2-digit",minute:"2-digit",second:"2-digit",fractionalSecondDigits:3})}</time>
                    <span>{n.severity==="error"?"BŁĄD":n.severity==="warning"?"UWAGA":"INFO"} · {n.context ?? controlSourceLabel(n.source)} · {controlSourceLabel(n.source)}</span><strong>{n.title ?? n.message}</strong></summary><Explanation notice={n}/></details>)}
            </div>
        </details>
    </section>;
}
