export type TraceAction="zoom-in"|"zoom-out"|"earlier"|"later"|"pause"|"live"|"fit"|"pin"|"dock"|"maximize"|"restore"|"close"|"resize";
export function TraceIcon({action}:{action:TraceAction}) {
    return <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true" focusable="false">
        {(action==="zoom-in"||action==="zoom-out")&&<><circle cx="10" cy="10" r="6"/><path d="m14.5 14.5 5 5M7 10h6"/>{action==="zoom-in"&&<path d="M10 7v6"/>}</>}
        {action==="earlier"&&<path d="M19 12H5m6-6-6 6 6 6"/>}
        {action==="later"&&<path d="M5 12h14m-6-6 6 6-6 6"/>}
        {action==="pause"&&<><path d="M8 5v14M16 5v14" strokeWidth="3"/></>}
        {action==="live"&&<><circle cx="12" cy="12" r="2" fill="currentColor"/><path d="M8 8a5.7 5.7 0 0 0 0 8m8-8a5.7 5.7 0 0 1 0 8M5 5a10 10 0 0 0 0 14M19 5a10 10 0 0 1 0 14"/></>}
        {action==="fit"&&<path d="M4 4v16M20 4v16M7 12h10m-7-3-3 3 3 3m4-6 3 3-3 3"/>}
        {action==="pin"&&<path d="m15 3 6 6-5 1-3 5-4-4-5 1 3-5 1-4 7 0ZM10 14l-7 7"/>}
        {action==="dock"&&<><rect x="3" y="4" width="18" height="16" rx="2"/><path d="M3 9h18m-9 3v6m-3-3 3 3 3-3"/></>}
        {action==="maximize"&&<rect x="4" y="4" width="16" height="16" rx="1"/>}
        {action==="restore"&&<><path d="M8 8V4h12v12h-4"/><rect x="4" y="8" width="12" height="12" rx="1"/></>}
        {action==="close"&&<path d="m6 6 12 12M6 18 18 6"/>}
        {action==="resize"&&<path d="m7 19 12-12m-5 12 5-5"/>}
    </svg>;
}
export function TraceButton({action,label,title,onClick,active,disabled}:{action:TraceAction;label:string;title?:string;onClick:()=>void;active?:boolean;disabled?:boolean}) {
    return <button type="button" className={`trace-icon-button ${active?"primary-button":"secondary-button"}`} aria-label={label} title={title??label}
        aria-pressed={active} disabled={disabled} onClick={onClick}><TraceIcon action={action}/></button>;
}
