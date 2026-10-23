import {useEffect,useRef,useState,type KeyboardEvent,type PointerEvent,type ReactNode} from "react";
import {TraceButton,TraceIcon} from "./TraceButton";
import {fitTraceWindow,initialTraceWindow,maximizedTraceWindow,type TraceWindowRect} from "./traceWindowGeometry";

const storageKey="robot-shaft-trace-window-v2";
const viewport=()=>({width:window.innerWidth,height:window.innerHeight});
function loadRect() {
    try {
        const rect=JSON.parse(localStorage.getItem(storageKey)??"null");
        if(rect&&[rect.x,rect.y,rect.width,rect.height].every(Number.isFinite)) return fitTraceWindow(rect,viewport());
    } catch {/* Pamięć lokalna jest opcjonalna. */}
    return initialTraceWindow(viewport());
}
export function TraceWindow({axis,controls,children,onDock,onClose}:{axis:number;controls:ReactNode;children:ReactNode;onDock:()=>void;onClose:()=>void}) {
    const [rect,setRect]=useState(loadRect),[maximized,setMaximized]=useState(false),[screen,setScreen]=useState(viewport);
    const moveHandle=useRef<HTMLDivElement>(null);
    const drag=useRef<{pointer:number;x:number;y:number;rect:TraceWindowRect;resize:boolean}|null>(null);
    const shown=maximized?maximizedTraceWindow(screen):fitTraceWindow(rect,screen);
    useEffect(()=>{
        moveHandle.current?.focus({preventScroll:true});
        const resize=()=>{const next=viewport();setScreen(next);setRect(old=>fitTraceWindow(old,next));};
        window.addEventListener("resize",resize);return ()=>window.removeEventListener("resize",resize);
    },[]);
    useEffect(()=>{try {localStorage.setItem(storageKey,JSON.stringify(rect));} catch {/* Brak zapisu nie blokuje okna. */}},[rect]);
    function begin(event:PointerEvent<HTMLElement>,resize:boolean) {
        if(maximized||event.button!==0||!event.isPrimary) return;
        event.preventDefault();event.currentTarget.focus({preventScroll:true});
        drag.current={pointer:event.pointerId,x:event.clientX,y:event.clientY,rect:shown,resize};
        event.currentTarget.setPointerCapture(event.pointerId);
    }
    function move(event:PointerEvent<HTMLElement>) {
        const anchor=drag.current;if(!anchor||anchor.pointer!==event.pointerId) return;
        if(!(event.buttons&1)) {drag.current=null;return;}
        const dx=event.clientX-anchor.x,dy=event.clientY-anchor.y;
        setRect(fitTraceWindow(anchor.resize?{...anchor.rect,width:anchor.rect.width+dx,height:anchor.rect.height+dy}:
            {...anchor.rect,x:anchor.rect.x+dx,y:anchor.rect.y+dy},viewport()));
    }
    function finish(event:PointerEvent<HTMLElement>) {
        if(drag.current?.pointer!==event.pointerId) return;
        drag.current=null;
        if(event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
    }
    function keyboard(event:KeyboardEvent<HTMLElement>,resize:boolean) {
        if(maximized||!["ArrowLeft","ArrowRight","ArrowUp","ArrowDown"].includes(event.key)) return;
        event.preventDefault();const step=event.shiftKey?50:10;
        const dx=event.key==="ArrowLeft"?-step:event.key==="ArrowRight"?step:0,dy=event.key==="ArrowUp"?-step:event.key==="ArrowDown"?step:0;
        setRect(old=>fitTraceWindow(resize?{...old,width:old.width+dx,height:old.height+dy}:{...old,x:old.x+dx,y:old.y+dy},viewport()));
    }
    const gestures={onPointerMove:move,onPointerUp:finish,onPointerCancel:finish,onLostPointerCapture:()=>{drag.current=null;}};
    return <div className="trace-window" role="dialog" aria-label={`Przypięty wykres J${axis}`} style={{left:shown.x,top:shown.y,width:shown.width,height:shown.height}} data-maximized={maximized}>
        <header className="trace-window-header">
            <div className="trace-window-titlebar">
                <div ref={moveHandle} className="trace-window-move" role="button" tabIndex={0} aria-label="Przesuń okno wykresów"
                    aria-disabled={maximized} title="Przeciągnij, aby przesunąć okno; strzałki przesuwają o 10 px, Shift o 50 px"
                    onPointerDown={e=>begin(e,false)} onKeyDown={e=>keyboard(e,false)} {...gestures}>
                    <span aria-hidden="true">⠿</span><h2>Diagnostyka · J{axis}</h2>
                </div>
                <div className="trace-window-actions" role="group" aria-label="Sterowanie oknem wykresów">
                    <TraceButton action="dock" label="Wróć do modułu Diagnostyka" onClick={onDock}/>
                    <TraceButton action={maximized?"restore":"maximize"} label={maximized?"Przywróć rozmiar okna":"Powiększ okno wykresów"} onClick={()=>{drag.current=null;setMaximized(v=>!v);}}/>
                    <TraceButton action="close" label="Zamknij pływające okno" onClick={onClose}/>
                </div>
            </div>
            {controls}
        </header>
        <div className="trace-window-body">{children}</div>
        {!maximized&&<button type="button" className="trace-window-resize" aria-label="Zmień rozmiar okna wykresów"
            title="Przeciągnij narożnik lub użyj strzałek, aby zmienić rozmiar" onPointerDown={e=>begin(e,true)} onKeyDown={e=>keyboard(e,true)} {...gestures}><TraceIcon action="resize"/></button>}
    </div>;
}
