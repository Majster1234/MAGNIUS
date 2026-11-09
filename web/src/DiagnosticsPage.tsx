import {useEffect,useMemo,useRef,useState} from "react";
import {emptyScales,historyChannel,mergeTrace,positionText,reduceTrace,traceBase,traceChannelCount,traceGroup,valueAt,yRange,type ShaftFrame,type ShaftHistory,type ShaftPoint,type ShaftStatus,type YScale} from "./shaftTrace";
import {createPortal} from "react-dom";
import {TraceButton} from "./TraceButton";
import {TraceWindow} from "./TraceWindow";
import {ShaftHistoryCache} from "./shaftHistoryCache";

const colors=["#1aa79c","#4e8ee8","#d99829","#b16bd6","#e47961","#6caa46"];
function localTime(value:number) {const d=new Date(value);return Number.isFinite(d.getTime())?new Date(value-d.getTimezoneOffset()*60000).toISOString().slice(0,23):"";}
function timeText(value:number|null|undefined) {return value==null?"—":new Date(value).toLocaleString("pl-PL");}
function loadSettings() {
    try {
        const v=JSON.parse(localStorage.getItem("robot-shaft-trace-view-v1")??"null");
        if(v && Number.isFinite(v.seconds) && v.seconds>0 && v.scales?.length===6 && v.scales.every((s:YScale)=>typeof s.auto==="boolean" && Number.isFinite(s.minimum) && Number.isFinite(s.maximum) && s.minimum<s.maximum)) return v as {seconds:number;scales:YScale[]};
    } catch { /* Niedostępna pamięć przeglądarki nie blokuje diagnostyki. */ }
    return {seconds:10,scales:emptyScales()};
}
/** Widoczność kanałów dodatkowych; zapis i historia obejmują je zawsze. */
type Overlays={reference:boolean;command:boolean;rpm:boolean};
const overlayLabels:[keyof Overlays,string,string][]=[
    ["reference","Referencja planowana","Cel planowany PTP/LIN/JOG TCP — linia przerywana w kolorze osi"],
    ["command","Cel F5 (wysłany)","Pozycja docelowa z ramki F5 faktycznie wysłanej do napędu (z wyprzedzeniem i dolotem) — linia kropkowana"],
    ["rpm","RPM F5","Prędkość wału z ramki F5 — cienka szara linia, prawa oś"],
];
function loadOverlays():Overlays {
    try {
        const v=JSON.parse(localStorage.getItem("robot-shaft-trace-overlays-v1")??"null");
        if(v && ["reference","command","rpm"].every(k=>typeof v[k]==="boolean")) return v as Overlays;
    } catch { /* Domyślnie jak dotąd: tylko referencja. */ }
    return {reference:true,command:false,rpm:false};
}
function line(c:CanvasRenderingContext2D,points:ShaftPoint[],x:(t:number)=>number,y:(q:number)=>number,maximum:number) {
    c.beginPath();let previous:ShaftPoint|null=null;
    for(const p of reduceTrace(points,maximum)) {
        if(!previous||p.gap||p.segment!==previous.segment) c.moveTo(x(p.time_ms),y(p.position_cdeg/100));
        else c.lineTo(x(p.time_ms),y(p.position_cdeg/100));
        previous=p;
    }
    c.stroke();
}
function ShaftChart({axis,points,reference,command,rpm,start,end,scale,cursor,loading,detached,onCursor,onPan}:{axis:number;points:ShaftPoint[];reference:ShaftPoint[]|null;command:ShaftPoint[]|null;rpm:ShaftPoint[]|null;start:number;end:number;scale:YScale;cursor:number|null;loading:boolean;detached:boolean;onCursor:(value:number|null)=>void;onPan:(end:number)=>void}) {
    // Margines prawy na oś RPM tylko wtedy, gdy kanał jest włączony.
    const right=rpm?52:12,margin=64+right;
    const canvas=useRef<HTMLCanvasElement>(null);
    const drag=useRef<{pointer:number;x:number;end:number;span:number;width:number;moved:boolean}|null>(null);
    const wheelView=useRef({start,end,onPan,margin});wheelView.current={start,end,onPan,margin};
    const [dragging,setDragging]=useState(false);
    function releaseDrag() {drag.current=null;setDragging(false);}
    useEffect(()=>{
        const element=canvas.current!;
        function wheel(event:WheelEvent) {
            if(event.ctrlKey||event.metaKey||!Number.isFinite(event.deltaX)||event.deltaX===0||Math.abs(event.deltaX)<Math.abs(event.deltaY)||drag.current) return;
            const view=wheelView.current,width=Math.max(1,element.clientWidth-view.margin);
            const pixels=event.deltaX*(event.deltaMode===1?16:event.deltaMode===2?width:1);
            const shift=pixels/width*(view.end-view.start);
            event.preventDefault();
            // Sumowanie także wtedy, gdy kilka zdarzeń przyjdzie przed renderem Reacta.
            wheelView.current={...view,start:view.start+shift,end:view.end+shift};
            view.onPan(view.end+shift);
        }
        // Natywny listener pozwala zatrzymać przewijanie strony dla poziomej rolki.
        element.addEventListener("wheel",wheel,{passive:false});
        return ()=>element.removeEventListener("wheel",wheel);
    },[]);
    useEffect(()=>{
        const element=canvas.current!;
        function draw() {
            const width=element.clientWidth,height=element.clientHeight,dpr=devicePixelRatio||1;
            if(!width||!height) return;
            element.width=Math.round(width*dpr);element.height=Math.round(height*dpr);
            const c=element.getContext("2d")!;c.scale(dpr,dpr);c.clearRect(0,0,width,height);
            const left=64,top=12,w=Math.max(1,width-left-right),h=Math.max(1,height-top-28);
            const inside=(p:ShaftPoint)=>p.time_ms>=start && p.time_ms<=end;
            const visible=points.filter(inside),planned=(reference??[]).filter(inside),sent=(command??[]).filter(inside),speed=(rpm??[]).filter(inside);
            const [low,high]=yRange([...visible,...planned,...sent],scale);
            const x=(t:number)=>left+(t-start)/(end-start)*w,y=(q:number)=>top+(high-q)/(high-low)*h;
            c.font="11px sans-serif";c.textAlign="right";
            for(let i=0;i<=4;i++) {
                const q=low+(high-low)*i/4,py=y(q);c.strokeStyle="#283648";c.lineWidth=1;c.beginPath();c.moveTo(left,py);c.lineTo(left+w,py);c.stroke();
                c.fillStyle="#9aaaba";c.fillText(q.toFixed(2),left-6,py+4);
            }
            c.textAlign="center";
            for(let i=0;i<=4;i++) {const t=start+(end-start)*i/4;c.fillText(((t-start)/1000).toFixed(1)+"s",x(t),height-5);}
            // RPM na własnej osi po prawej: od 0, bo F5 niesie moduł prędkości.
            const rpmHigh=Math.max(1,...speed.map(p=>p.position_cdeg/100))*1.1,ry=(v:number)=>top+(1-v/rpmHigh)*h;
            if(rpm) {c.textAlign="left";for(let i=0;i<=4;i++) c.fillText((rpmHigh*i/4).toFixed(rpmHigh<10?1:0),left+w+6,ry(rpmHigh*i/4)+4);c.fillText("RPM",left+w+6,top-2);}
            const maximum=Math.max(2,Math.floor(w*2));
            c.save();c.beginPath();c.rect(left,top,w,h);c.clip();
            if(rpm) {c.strokeStyle="rgba(154,170,186,.75)";c.lineWidth=1;line(c,speed,x,ry,maximum);}
            c.strokeStyle=colors[axis-1];c.fillStyle=colors[axis-1];
            // Referencja pod pomiarem: przerywana linia w kolorze osi, przerwy jak w pomiarze.
            if(reference) {c.lineWidth=1.25;c.setLineDash([6,4]);line(c,planned,x,y,maximum);}
            // Cel F5: kropkowana jasna linia — to, co napęd faktycznie dostał.
            if(command) {c.save();c.strokeStyle="#e8eef4";c.lineWidth=1.5;c.setLineDash([1.5,3]);line(c,sent,x,y,maximum);c.restore();}
            c.setLineDash([]);c.lineWidth=1.5;line(c,visible,x,y,maximum);
            for(const p of visible) if(p.gap) {c.beginPath();c.arc(x(p.time_ms),y(p.position_cdeg/100),2,0,2*Math.PI);c.fill();}
            if(cursor!==null && cursor>=start && cursor<=end) {c.setLineDash([3,3]);c.strokeStyle="#53636c";c.beginPath();c.moveTo(x(cursor),top);c.lineTo(x(cursor),top+h);c.stroke();}
            c.restore();
            if(!visible.length) {c.fillStyle="#9aaaba";c.textAlign="center";c.fillText(loading?"Wczytywanie historii…":"Brak próbek w tym zakresie",left+w/2,top+h/2);}
        }
        draw();const resize=new ResizeObserver(draw);resize.observe(element);return ()=>resize.disconnect();
    },[axis,points,reference,command,rpm,right,start,end,scale,cursor,loading]);
    return <canvas ref={canvas} className={dragging?"is-dragging":""} aria-label={`Przebieg pozycji wału J${axis}`} aria-describedby={detached?"trace-floating-pan-help":"trace-pan-help"} role="img"
        onPointerDown={event=>{
            if(event.button!==0||!event.isPrimary) return;
            event.preventDefault();
            drag.current={pointer:event.pointerId,x:event.clientX,end,span:end-start,width:Math.max(1,event.currentTarget.clientWidth-margin),moved:false};
            event.currentTarget.setPointerCapture(event.pointerId);
        }}
        onPointerUp={event=>{
            if(drag.current?.pointer!==event.pointerId) return;
            releaseDrag();
            if(event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
        }} onPointerCancel={releaseDrag} onLostPointerCapture={releaseDrag}
        onPointerLeave={()=>onCursor(null)} onPointerMove={event=>{
        const anchor=drag.current;
        if(anchor?.pointer===event.pointerId) {
            if(!(event.buttons&1)) {releaseDrag();return;}
            const delta=event.clientX-anchor.x;
            if(!anchor.moved&&Math.abs(delta)<3) return;
            anchor.moved=true;setDragging(true);onPan(anchor.end-delta/anchor.width*anchor.span);return;
        }
        const box=event.currentTarget.getBoundingClientRect(),ratio=Math.min(1,Math.max(0,(event.clientX-box.left-64)/Math.max(1,box.width-margin)));
        onCursor(start+ratio*(end-start));
    }}/>;
}
export function DiagnosticsPage({visible,pinnedAxis,onPin,onDock,onClose}:{visible:boolean;pinnedAxis:number|null;onPin:(axis:number)=>void;onDock:()=>void;onClose:()=>void}) {
    const [initial]=useState(loadSettings),[seconds,setSeconds]=useState(initial.seconds),[scales,setScales]=useState<YScale[]>(initial.scales);
    const [status,setStatus]=useState<ShaftStatus|null>(null),[connected,setConnected]=useState(false),[error,setError]=useState("");
    const [receivedAt,setReceivedAt]=useState(performance.now()),[clock,setClock]=useState(performance.now());
    const [live,setLive]=useState(true),[end,setEnd]=useState(Date.now()),[cursor,setCursor]=useState<number|null>(null);
    const [overlays,setOverlays]=useState(loadOverlays);
    // Grupy po 6 (traceGroups): pomiar, referencja, cel F5, RPM F5.
    const livePoints=useRef<ShaftPoint[][]>(Array.from({length:traceChannelCount},()=>[]));
    const history=useRef<ShaftHistoryCache|null>(null),statusRef=useRef<ShaftStatus|null>(null);
    const [historyVersion,setHistoryVersion]=useState(0),[liveVersion,setLiveVersion]=useState(0),[cursorPoints,setCursorPoints]=useState<ShaftPoint[][]|null>(null);
    const range=useRef({seconds,live,end});range.current={seconds,live,end};
    const [reload,setReload]=useState(0);
    const start=end-seconds*1000;
    const clockReady=status!==null;
    const channels=useMemo(()=>history.current?.select(start,end,livePoints.current)??Array.from({length:traceChannelCount},()=>[]),[historyVersion,start,end,live?liveVersion:0]);
    const series=channels.slice(0,6),references=channels.slice(6,12),commands=channels.slice(12,18),speeds=channels.slice(18,24);
    const loading=history.current?.loading??true;
    function acceptStatus(incoming:ShaftStatus) {
        if(statusRef.current&&incoming.server_time_ms<statusRef.current.server_time_ms) return;
        statusRef.current=incoming;setStatus(incoming);setReceivedAt(performance.now());
    }
    const statusAge=Math.max(0,clock-receivedAt),sourceConnected=connected&&status?.connected&&statusAge<500;
    useEffect(()=>{const timer=setInterval(()=>setClock(performance.now()),100);return ()=>clearInterval(timer);},[]);
    useEffect(()=>{try {localStorage.setItem("robot-shaft-trace-view-v1",JSON.stringify({seconds,scales}));} catch {/* Odczyt działa bez localStorage. */}},[seconds,scales]);
    useEffect(()=>{try {localStorage.setItem("robot-shaft-trace-overlays-v1",JSON.stringify(overlays));} catch {/* Jak wyżej. */}},[overlays]);
    useEffect(()=>{
        const cache=new ShaftHistoryCache(async(start,end,signal)=>{
            const response=await fetch(`${traceBase}/history?start_ms=${start}&end_ms=${end}&max_points=2000`,{signal});
            const data=await response.json() as ShaftHistory & {detail?:string};
            if(!response.ok) throw new Error(data.detail??"Nie udało się odczytać historii");
            return data;
        },()=>setHistoryVersion(v=>v+1),data=>{acceptStatus(data.status);setError("");},error=>setError(error instanceof Error?error.message:"Błąd historii"),traceChannelCount);
        history.current=cache;return ()=>{cache.dispose();history.current=null;};
    },[]);
    useEffect(()=>{
        if(clockReady) history.current?.setView(start,end,live,statusRef.current!.server_time_ms,reload);
    },[start,end,live,reload,clockReady]);
    useEffect(()=>{
        // Gdy WS nie odpowiada, HTTP nadal pozwala otworzyć zachowaną historię.
        const controller=new AbortController();
        const timer=setTimeout(async()=>{
            if(statusRef.current) return;
            try {
                const response=await fetch(traceBase,{signal:controller.signal});
                if(!response.ok) throw new Error("Nie udało się odczytać statusu trace");
                const data=await response.json() as ShaftStatus;
                if(!controller.signal.aborted&&!statusRef.current) {acceptStatus(data);setEnd(data.server_time_ms);}
            } catch(error) {if(!controller.signal.aborted&&!statusRef.current) setError(error instanceof Error?error.message:"Błąd statusu trace");}
        },200);
        return ()=>{clearTimeout(timer);controller.abort();};
    },[]);
    useEffect(()=>{
        let cancelled=false,socket:WebSocket|null=null,retry:ReturnType<typeof setTimeout>|undefined;
        const connect=()=>{
            socket=new WebSocket(`${location.protocol==="https:"?"wss":"ws"}://${location.host}${traceBase}/ws`);
            socket.onopen=()=>{if(!cancelled) {setConnected(true);setError("");setReload(v=>v+1);}};
            socket.onmessage=event=>{
                if(cancelled) return;
                let frame:ShaftFrame;
                try {frame=JSON.parse(event.data) as ShaftFrame;} catch {return;}
                if(!frame.status||!Array.isArray(frame.samples)) return;
                acceptStatus(frame.status);
                if(frame.samples.length) {
                    livePoints.current=livePoints.current.map((items,i)=>mergeTrace(items,frame.samples.filter(p=>p.axis===i%6+1&&traceGroup(p)===Math.floor(i/6)),-Infinity));
                    setLiveVersion(v=>v+1);
                }
                if(range.current.live) {
                    const right=frame.status.server_time_ms;setEnd(right);
                }
                if(frame.expired) {setError("Przerwa w odbiorze próbek — odświeżono historię");setReload(v=>v+1);}
            };
            socket.onclose=()=>{if(!cancelled) {setConnected(false);retry=setTimeout(connect,1000);}};
            socket.onerror=()=>socket?.close();
        };
        connect();return ()=>{cancelled=true;clearTimeout(retry);socket?.close();};
    },[]);
    useEffect(()=>{
        setCursorPoints(null);
        if(cursor===null || seconds<=60) return;
        const abort=new AbortController();const timer=setTimeout(async()=>{
            try {const response=await fetch(`${traceBase}/history?start_ms=${cursor-80}&end_ms=${cursor}&max_points=16`,{signal:abort.signal});
                if(response.ok) {const data=await response.json() as ShaftHistory;if(!abort.signal.aborted) setCursorPoints(Array.from({length:traceChannelCount},(_,i)=>historyChannel(data,i)));}
            } catch {/* Następne położenie kursora ponowi odczyt. */}
        },120);return ()=>{clearTimeout(timer);abort.abort();};
    },[cursor,seconds]);
    function scale(axis:number,patch:Partial<YScale>) {setScales(old=>old.map((v,i)=>i===axis?{...v,...patch}:v));}
    function panTo(value:number) {
        range.current={...range.current,live:false,end:value};
        setLive(false);setCursor(null);setEnd(value);
    }
    function pan(delta:number) {panTo(range.current.end+delta*seconds*1000);}
    function goLive() {
        const right=sourceConnected?status!.server_time_ms:Date.now();
        range.current={seconds:10,live:true,end:right};
        setLive(true);setSeconds(10);setEnd(right);setCursor(null);setReload(v=>v+1);
    }
    function allHistory() {
        const first=status?.axes.flatMap(a=>a.first_ms===null?[]:[a.first_ms])??[],last=status?.axes.flatMap(a=>a.last_ms===null?[]:[a.last_ms])??[];
        if(first.length&&last.length) {setLive(false);setSeconds(Math.max(.1,(Math.max(...last)-Math.min(...first))/1000));setEnd(Math.max(...last));setCursor(null);}
    }
    const controls=<>
        <div className="trace-controls card">
            <label>Okno czasu [s]<input aria-label="Okno czasu [s]" type="number" min="0.1" step="0.1" value={seconds} onChange={e=>{const n=Number(e.target.value);if(Number.isFinite(n)&&n>=.1) setSeconds(n);}}/></label>
            <div className="trace-toolbar" role="group" aria-label="Sterowanie wykresami">
                <TraceButton action="zoom-in" label="Powiększ czas" title="Powiększ czas — krótsze okno" onClick={()=>setSeconds(v=>Math.max(.1,v/2))}/>
                <TraceButton action="zoom-out" label="Pomniejsz czas" title="Pomniejsz czas — dłuższe okno" onClick={()=>setSeconds(v=>v*2)}/>
                <TraceButton action="earlier" label="Wcześniej" title="Wcześniej — przesuń o pół okna" onClick={()=>pan(-.5)}/>
                <TraceButton action="later" label="Później" title="Później — przesuń o pół okna" onClick={()=>pan(.5)}/>
                <TraceButton action="pause" label="Wstrzymaj podgląd" disabled={!live} onClick={()=>{range.current.live=false;setLive(false);}}/>
                <TraceButton action="live" label="Na żywo" title="Na żywo — wróć do ostatnich 10 sekund" active={live} onClick={goLive}/>
                <TraceButton action="fit" label="Cała historia" title="Dopasuj całą zachowaną historię" onClick={allHistory}/>
            </div>
            {!live&&<label>Koniec zakresu<input aria-label="Koniec zakresu" type="datetime-local" step="0.001" value={localTime(end)} onChange={e=>{const t=new Date(e.target.value).getTime();if(Number.isFinite(t)) setEnd(t);}}/></label>}
            <div className="trace-channels" role="group" aria-label="Kanały dodatkowe">
                {overlayLabels.map(([key,label,title])=><label key={key} title={title}><input type="checkbox" checked={overlays[key]} onChange={e=>setOverlays(v=>({...v,[key]:e.target.checked}))}/>{label}</label>)}
            </div>
            <span className="trace-cursor">{cursor===null?"Kursor: najedź na wykres":`Kursor: ${new Date(cursor).toLocaleTimeString("pl-PL",{fractionalSecondDigits:3})}`}</span>
        </div>
    </>;
    function chart(axis:number) {
            const i=axis-1,points=series[i],info=status?.axes.find(a=>a.axis===axis),settings=scales[i];
            const fresh=sourceConnected&&info?.fresh&&info.age_seconds!==null&&info.age_seconds+statusAge/1000<.25;
            const reading=cursor!==null?valueAt(cursorPoints?.[i]??points,cursor):live?(fresh?info?.latest:null):valueAt(points,end);
            // Odchyłka pomiaru od referencji w chwili odczytu; brak, gdy referencja nie istnieje w pobliżu (np. JOG prędkością).
            const planned=reading?valueAt(cursorPoints?.[i+6]??references[i],reading.time_ms):null;
            const deviation=overlays.reference&&reading&&planned?(reading.position_cdeg-planned.position_cdeg)/100:null;
            const sent=overlays.command&&reading?valueAt(cursorPoints?.[i+12]??commands[i],reading.time_ms):null;
            const speed=overlays.rpm&&reading?valueAt(cursorPoints?.[i+18]??speeds[i],reading.time_ms):null;
            return <article key={axis} className="shaft-chart card" data-axis={axis}>
                <header><h2><span style={{background:colors[i]}}/>J{axis}<small>CAN {axis} · wał</small></h2><strong className="shaft-value">{positionText(reading)}</strong>{deviation!==null&&<span className="shaft-deviation" title={`Referencja ${positionText(planned)}`}>Δ ref {deviation>=0?"+":""}{deviation.toFixed(2)}°</span>}{sent&&reading&&<span className="shaft-deviation" title={`Cel F5 ${positionText(sent)}`}>Δ F5 {reading.position_cdeg>=sent.position_cdeg?"+":""}{((reading.position_cdeg-sent.position_cdeg)/100).toFixed(2)}°</span>}{speed&&<span className="shaft-deviation">{(speed.position_cdeg/100).toFixed(speed.position_cdeg<1000?2:0)} RPM</span>}<span className={`badge ${info?.error?"error":fresh?"ok":"stale"}`}>{info?.error?"Błąd zapisu":fresh?"Świeże dane":"Brak świeżego pomiaru"}</span>{axis!==pinnedAxis&&<TraceButton action="pin" label={`Przypnij wykres J${axis}`} title={`Przypnij J${axis} — podgląd w innych modułach`} onClick={()=>onPin(axis)}/>}</header>
                <ShaftChart axis={axis} points={points} reference={overlays.reference?references[i]:null} command={overlays.command?commands[i]:null} rpm={overlays.rpm?speeds[i]:null} start={start} end={end} scale={settings} cursor={cursor} loading={loading} detached={axis===pinnedAxis} onCursor={setCursor} onPan={panTo}/>
                <div className="shaft-scale"><label><input type="checkbox" checked={settings.auto} onChange={e=>scale(i,{auto:e.target.checked})}/>Auto Y</label>
                    <label>Min [°]<input aria-label={`J${axis} minimum Y`} type="number" step="0.01" disabled={settings.auto} value={settings.minimum} onChange={e=>{const n=Number(e.target.value);if(Number.isFinite(n)&&n<settings.maximum) scale(i,{minimum:n});}}/></label>
                    <label>Max [°]<input aria-label={`J${axis} maksimum Y`} type="number" step="0.01" disabled={settings.auto} value={settings.maximum} onChange={e=>{const n=Number(e.target.value);if(Number.isFinite(n)&&n>settings.minimum) scale(i,{maximum:n});}}/></label>
                </div>
                <footer><span>{((info?.file_bytes??0)/1e6).toFixed(2)} / 20 MB · {info?.retained_records??0} próbek</span><div><a href={`${traceBase}/axes/${axis}/export.csv?start_ms=${start}&end_ms=${end}`}>CSV zakresu</a><a href={`${traceBase}/axes/${axis}/export.csv`}>CSV całości</a></div></footer>
                <p className="trace-retained">Historia: {timeText(info?.first_ms)} — {timeText(info?.last_ms)}</p>
                {info?.error&&<p className="trace-error">{info.error}</p>}
                {!!info?.corrupt_records&&<p className="trace-error">Pominięte uszkodzone wpisy: {info.corrupt_records}</p>}
            </article>;
    }
    const badges=<div className="trace-badges"><span className={`badge ${sourceConnected?"ok":"stale"}`}>{sourceConnected?"Źródło połączone":"Oczekiwanie na źródło"}</span><span className={`badge ${connected&&statusAge<500&&status?.recording?"ok":"warn"}`}>{!connected||statusAge>=500?"Stan zapisu nieaktualny":status?.recording?"Zapis ciągły aktywny":"Zapis niedostępny"}</span></div>;
    const losses=<p className="trace-note">Utraty: kolejka źródła {status?.source_dropped??0} · brakujące próbki {status?.missing_samples??0} · zapis {status?.writer_dropped??0}. Limit obejmuje cały plik osi; najstarsze dane są nadpisywane.</p>;
    return <>
        {visible&&<section className="diagnostics-screen" aria-label="Diagnostyka pozycji wałów">
            <div className="page-title"><div><div className="eyebrow">TRACE POZYCJI WAŁÓW</div><h1>Diagnostyka</h1><p>J1–J6 · ciągły zapis · dokładność prezentacji 0,01°</p></div>{badges}</div>
            {controls}
            {error&&<p className="trace-error" role="alert">{error}</p>}
            <p className="trace-note" id="trace-pan-help">Przeciągnij wykres lub użyj poziomej rolki myszy, aby przeglądać historię wszystkich osi. „Na żywo” wraca do ostatnich 10 s. Pinezka odłącza jedną oś; zakres czasu pozostaje wspólny. Linia ciągła: pomiar wału; przerywana w kolorze osi: referencja ruchu PTP/LIN/JOG TCP (cel planowany); kropkowana jasna: cel F5 wysłany do napędu; szara cienka (prawa oś): RPM F5. Δ ref / Δ F5: pomiar minus referencja / cel F5. Kanały włączasz checkboxami u góry.</p>
            <div className="shaft-charts">{series.map((_,i)=>i+1===pinnedAxis?null:chart(i+1))}</div>
            {losses}
        </section>}
        {pinnedAxis!==null&&createPortal(<TraceWindow axis={pinnedAxis} controls={controls} onDock={onDock} onClose={onClose}>
            <section className="diagnostics-screen" aria-label={`Pozycja wału J${pinnedAxis}`}>
                {badges}
                {error&&<p className="trace-error" role="alert">{error}</p>}
                <p className="trace-note" id="trace-floating-pan-help">Przeciągnij wykres lub użyj poziomej rolki myszy, aby przeglądać historię. Zakres czasu jest wspólny z modułem Diagnostyka. Zapis trwa stale.</p>
                {chart(pinnedAxis)}
                {losses}
            </section>
        </TraceWindow>,document.body)}
    </>;
}
