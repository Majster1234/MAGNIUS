import { useEffect, useState } from "react";
import type { Notice } from "./eventLog";
import type { TcpAxis, TcpFrame, TcpHeld } from "./planningTcpJog";
import { TCP_STEPS } from "./planningTcpStep";
import type { TcpJogStatus } from "./physicalTcpJog";

export function TcpControls({ allowed, planning, held, stepAvailable, start, stop, onNotice, onCondition, liveStatus, busy = false }: {
    allowed: boolean; planning: boolean; missing: string[]; held: TcpHeld | null;
    stepAvailable: boolean;
    start: (axis: TcpAxis, direction: number, frame: TcpFrame, speed: number, step?: number) => void; stop: (emergency?: boolean) => void;
    liveStatus?: TcpJogStatus; busy?: boolean;
    onNotice?:(value:Notice)=>void;onCondition?:(key:string,value:Notice|null)=>void;
}) {
    const [frame, setFrame] = useState<TcpFrame>("base");
    const [linear, setLinear] = useState<number | "">(planning ? 10 : ""), [angular, setAngular] = useState<number | "">(planning ? 5 : "");
    const [mode, setMode] = useState("continuous"), [stepIndex, setStepIndex] = useState(1);
    const impulse = mode === "step", step = TCP_STEPS[stepIndex];
    useEffect(()=>{
        onNotice?.({source:"tcp-mode",severity:"info",context:planning?"Cel TCP":"JOG TCP LIVE",message:!planning?"Przytrzymaj kierunek TCP. Puszczenie hamuje, także w trybie impulsowym.":impulse?held?.impulse?"Impuls TCP w toku":"Jedno naciśnięcie = jeden impuls TCP":"Przytrzymaj kierunek TCP. Puszczenie zatrzymuje cel symulacji."});
        onCondition?.("tcp-availability",planning && (!allowed && !held || impulse && !stepAvailable)?{source:"tcp-availability",severity:"warning",context:"Cel TCP",title:"TCP czeka na gotowość planowania",message:impulse && !stepAvailable?"Planer ruchu impulsowego niedostępny.":"TCP wymaga aktualnego modelu i sześciu pozycji celu.",description:"Edycja TCP jest chwilowo niedostępna w bieżącym stanie panelu.",remedy:"Sprawdź model i komplet pozycji celu, zaczekaj na zakończenie JOG lub obliczeń. Symulowane odniesienia wymagają świadomie włączonego trybu planowania bez napędów."}:null);
    },[allowed,planning,impulse,stepAvailable,held?.axis,held?.impulse,onNotice,onCondition]);
    useEffect(()=>()=>onCondition?.("tcp-availability",null),[onCondition]);
    function activate(axis: TcpAxis, direction: number) {
        const speed = ["x", "y", "z"].includes(axis) ? linear : angular;
        if (speed === "" || !Number.isFinite(speed) || speed <= 0) {
            onNotice?.({source:"tcp-live",severity:"warning",context:"JOG TCP LIVE",message:"Podaj dodatnią prędkość TCP przed rozpoczęciem ruchu."}); return;
        }
        start(axis, direction, frame, speed, impulse ? step : undefined);
    }
    function release() { if (!impulse || !planning) stop(); }
    const locked = !!held || busy;
    function button(axis: TcpAxis, direction: number, label: string, position: string) {
        const active = held?.axis === axis && held.direction === direction;
        return <button type="button" className={`tcp-direction tcp-${position}${active ? " tcp-held" : ""}`}
            aria-label={`TCP ${axis.toUpperCase()} ${direction > 0 ? "plus" : "minus"}`} aria-pressed={active}
            disabled={!allowed && !active || busy && !active || impulse && !stepAvailable || !!held && (!active || planning && !!held.impulse)}
            onPointerDown={event => {
                if (event.button !== 0) return;
                event.preventDefault(); event.currentTarget.setPointerCapture(event.pointerId);
                activate(axis, direction);
            }} onPointerUp={release} onPointerCancel={() => stop(true)} onLostPointerCapture={release}
            onClick={event => { if (planning && impulse && event.detail === 0) activate(axis, direction); }}
            onKeyDown={event => {
                if (event.key !== "Enter" && event.key !== " ") return;
                if (!planning) { event.preventDefault(); if (!event.repeat) activate(axis, direction); }
                else if (impulse && event.repeat) event.preventDefault();
            }} onKeyUp={event => { if (!planning && (event.key === "Enter" || event.key === " ")) {event.preventDefault();release();} }}
            onContextMenu={event => event.preventDefault()}>{label}</button>;
    }
    return <section className={`tcp-controls${impulse ? " tcp-impulse" : ""}`} aria-label="Sterowanie TCP">
        <header><div><strong>{planning ? "TCP" : "TCP · LIVE"}</strong><select aria-label="Tryb TCP" value={mode} disabled={locked} onChange={e => setMode(e.target.value)}>
            <option value="continuous">Ciągły</option><option value="step">Impulsowy</option></select></div>
            <label className="tcp-frame"><select aria-label="Układ TCP" value={frame} disabled={locked} onChange={e => setFrame(e.target.value as TcpFrame)}>
                <option value="base">Baza</option><option value="tool">Narzędzie</option></select></label></header>
        <div className="tcp-joysticks">{([false, true] as const).map(rotation => <div key={String(rotation)} className="tcp-joystick-group">
            <span>{rotation ? "Obrót" : "Przesuw"}</span>
            <div className="tcp-pad"><div className="tcp-cross">
                {button(rotation ? "ry" : "y", 1, rotation ? "Ry+" : "Y+", "up")}
                {button(rotation ? "rx" : "x", -1, rotation ? "Rx−" : "X−", "left")}
                <span className="tcp-center" aria-hidden="true">●</span>
                {button(rotation ? "rx" : "x", 1, rotation ? "Rx+" : "X+", "right")}
                {button(rotation ? "ry" : "y", -1, rotation ? "Ry−" : "Y−", "down")}
            </div>
            <div className="tcp-depth">{button(rotation ? "rz" : "z", 1, rotation ? "Rz+" : "Z+", "depth-plus")}
                {button(rotation ? "rz" : "z", -1, rotation ? "Rz−" : "Z−", "depth-minus")}</div></div>
            {(!impulse || !planning) && <label className="tcp-speed"><input type="number" aria-label={rotation ? "Prędkość obrotu TCP" : "Prędkość przesuwu TCP"}
                min="0.01" max={planning ? rotation ? 30 : 100 : rotation ? liveStatus?.max_angular_velocity ? liveStatus.max_angular_velocity * 180 / Math.PI : undefined : liveStatus?.max_linear_velocity ? liveStatus.max_linear_velocity * 1000 : undefined}
                step="any" value={rotation ? angular : linear} disabled={locked}
                onChange={e => (rotation ? setAngular : setLinear)(e.target.value === "" ? "" : Number(e.target.value))} />{rotation ? "°/s" : "mm/s"}</label>}
        </div>)}</div>
        {impulse && <div className="tcp-step-selector"><label htmlFor="tcp-step-size">Impuls: <strong>{step} mm / {step}°</strong></label>
            <input id="tcp-step-size" aria-label="Wielkość impulsu TCP" type="range" min="0" max="4" step="1" value={stepIndex}
                aria-valuetext={`${step} mm / ${step}°`} disabled={locked} onChange={e => setStepIndex(Number(e.target.value))} />
            <div className="tcp-step-ticks" aria-hidden="true">{TCP_STEPS.map(value => <span key={value}>{value}</span>)}</div></div>}
        {impulse && <div className="tcp-message"><button type="button" className="tcp-step-stop" disabled={!held?.impulse} onClick={() => stop()}>Zatrzymaj impuls</button></div>}
        {!planning && <p className="tcp-message" role="status">{liveStatus?.fresh ? liveStatus.message : "Oczekiwanie na aktualny status JOG TCP"}</p>}
    </section>;
}
