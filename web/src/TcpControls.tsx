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
    const [mode, setMode] = useState<"continuous" | "step">("continuous"), [stepIndex, setStepIndex] = useState(1);
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
    // Numpad pokazuje jedną grupę naraz; Shift chwilowo odwraca Przesuw/Obrót. W trakcie ruchu grupa wynika z trzymanej osi.
    const [rotationMode, setRotationMode] = useState(false), [shift, setShift] = useState(false);
    useEffect(() => {
        const key = (event: KeyboardEvent) => { if (event.key === "Shift") setShift(event.type === "keydown"); };
        const blur = () => setShift(false);
        window.addEventListener("keydown", key); window.addEventListener("keyup", key); window.addEventListener("blur", blur);
        return () => { window.removeEventListener("keydown", key); window.removeEventListener("keyup", key); window.removeEventListener("blur", blur); };
    }, []);
    const rotation = held ? held.axis.startsWith("r") : rotationMode !== shift;
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
    function segment<T extends string>(name: string, value: T, options: [T, string][], change: (value: T) => void) {
        return <span className="tcp-seg" role="radiogroup" aria-label={name}>{options.map(([option, label]) =>
            <button key={option} type="button" role="radio" aria-checked={value === option} disabled={locked}
                onClick={() => change(option)}>{label}</button>)}</span>;
    }
    const r = rotation ? "r" : "", sign = (axis: string) => rotation ? `R${axis}` : axis.toUpperCase();
    return <section className={`tcp-controls${impulse ? " tcp-impulse" : ""}`} aria-label="Sterowanie TCP">
        <div className="tcp-numpad">
            <div className="tcp-pad" role="group" aria-label={rotation ? "Obrót TCP" : "Przesuw TCP"}>
                {button(`${r}z` as TcpAxis, 1, `${sign("z")}+`, "z-plus")}
                {button(`${r}y` as TcpAxis, 1, `${sign("y")}+`, "up")}
                <span className="tcp-gap" aria-hidden="true" />
                {button(`${r}x` as TcpAxis, -1, `${sign("x")}−`, "left")}
                <span className="tcp-center" aria-hidden="true">●</span>
                {button(`${r}x` as TcpAxis, 1, `${sign("x")}+`, "right")}
                {button(`${r}z` as TcpAxis, -1, `${sign("z")}−`, "z-minus")}
                {button(`${r}y` as TcpAxis, -1, `${sign("y")}−`, "down")}
                <span className="tcp-gap" aria-hidden="true" />
            </div>
            <div className="tcp-options">
                {segment("Grupa TCP", rotation ? "rotation" : "translation", [["translation", "Przesuw"], ["rotation", "Obrót"]], value => setRotationMode((value === "rotation") !== shift))}
                {segment("Układ TCP", frame, [["base", "Baza"], ["tool", "Narzędzie"]], setFrame)}
                {segment("Tryb TCP", mode, [["continuous", "Ciągły"], ["step", "Impuls"]], setMode)}
                {/* Stałe miejsce na prędkość: w impulsie planowania pole jest ukryte, ale nie zwalnia miejsca. */}
                <label className={`tcp-speed${impulse && planning ? " is-hidden" : ""}`} aria-hidden={impulse && planning || undefined}>Prędkość<input type="number" aria-label={rotation ? "Prędkość obrotu TCP" : "Prędkość przesuwu TCP"}
                    min="0.01" max={planning ? rotation ? 30 : 100 : rotation ? liveStatus?.max_angular_velocity ? liveStatus.max_angular_velocity * 180 / Math.PI : undefined : liveStatus?.max_linear_velocity ? liveStatus.max_linear_velocity * 1000 : undefined}
                    step="any" value={rotation ? angular : linear} disabled={locked || impulse && planning}
                    onChange={e => (rotation ? setAngular : setLinear)(e.target.value === "" ? "" : Number(e.target.value))} />{rotation ? "°/s" : "mm/s"}</label>
                <span className="tcp-hint"><kbd>Shift</kbd> przełącza na {rotation ? "przesuw" : "obrót"}</span>
            </div>
            {/* Blok impulsu jest zawsze obecny (nieaktywny w trybie ciągłym), więc zmiana trybu nie przesuwa karty. */}
            <div className={`tcp-step-selector${impulse ? "" : " inactive"}`}><label htmlFor="tcp-step-size">Impuls: <strong>{step} mm / {step}°</strong></label>
                <input id="tcp-step-size" aria-label="Wielkość impulsu TCP" type="range" min="0" max="4" step="1" value={stepIndex}
                    aria-valuetext={`${step} mm / ${step}°`} disabled={locked || !impulse} onChange={e => setStepIndex(Number(e.target.value))} />
                <div className="tcp-step-ticks" aria-hidden="true">{TCP_STEPS.map(value => <span key={value}>{value}</span>)}</div>
                <button type="button" className="tcp-step-stop" disabled={!held?.impulse} onClick={() => stop()}>Zatrzymaj impuls</button></div>
        </div>
        <p className="tcp-message" role="status">{!planning ? liveStatus?.fresh ? liveStatus.message : "Oczekiwanie na aktualny status JOG TCP" : "Cel TCP porusza wyłącznie bliźniakiem 3D."}</p>
    </section>;
}
