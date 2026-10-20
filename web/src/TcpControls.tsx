import { useState } from "react";
import type { TcpAxis, TcpFrame, TcpHeld } from "./planningTcpJog";
import { TCP_STEPS } from "./planningTcpStep";

export function TcpControls({ allowed, planning, missing, held, stepAvailable, start, stop }: {
    allowed: boolean; planning: boolean; missing: string[]; held: TcpHeld | null;
    stepAvailable: boolean;
    start: (axis: TcpAxis, direction: number, frame: TcpFrame, speed: number, step?: number) => void; stop: () => void;
}) {
    const [frame, setFrame] = useState<TcpFrame>("base");
    const [linear, setLinear] = useState(10), [angular, setAngular] = useState(5);
    const [mode, setMode] = useState("continuous"), [stepIndex, setStepIndex] = useState(1);
    const impulse = mode === "step", step = TCP_STEPS[stepIndex];
    function activate(axis: TcpAxis, direction: number) { start(axis, direction, frame, ["x", "y", "z"].includes(axis) ? linear : angular, impulse ? step : undefined); }
    function release() { if (!impulse) stop(); }
    function button(axis: TcpAxis, direction: number, label: string, position: string) {
        const active = held?.axis === axis && held.direction === direction;
        return <button type="button" className={`tcp-direction tcp-${position}${active ? " tcp-held" : ""}`}
            aria-label={`TCP ${axis.toUpperCase()} ${direction > 0 ? "plus" : "minus"}`} aria-pressed={active}
            disabled={!allowed || impulse && !stepAvailable || !!held && (!active || !!held.impulse)}
            onPointerDown={event => {
                if (event.button !== 0) return;
                event.preventDefault(); event.currentTarget.setPointerCapture(event.pointerId);
                activate(axis, direction);
            }} onPointerUp={release} onPointerCancel={stop} onLostPointerCapture={release}
            onClick={event => { if (impulse && event.detail === 0) activate(axis, direction); }}
            onKeyDown={event => { if (impulse && event.repeat && (event.key === "Enter" || event.key === " ")) event.preventDefault(); }}
            onContextMenu={event => event.preventDefault()}>{label}</button>;
    }
    return <section className={`tcp-controls${impulse ? " tcp-impulse" : ""}`} aria-label="Sterowanie TCP">
        <header><div><strong>TCP</strong><select aria-label="Tryb TCP" value={mode} disabled={!!held} onChange={e => setMode(e.target.value)}>
            <option value="continuous">Ciągły</option><option value="step">Impulsowy</option></select></div>
            <label className="tcp-frame"><select aria-label="Układ TCP" value={frame} disabled={!!held} onChange={e => setFrame(e.target.value as TcpFrame)}>
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
            {!impulse && <label className="tcp-speed"><input type="number" aria-label={rotation ? "Prędkość obrotu TCP" : "Prędkość przesuwu TCP"}
                min="0.01" max={rotation ? 30 : 100} step="any" value={rotation ? angular : linear} disabled={!!held}
                onChange={e => (rotation ? setAngular : setLinear)(Math.max(.01, Math.min(rotation ? 30 : 100, Number(e.target.value) || .01)))} />{rotation ? "°/s" : "mm/s"}</label>}
        </div>)}</div>
        {impulse && <div className="tcp-step-selector"><label htmlFor="tcp-step-size">Impuls: <strong>{step} mm / {step}°</strong></label>
            <input id="tcp-step-size" aria-label="Wielkość impulsu TCP" type="range" min="0" max="4" step="1" value={stepIndex}
                aria-valuetext={`${step} mm / ${step}°`} disabled={!!held} onChange={e => setStepIndex(Number(e.target.value))} />
            <div className="tcp-step-ticks" aria-hidden="true">{TCP_STEPS.map(value => <span key={value}>{value}</span>)}</div></div>}
        <p className="tcp-message" role="status">{!planning ? `${missing.length ? `Brak napędów: ${missing.join(", ")}. ` : "Napędy 6/6. "}Wykonanie fizyczne TCP niedostępne.` :
            impulse && !stepAvailable ? "Planer ruchu impulsowego niedostępny." :
            !allowed && !held ? "TCP wymaga aktualnego modelu i sześciu pozycji celu. Bez napędów włącz ignorowanie braków w planowaniu." :
            impulse ? <><span>{held?.impulse ? "Impuls w toku" : "Jedno naciśnięcie = jeden impuls"}</span><button type="button" className="tcp-step-stop" disabled={!held?.impulse} onClick={stop}>Zatrzymaj impuls</button></> : "Przytrzymaj kierunek. Puszczenie zatrzymuje cel."}</p>
    </section>;
}
