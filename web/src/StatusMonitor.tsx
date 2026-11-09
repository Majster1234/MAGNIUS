import { useEffect, useId, useRef, useState } from "react";
import { nextColumn, pulseLabels, PULSE_COLUMNS, worst, type DriveCell, type PulseSection, type SectionId } from "./systemPulse";

const ids: SectionId[] = ["SYS", "ROS", "API", "WEB"];
const empty = () => Object.fromEntries(ids.map(id => [id, Array<boolean>(PULSE_COLUMNS).fill(false)])) as Record<SectionId, boolean[]>;
const reducedMotion = () => typeof matchMedia === "function" && matchMedia("(prefers-reduced-motion: reduce)").matches;

// Dymek otwierany najechaniem, fokusem klawiatury (CSS) albo dotknięciem (stan `open`).
function Tip({ className, label, head, lines, children }: {
    className: string; label: string; head: string; lines: string[]; children: React.ReactNode;
}) {
    const [open, setOpen] = useState(false);
    return <span className={`pulse-status ${className}${open ? " open" : ""}`} tabIndex={0} role="button" aria-label={`${label}. ${lines.join(". ")}`}
        aria-expanded={open} onClick={() => setOpen(value => !value)} onBlur={() => setOpen(false)}
        onKeyDown={event => { if (event.key === "Escape") setOpen(false); }}>
        {children}
        <span className="pulse-tip" role="tooltip"><span className="pulse-tip-head">{head}</span>{lines.map((line, index) => <span key={index}>{line}</span>)}</span>
    </span>;
}

/** Monitor pracy: kanały przesuwają się o piksel przy każdym odebraniu danych danej sekcji. */
export function StatusMonitor({ sections, drives, ticks }: {
    sections: PulseSection[]; drives: DriveCell[]; ticks: Record<SectionId, unknown>;
}) {
    const pattern = useId().replace(/:/g, "");
    const [channels, setChannels] = useState(empty);
    const previous = useRef<Record<SectionId, unknown>>({ SYS: undefined, ROS: undefined, API: undefined, WEB: undefined });
    const counters = useRef<Record<SectionId, number>>({ SYS: 0, ROS: 0, API: 0, WEB: 0 });
    useEffect(() => {
        const changed = ids.filter(id => ticks[id] !== undefined && ticks[id] !== previous.current[id]);
        if (!changed.length) return;
        for (const id of changed) previous.current[id] = ticks[id];
        if (reducedMotion()) return;
        setChannels(current => {
            const next = { ...current };
            changed.forEach((id, index) => { next[id] = nextColumn(current[id], ++counters.current[id] * 4 + ids.indexOf(id) + index); });
            return next;
        });
    }, [ticks.SYS, ticks.ROS, ticks.API, ticks.WEB]);
    const state = Object.fromEntries(sections.map(item => [item.id, item.state])) as Record<SectionId, PulseSection["state"]>;
    const online = drives.filter(cell => cell.state === "ok" || cell.state === "warn").length;
    return <div className="pulse-monitor" role="group" aria-label="Monitor pracy systemu">
        <svg className="pulse-channels" viewBox={`0 0 ${PULSE_COLUMNS * 3} 17`} aria-hidden="true">
            <defs><pattern id={pattern} width="3" height="5" patternUnits="userSpaceOnUse"><rect width="2" height="2" /></pattern></defs>
            {ids.map((id, row) => <g key={id} className={`pulse-channel ${state[id] ?? "stale"}`}>
                <rect className="pulse-grid" y={row * 5} width={PULSE_COLUMNS * 3} height="2" fill={`url(#${pattern})`} />
                {channels[id].map((on, column) => on ? <rect key={column} x={column * 3} y={row * 5} width="2" height="2" /> : null)}
            </g>)}
        </svg>
        <span className="pulse-sep" />
        <div className="pulse-statuses">{sections.map(item => <Tip key={item.id} className={item.state} label={`${item.title}: ${pulseLabels[item.state]}`}
            head={`${item.id} · ${pulseLabels[item.state]}`} lines={[item.title, ...item.lines]}>
            <i /><b>{item.id}</b>
        </Tip>)}</div>
        <span className="pulse-sep" />
        <Tip className={`drives ${worst(drives.map(cell => cell.state))}`} label={`Napędy J1–J6: ${online}/6 online`}
            head={`Napędy · ${online}/6 online`} lines={drives.map(cell => cell.text)}>
            <span className="pulse-drives">{drives.map(cell => <i key={cell.axis} className={cell.state} />)}</span>
            <span className="pulse-drives-count">{online}/6</span>
        </Tip>
    </div>;
}
