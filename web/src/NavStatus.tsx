import { useState, type ReactNode } from "react";
import { barPercent, gigabytes, loadColor, loadGradient, percentText, type NavResources } from "./navStatus";
import { pulseLabels, worst, type DriveCell, type PulseSection } from "./systemPulse";

// Dymek otwierany najechaniem, fokusem klawiatury (CSS) albo dotknięciem; wyświetlany po prawej stronie menu.
function Tip({ className, label, head, lines, children }: {
    className: string; label: string; head: string; lines: string[]; children: ReactNode;
}) {
    const [open, setOpen] = useState(false);
    return <span className={`ns-tipwrap ${className}${open ? " open" : ""}`} tabIndex={0} role="button" aria-label={`${label}. ${lines.join(". ")}`}
        aria-expanded={open} onClick={() => setOpen(value => !value)} onBlur={() => setOpen(false)}
        onKeyDown={event => { if (event.key === "Escape") setOpen(false); }}>
        {children}
        <span className="ns-tip" role="tooltip"><b>{head}</b>{lines.map((line, index) => <span key={index}>{line}</span>)}</span>
    </span>;
}

const threads = (n: number) => n === 1 ? "1 wątek" : n % 10 >= 2 && n % 10 <= 4 && (n % 100 < 12 || n % 100 > 14) ? `${n} wątki` : `${n} wątków`;

/** Pasek obciążenia: tło z gradientem skali, zasłona zakrywa niewykorzystaną część (kolor wynika z położenia). */
function LoadBar({ value, full = 100, vertical = false }: { value: number | null; full?: number; vertical?: boolean }) {
    const used = value === null ? 0 : barPercent(value / full * 100) * full / 100;
    return <span className={vertical ? "ns-core-bar" : "ns-meter"} style={{ backgroundImage: loadGradient(vertical ? "0deg" : "90deg", full) }}>
        <i style={vertical ? { height: `${100 - used}%` } : { width: `${100 - used}%` }} /></span>;
}

/** Stan systemu w menu. Jeden układ dla obu stanów menu: przy rozwijaniu cienkie paski CPU/RAM/CAN płynnie się
 *  wydłużają i pogrubiają, a opisy, procenty i słupki wątków wysuwają się (animacja w theme.css). */
export function NavStatus({ sections, drives, resources, collapsed }: {
    sections: PulseSection[]; drives: DriveCell[]; resources: NavResources; collapsed: boolean;
}) {
    const online = drives.filter(cell => cell.state === "ok" || cell.state === "warn").length;
    const drivesState = worst(drives.map(cell => cell.state));
    const overall = worst([...sections.map(item => item.state), drivesState]);
    const status = (item: PulseSection, className: string) => <Tip key={item.id} className={`${className} ${item.state}`}
        label={`${item.title}: ${pulseLabels[item.state]}`} head={`${item.id} · ${pulseLabels[item.state]}`} lines={[item.title, ...item.lines]}>
        <i /><b>{item.id}</b>
    </Tip>;
    const driveTip = (className: string, children: ReactNode) => <Tip className={`${className} ${drivesState}`} label={`Napędy J1–J6: ${online}/6 online`}
        head={`Napędy · ${online}/6 online`} lines={drives.map(cell => cell.text)}>{children}</Tip>;
    const driveBars = <span className="ns-drives">{drives.map(cell => <i key={cell.axis} className={cell.state} />)}</span>;
    const { cpu, cores, ram, ramUsed, ramTotal, canLoad, canLimit } = resources;
    const canScale = canLimit ?? 100;
    return <div className={`nav-status ${collapsed ? "compact" : "full"}`} role="group" aria-label="Stan systemu">
        <div className="ns-fold ns-head"><span className="ns-label">Stan systemu</span><span className={`badge ${overall}`}><span className="dot" />{pulseLabels[overall]}</span></div>
        {/* Statusy i napędy: lista pionowa (zwinięte) ↔ pola w rzędzie (rozwinięte), przenikanie */}
        <div className="ns-fold ns-only-compact ns-list">{sections.map(item => status(item, "ns-item"))}
            {driveTip("ns-drive-tip", <>{driveBars}<b>{online}/6</b></>)}</div>
        <div className="ns-fold ns-only-full ns-chips">{sections.map(item => status(item, "ns-chip"))}</div>
        <div className="ns-fold ns-only-full">{driveTip("ns-drv", <><span className="ns-label">Napędy</span>{driveBars}<b>{online}/6</b></>)}</div>
        <div className="ns-gauges">
            <div className="ns-gauge" title={`CPU ${percentText(cpu)} %`}>
                <div className="ns-fold ns-row"><span className="ns-label">CPU</span><b style={{ color: loadColor(cpu === null ? null : cpu / 100) }}>{percentText(cpu)}</b>
                    <em>%{cores.length ? ` · ${threads(cores.length)}` : ""}</em></div>
                <LoadBar value={cpu} />
                {cores.length > 0 && <div className="ns-fold ns-cores">{cores.map((value, index) => <div className="ns-core" key={index} title={`Wątek ${index}: ${percentText(value)} %`}>
                    <LoadBar value={value} vertical /><small style={{ color: loadColor(value / 100) }}>{percentText(value)}</small><small className="ns-core-n">W{index}</small>
                </div>)}</div>}
            </div>
            <div className="ns-gauge" title={`RAM ${percentText(ram)} %`}>
                <div className="ns-fold ns-row"><span className="ns-label">RAM</span><b style={{ color: loadColor(ram === null ? null : ram / 100) }}>{percentText(ram)}</b>
                    <em>% · {gigabytes(ramUsed)} / {gigabytes(ramTotal)} G</em></div>
                <LoadBar value={ram} />
            </div>
            <div className="ns-gauge" title={`Obciążenie CAN ${percentText(canLoad, 1)} %${canLimit !== null ? ` (limit ${percentText(canLimit)} %)` : ""}`}>
                <div className="ns-fold ns-row"><span className="ns-label">CAN</span><b style={{ color: loadColor(canLoad === null ? null : canLoad / canScale) }}>{percentText(canLoad, 1)}</b>
                    <em>%{canLimit !== null ? ` · limit ${percentText(canLimit)} %` : ""}</em></div>
                <span className="ns-meter-wrap"><LoadBar value={canLoad} full={canScale} />
                    {canLimit !== null && <s className="ns-limit" style={{ left: `${barPercent(canLimit)}%` }} />}</span>
            </div>
        </div>
    </div>;
}
