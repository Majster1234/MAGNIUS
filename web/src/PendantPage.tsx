// Teach pendant na telefon (trasa #pendant, makieta temp/sterowanie_mobilne/v5_zezwolenie.html): tylko poziomo, pełny
// ekran, model 3D z limitami osi, drążek JOG TCP / osi z przyciskiem zezwolenia (deadman). Ten sam WebSocket i ci sami
// klienci JOG co sterowanie ręczne (JogControl, PhysicalTcpJog): podtrzymanie 50 ms (TCP w potoku), cisza ACK 400 ms /
// stanu 500 ms = łagodny STOP; rozłączenie = hamowanie po stronie serwera, ENABLE bez zmian. Logika drążka: pendantJog.ts.
// Główny wątek telefonu: stan w refie dla pętli sterowania, render React ≤ 10 Hz, scena 3D ≤ 20 kl./s.
import { useEffect, useRef, useState, type PointerEvent as ReactPointerEvent } from "react";
import { JogControl } from "./manualControl";
import type { Axis, ManualState } from "./manualTypes";
import { PhysicalTcpJog } from "./physicalTcpJog";
import { previewLimits, type LimitIndicator } from "./previewLimits";
import { activeConfigurationValues } from "./programApi";
import { axisLabel, JOINT_PAIRS, jointBlocker, jointSpeed, sameIntent, SPEED_PRESETS, StickSelector, TCP_PAIRS, tcpLabel, tcpSpeed,
    type Intent, type PendantMode } from "./pendantJog";
import type { TcpFrame } from "./planningTcpJog";

type Scene = ReturnType<typeof import("./programPreviewScene").createProgramScene>;
const levelOf = (list: LimitIndicator[]) => list.reduce<string>((a, x) => x.level === "warning" ? a || "warning" : x.level === "exceeded" || a !== "exceeded" ? x.level : a, "");

async function goFullscreen() {
    const el = document.documentElement as HTMLElement & { webkitRequestFullscreen?: () => Promise<void> };
    try { if (!document.fullscreenElement) await (el.requestFullscreen ? el.requestFullscreen({ navigationUI: "hide" }) : el.webkitRequestFullscreen?.()); } catch { /* np. iPhone — tylko „Do ekranu początkowego” */ }
    try { await (screen.orientation as ScreenOrientation & { lock?: (o: string) => Promise<void> }).lock?.("landscape"); } catch { /* przeglądarka bez blokady orientacji */ }
}

// Drążek: wartości w refie (bez renderu na każdy ruch), gałka przesuwana stylem; puszczenie = 0.
function Stick({ value, labels, disabled, onBlockedTouch }: { value: React.MutableRefObject<{ x: number; y: number }>; labels: string[]; disabled: boolean; onBlockedTouch: () => void }) {
    const ref = useRef<HTMLDivElement>(null), knob = useRef<HTMLDivElement>(null), active = useRef(false);
    const move = (e: ReactPointerEvent) => {
        const el = ref.current!, r = el.getBoundingClientRect(), radius = r.width / 2 - (knob.current?.offsetWidth ?? 60) / 2 + 6;
        let dx = (e.clientX - (r.left + r.width / 2)) / radius, dy = -(e.clientY - (r.top + r.height / 2)) / radius;
        const n = Math.hypot(dx, dy); if (n > 1) { dx /= n; dy /= n; }
        if (knob.current) knob.current.style.transform = `translate(calc(-50% + ${dx * radius}px), calc(-50% + ${-dy * radius}px))`;
        value.current = { x: dx, y: dy };
    };
    const release = () => { active.current = false; value.current = { x: 0, y: 0 }; ref.current?.classList.remove("held"); if (knob.current) knob.current.style.transform = "translate(-50%,-50%)"; };
    useEffect(() => { if (disabled) release(); }, [disabled]);   // eslint-disable-line react-hooks/exhaustive-deps
    return <div ref={ref} className={`pd-stick${disabled ? " disabled" : ""}`} role="application" aria-label="Drążek JOG"
        onPointerDown={e => { if (disabled) { onBlockedTouch(); return; } active.current = true; ref.current?.setPointerCapture(e.pointerId); ref.current?.classList.add("held"); move(e); }}
        onPointerMove={e => { if (active.current) move(e); }} onPointerUp={release} onPointerCancel={release} onLostPointerCapture={release}>
        <span className="ax t">{labels[0]}</span><span className="ax b">{labels[1]}</span><span className="ax l">{labels[2]}</span><span className="ax r">{labels[3]}</span>
        <div ref={knob} className="knob" />
    </div>;
}

export function PendantPage() {
    const [state, setState] = useState<ManualState | null>(null);
    const [connected, setConnected] = useState(false);
    const [mode, setMode] = useState<PendantMode>("tcp");
    const [pair, setPair] = useState(0);
    const [frame, setFrame] = useState<TcpFrame>("base");
    const [speed, setSpeed] = useState<(typeof SPEED_PRESETS)[number]["id"]>("slow");
    const [enabled, setEnabled] = useState(false);          // zezwolenie (deadman) wciśnięte
    const [stopped, setStopped] = useState(false);          // zatrzask STOP
    const [moving, setMoving] = useState<Intent | null>(null);
    const [note, setNote] = useState<{ text: string; until: number } | null>(null);
    const [startOverlay, setStartOverlay] = useState(() => new URLSearchParams(location.search).get("fs") === "1" && !document.fullscreenElement);
    const [fullscreen, setFullscreen] = useState(!!document.fullscreenElement);
    const stick = useRef({ x: 0, y: 0 });
    const latest = useRef<ManualState | null>(null);       // najnowszy stan (pętla sterowania, scena) — bez czekania na render

    const live = useRef({ mode, pair, frame, speed, enabled, stopped, state: null as ManualState | null });
    live.current = { mode, pair, frame, speed, enabled, stopped, state: latest.current };
    const jog = useRef<JogControl | null>(null), tcp = useRef<PhysicalTcpJog | null>(null);
    const active = useRef<Intent | null>(null), cooldown = useRef(0);
    const host = useRef<HTMLDivElement>(null), scene = useRef<Scene | null>(null);
    const blocked = useRef(false);
    const startGeneration = useRef(0);                      // generacja JOG TCP przy starcie (odrzucony start ≠ koniec drogi)                          // koniec ruchu przez serwer (limit, osobliwość): czekaj na środek drążka
    const say = (text: string) => setNote({ text, until: performance.now() + 4000 });

    // Widok: tytuł, viewport bez powiększania, manifest „pełny ekran / poziomo” (instalacja na ekranie początkowym).
    useEffect(() => {
        document.title = "Teach pendant · Robot 6-DOF"; document.body.classList.add("pendant-body");
        const viewport = document.querySelector('meta[name="viewport"]'), previous = viewport?.getAttribute("content");
        viewport?.setAttribute("content", "width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover");
        const manifest = document.createElement("link"); manifest.rel = "manifest"; manifest.href = "/pendant.manifest.json"; document.head.appendChild(manifest);
        const full = () => setFullscreen(!!document.fullscreenElement); document.addEventListener("fullscreenchange", full);
        return () => { document.body.classList.remove("pendant-body"); if (previous) viewport?.setAttribute("content", previous); manifest.remove(); document.removeEventListener("fullscreenchange", full); };
    }, []);

    // Model 3D (geometria aktywnej konfiguracji) ze wskaźnikami limitów jak w sterowaniu ręcznym.
    useEffect(() => {
        let closed = false;
        void Promise.all([import("./programPreviewScene"), activeConfigurationValues()]).then(([engine, values]) => {
            if (closed || !host.current) return;
            try { scene.current = engine.createProgramScene(host.current, { maxFps: 20 }); scene.current.setGeometry(values); scene.current.setPositions([], null); }
            catch { host.current.insertAdjacentHTML("beforeend", '<p class="pd-noscene">Podgląd 3D niedostępny (WebGL)</p>'); }
        });
        return () => { closed = true; scene.current?.dispose(); scene.current = null; };
    }, []);
    const showPose = (next: ManualState) => {
        scene.current?.setPose(next.axes.map(a => a.joint_position_known ? a.joint_position_degrees ?? 0 : 0));
        scene.current?.setLimits(next.axes.map(a => previewLimits(a, a.joint_position_known ? a.joint_position_degrees ?? null : null, next.fresh)));
    };

    // Połączenie JOG (jak ManualPage): stan 20 Hz, ACK, ponowne łączenie co 1 s.
    useEffect(() => {
        let socket: WebSocket | null = null, closed = false, reconnect: ReturnType<typeof setTimeout> | undefined;
        let received = Number.NaN, reported = 0, rendered = 0;
        // Znaczniki diagnostyczne jak w panelu PC (nagranie log/manual-can: browser_state_gap/received, client_stop).
        const report = (event: string, gap = 0, axis = 0, reason = 0) => {
            if (socket?.readyState !== 1 || latest.current?.recording !== true) return;
            socket.send(JSON.stringify({ type: "trace", event, client_epoch_ms: Date.now(), client_perf_ms: performance.now(), gap_ms: Number.isFinite(gap) ? gap : 0,
                axis, reason, state_monotonic_ns: latest.current?.state_monotonic_ns ?? "" }));
        };
        const connect = () => {
            let sequence = 0; const next = () => ++sequence;
            socket = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/api/v1/manual/ws`);
            jog.current = new JogControl(socket, () => performance.now(), (reason, axis) => {
                report("client_stop", 0, axis, ["release", "ack_timeout", "state_timeout", "socket_closed", "state_stale", "ack_rejected"].indexOf(reason));
                if (reason !== "release") say(`J${axis}: przerwano JOG (${reason}) — puść drążek`);
            }, next);
            tcp.current = new PhysicalTcpJog(socket, next, () => performance.now(), message => say(`JOG TCP: ${message}`));
            socket.onopen = () => { if (!closed) setConnected(true); };
            socket.onmessage = event => {
                try {
                    const data = JSON.parse(event.data);
                    if (data.type === "state" && Array.isArray(data.state?.axes) && data.state.axes.length === 6) {
                        const now = performance.now(), gap = now - received; received = now; latest.current = data.state;
                        tcp.current?.receive(data.state.tcp_jog, data.state.fresh); jog.current?.stateReceived(data.state.fresh);
                        if (gap >= 100 || now - reported >= 1000) { report(gap >= 100 ? "state_gap" : "state_received", gap); reported = now; }
                        showPose(data.state);
                        if (now - rendered >= 100) { rendered = now; setState(data.state); }
                    } else if (data.type === "ack") { jog.current?.ack(data.sequence, data.accepted); if (!data.accepted) say(data.message || "Komenda JOG odrzucona"); }
                    else if (data.type === "tcp_ack") tcp.current?.ack(data.sequence, data.accepted, data.message);
                } catch { tcp.current?.stop(true); jog.current?.stop(); }
            };
            socket.onclose = () => {
                tcp.current?.stop(true); jog.current?.stop(); active.current = null; setMoving(null);
                if (!closed) { setConnected(false); reconnect = setTimeout(connect, 1000); }
            };
            socket.onerror = () => socket?.close();
        };
        connect();
        return () => { closed = true; if (reconnect) clearTimeout(reconnect); tcp.current?.stop(true); jog.current?.stop(); socket?.close(); };
    }, []);

    // Zegar sterowania 50 ms: drążek → zamiar → start/stop jednego JOG; podtrzymanie.
    useEffect(() => {
        const selector = new StickSelector();
        const stopActive = (emergency = false) => {
            const current = active.current; if (!current) return;
            if (current.mode === "joint") jog.current?.stop(); else tcp.current?.stop(emergency);
            active.current = null; cooldown.current = performance.now() + 200; setMoving(null);
        };
        const timer = setInterval(() => {
            const { mode, pair, frame, speed, enabled, stopped } = live.current, state = latest.current, now = performance.now();
            const portrait = matchMedia("(orientation: portrait)").matches;
            let want = enabled && !stopped && !document.hidden && !portrait && state?.fresh ? selector.update(stick.current.x, stick.current.y, mode, pair) : (selector.reset(), null);
            if (blocked.current) { if (Math.hypot(stick.current.x, stick.current.y) < .2 || !want) blocked.current = false; else want = null; }
            // Serwer zakończył JOG TCP przy trzymanym drążku: po zakłóceniu łączności (KEEPALIVE) wznowienie po chwili;
            // koniec drogi (limit, osobliwość, uchyb) — komunikat i blokada do powrotu drążka do środka.
            if (active.current?.mode === "tcp" && !tcp.current?.busy) {
                const message = state?.tcp_jog?.message ?? "";
                active.current = null; setMoving(null);
                if ((state?.tcp_jog?.generation ?? 0) <= startGeneration.current) cooldown.current = now + 500;   // start odrzucony: ponów
                else if (/KEEPALIVE|łączności/.test(message)) { cooldown.current = now + 300; say("Chwilowa utrata łączności — wznawianie ruchu"); }
                else if (want) { blocked.current = true; want = null; say(`JOG TCP: ${message.replace(/^Zatrzymanie TCP potwierdzone;?\s*/, "") || "zatrzymano"} — puść drążek`); }
            }
            if (!sameIntent(want, active.current)) stopActive();
            if (want && !active.current && now >= cooldown.current && state) {
                const fraction = SPEED_PRESETS.find(p => p.id === speed)!.fraction;
                let started = false;
                if (want.mode === "joint") {
                    const axis = state.axes[want.axis - 1], blocker = jointBlocker(axis, state.configuration_revision);
                    if (blocker) say(`J${want.axis}: ${blocker}`);
                    else started = !!jog.current?.start(want.axis, want.direction, jointSpeed(axis, fraction), state.configuration_revision);
                } else {
                    const status = state.tcp_jog;
                    // Dolot poprzedniego ruchu (settling) można przejąć nowym startem — serwer robi to atomowo.
                    if (!status?.ready && !(status?.busy && status.state === "settling")) say(`JOG TCP: ${status?.message || "niegotowy"}`);
                    else { startGeneration.current = status.generation; started = !!tcp.current?.start(want.axis, want.direction, frame, tcpSpeed(status, want.axis, fraction)); }
                }
                if (started) { active.current = want; setMoving(want); }
                else cooldown.current = now + 500;   // bez zalewania komendami; ponowna próba po 0,5 s
            }
            jog.current?.pulse(); tcp.current?.pulse();
        }, 50);
        return () => { clearInterval(timer); stopActive(true); };
    }, []);

    // Utrata fokusu / ukrycie / zamknięcie: zwolnienie zezwolenia.
    useEffect(() => {
        const drop = () => setEnabled(false);
        const visibility = () => { if (document.hidden) drop(); };
        window.addEventListener("blur", drop); window.addEventListener("pagehide", drop); document.addEventListener("visibilitychange", visibility);
        return () => { window.removeEventListener("blur", drop); window.removeEventListener("pagehide", drop); document.removeEventListener("visibilitychange", visibility); };
    }, []);
    useEffect(() => { if (!note) return; const t = setTimeout(() => setNote(null), Math.max(0, note.until - performance.now())); return () => clearTimeout(t); }, [note]);

    function emergencyStop() {
        setEnabled(false); setStopped(true); tcp.current?.stop(true); jog.current?.stop(); active.current = null; setMoving(null);
        void fetch("/api/v1/program-run/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" }).catch(() => undefined);
    }
    async function enableDrives() {
        if (!window.confirm("Załączyć wszystkie napędy (ENABLE=1)? Silniki zaczną trzymać pozycję.")) return;
        try {
            const response = await fetch("/api/v1/manual/enable-all", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ enabled: true }) });
            const body = await response.json().catch(() => ({}));
            say(response.ok ? "Wysłano ENABLE wszystkich napędów" : body.detail || `Błąd ${response.status}`);
        } catch (error) { say(`ENABLE: ${(error as Error).message}`); }
    }

    const axes: Axis[] = state?.axes ?? [];
    const limits = axes.map(a => previewLimits(a, a.joint_position_known ? a.joint_position_degrees ?? null : null, !!state?.fresh));
    const nearest = limits.flatMap((list, j) => list.map(x => ({ ...x, j }))).sort((a, b) => a.distanceDegrees - b.distanceDegrees)[0];
    const disabledDrives = axes.filter(a => a.available && a.enable_known && !a.enabled).map(a => `J${a.axis}`);
    const tcpStatus = state?.tcp_jog;
    const warning = !connected ? { text: "Brak połączenia z serwerem — ponowne łączenie…", level: "danger" }
        : !state?.fresh ? { text: "Dane osi nieaktualne — JOG zablokowany", level: "danger" }
        : state.fault ? { text: `Awaria: ${state.recovery_message || state.message}`, level: "danger" }
        : note ? { text: note.text, level: "warning" }
        : mode === "tcp" && tcpStatus && !tcpStatus.ready ? { text: `JOG TCP: ${tcpStatus.message}`, level: "warning" }
        : nearest ? { text: nearest.level === "exceeded" ? `J${nearest.j + 1}: przekroczony limit ${nearest.side === "lower" ? "−" : "+"} (${nearest.limitDegrees}°)`
            : `J${nearest.j + 1}: ${nearest.distanceDegrees.toFixed(1)}° do limitu ${nearest.side === "lower" ? "−" : "+"}`, level: nearest.level } : null;
    const pairs = mode === "joint" ? JOINT_PAIRS.map(([a, b]) => `J${a}·J${b}`) : TCP_PAIRS.map(([a, b]) => `${tcpLabel(a)}·${tcpLabel(b)}`);
    const [ax, ay] = mode === "joint" ? JOINT_PAIRS[pair].map(j => `J${j}`) : TCP_PAIRS[pair].map(tcpLabel);
    const changeMode = (next: PendantMode) => { setMode(next); setPair(0); };

    return <div className={`pd-app${enabled ? " enabled" : ""}${stopped ? " stopped" : ""}${mode === "joint" ? " joint" : ""}`}>
        <section className="pd-view" ref={host} />
        <div className="pd-top">
            <span className={`pd-badge ${connected && state?.fresh ? "ok" : "bad"}`}><i />{connected ? state?.fresh ? "Połączono" : "Dane nieaktualne" : "Brak połączenia"}</span>
            <span className="pd-badge mode">{mode === "joint" ? "Osie" : `TCP · ${frame === "base" ? "baza" : "narzędzie"}`}</span>
            <span className={`pd-badge ${enabled ? "ok" : ""}`}>{enabled ? "✅ zezwolenie" : "⛔ JOG zablokowany"}</span>
            {moving && <span className="pd-badge move">▶ {axisLabel(moving)}{moving.direction > 0 ? "+" : "−"}</span>}
            <span className="spacer" />
            {mode === "tcp" && <div className="pd-seg"><button className={frame === "base" ? "on" : ""} onClick={() => setFrame("base")}>Baza</button>
                <button className={frame === "tool" ? "on" : ""} onClick={() => setFrame("tool")}>Narz.</button></div>}
            <a className="pd-icon" href="#overview" title="Zamknij pendant (panel główny)">✕</a>
            <button className="pd-icon" title={fullscreen ? "Wyjdź z pełnego ekranu" : "Pełny ekran"} onClick={() => fullscreen ? void document.exitFullscreen() : void goFullscreen()}>
                <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="2.2" strokeLinecap="round"><path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5" /></svg></button>
        </div>
        {warning && <div className="pd-warn" data-level={warning.level}>{warning.text}</div>}
        {disabledDrives.length > 0 && connected && state?.fresh && !state.fault &&
            <button className="pd-enable" onClick={() => void enableDrives()}>⚡ Załącz napędy ({disabledDrives.join(", ")})</button>}
        <div className="pd-deadman" role="button" aria-pressed={enabled} aria-label="Zezwolenie — trzymaj"
            onPointerDown={e => { if (stopped) { say("Zwolnij zatrzask STOP"); return; } (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId); setEnabled(true); }}
            onPointerUp={() => setEnabled(false)} onPointerCancel={() => setEnabled(false)} onLostPointerCapture={() => setEnabled(false)}
            onContextMenu={e => e.preventDefault()}>
            <span className="dm-icon">✋</span>ZEZWOLENIE<small>trzymaj kciukiem</small></div>
        <div className="pd-right">
            <div className="pd-seg mode"><button className={mode === "tcp" ? "on" : ""} onClick={() => changeMode("tcp")}>TCP</button>
                <button className={mode === "joint" ? "on" : ""} onClick={() => changeMode("joint")}>Osie</button></div>
            <div className="pd-chips">{pairs.map((text, k) => <button key={text} className={`pd-chip${k === pair ? " on" : ""}`} onClick={() => setPair(k)}>{text}</button>)}</div>
            <div className="pd-stickwrap"><Stick value={stick} labels={[`${ay}+`, `${ay}−`, `${ax}−`, `${ax}+`]} disabled={!enabled || stopped}
                onBlockedTouch={() => say(stopped ? "Zwolnij zatrzask STOP" : "Przytrzymaj ZEZWOLENIE lewym kciukiem")} /></div>
            <div className="pd-presets"><span>Prędkość</span>{SPEED_PRESETS.map(p => <button key={p.id} className={`pd-chip${speed === p.id ? " on" : ""}`}
                title={`${Math.round(p.fraction * 100)}% limitu JOG`} onClick={() => setSpeed(p.id)}>{p.label}</button>)}</div>
        </div>
        <div className="pd-bottom">
            <button className="pd-stop" onClick={() => stopped ? setStopped(false) : emergencyStop()}>{stopped ? "▶ ODBLOKUJ" : "STOP"}</button>
            <div className="pd-joints">{axes.map((a, j) => { const lo = a.joint_lower_degrees ?? -180, hi = a.joint_upper_degrees ?? 180, q = a.joint_position_degrees ?? 0;
                return <div key={a.axis} className="pd-j" data-level={levelOf(limits[j])} title={`J${a.axis}: ${q.toFixed(1)}° (${lo}…${hi}°)`}>J{a.axis}
                    <div className="bar"><i style={{ bottom: `${Math.min(100, Math.max(0, (q - lo) / (hi - lo) * 100))}%` }} /></div><span>{a.joint_position_known ? q.toFixed(0) : "—"}</span></div>; })}</div>
        </div>
        <div className="pd-rotate"><div className="rot-icon">📱↻</div><b>Obróć telefon poziomo</b><span>Teach pendant działa tylko w orientacji poziomej.</span>
            <button onClick={() => void goFullscreen()}>Pełny ekran — poziomo</button></div>
        {startOverlay && <div className="pd-start" onClick={() => { void goFullscreen(); setStartOverlay(false); }}>
            <div className="pd-start-card"><img src="/robot.svg" alt="" /><b>Teach pendant</b><span>Dotknij, aby uruchomić w pełnym ekranie (poziomo).</span>
                <button className="go">▶ Uruchom</button>
                <button className="link" onClick={e => { e.stopPropagation(); setStartOverlay(false); }}>bez pełnego ekranu</button></div></div>}
    </div>;
}
