// Próba sprzętowa JOG TCP przez produkcyjny WebSocket panelu (te same komendy co przeglądarka:
// start → KEEPALIVE co 50 ms → stop). Uruchamiać wyłącznie po zatwierdzeniu karty próby przez operatora.
// Użycie: node scripts/tcp_jog_hardware_trial.mjs <wynik.json> [--base http://host:8080] [--linear 5] [--angular 3]
//         [--hold 3] [--rounds 1] [--axes x,y,z,rx,ry,rz] [--max-joint 25] [--reenable] [--frame base|tool] [--directions 1,-1]
import { writeFileSync } from "node:fs";

const args = process.argv.slice(2);
const output = args[0];
if (!output) { console.error("Podaj plik wyniku JSON"); process.exit(2); }
const option = (name, fallback) => { const i = args.indexOf(`--${name}`); return i < 0 ? fallback : args[i + 1]; };
const base = option("base", "http://192.168.1.69:8080");
const linear = Number(option("linear", "5")), angular = Number(option("angular", "3"));
const holdMs = Number(option("hold", "3")) * 1000, rounds = Number(option("rounds", "1"));
const axes = option("axes", "x,y,z,rx,ry,rz").split(",");
const maxJoint = Number(option("max-joint", "25"));
const frame = option("frame", "base");
const directions = option("directions", "1,-1").split(",").map(Number);
const reenable = args.includes("--reenable");
if (!["base", "tool"].includes(frame) || !directions.every(d => d === 1 || d === -1)) { console.error("Nieprawidłowy układ lub kierunek"); process.exit(2); }
if (!(linear > 0 && linear <= 20 && angular > 0 && angular <= 10 && holdMs > 0 && holdMs <= 15000 && maxJoint > 0 && maxJoint <= 45))
  { console.error("Parametry poza kartą próby"); process.exit(2); }

const delay = ms => new Promise(r => setTimeout(r, ms));
let state = null, stateAt = 0, sequence = 0;
const acks = new Map();
const socket = new WebSocket(base.replace(/^http/, "ws") + "/api/v1/manual/ws");
socket.onmessage = event => {
    const data = JSON.parse(event.data);
    if (data.type === "state") { state = data.state; stateAt = performance.now(); }
    else if (data.type === "tcp_ack") acks.get(data.sequence)?.(data);
};
await new Promise((resolve, reject) => { socket.onopen = resolve; socket.onerror = () => reject(Error("Brak połączenia WS")); });
const waitFor = async (predicate, ms, label) => {
    const until = performance.now() + ms;
    while (performance.now() < until) { if (state && performance.now() - stateAt < 250 && predicate(state)) return state; await delay(20); }
    throw Error(`Przekroczony czas: ${label}`);
};
const send = (intent, action, emergency = false) => {
    const id = ++sequence;
    const ack = new Promise(resolve => { acks.set(id, resolve); setTimeout(() => resolve({ accepted: false, message: "brak ACK 100 ms" }), 100); });
    socket.send(JSON.stringify({ ...intent, space: "tcp", action, emergency, sequence: id }));
    return ack;
};
const joints = s => s.axes.map(a => a.joint_position_known ? a.joint_position_degrees : NaN);
const canStatus = async () => (await (await fetch(base + "/api/v1/can/status")).json()).summary.values;
const counters = v => Object.fromEntries(["errors", "timeouts", "late_responses", "deferred", "missed_cycles", "committed_batches"].map(k => [k, v[k]]));

async function ensureReady() {
    await waitFor(s => s.fresh, 3000, "świeże dane");
    if (state.fault || state.axes.some(a => !a.enabled)) {
        if (!reenable) throw Error(`Napędy nie są gotowe (fault=${state.fault}, recovery=${state.recovery_state}); wymagane --reenable`);
        await waitFor(s => !s.busy && ["idle", "idle_disabled"].includes(s.recovery_state ?? "idle"), 15000, "postój po STOP");
        // Po STOP odzyskiwanie potwierdza postój pomiarami; ENABLE dopiero po jego zakończeniu.
        // Zabezpieczenie sekwencji ENABLE może ją przerwać (np. zmiana pozycji) — wtedy kolejna próba.
        let enabled = false, reason = "";
        for (let sequenceAttempt = 0; sequenceAttempt < 3 && !enabled; ++sequenceAttempt) {
            await waitFor(s => !s.busy && ["idle", "idle_disabled"].includes(s.recovery_state ?? "idle"), 20000, "postój po STOP");
            let accepted = false;
            for (let attempt = 0; attempt < 20 && !accepted; ++attempt) {
                const r = await fetch(base + "/api/v1/manual/enable-all", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ enabled: true }) });
                accepted = r.ok;if (!accepted) { reason = await r.text();await delay(500); }
            }
            if (!accepted) continue;
            try { await waitFor(s => s.axes.every(a => a.enabled) && !s.fault, 10000, "ENABLE J1–J6");enabled = true; }
            catch { reason = state?.message ?? "brak potwierdzenia ENABLE";await delay(1000); }
        }
        if (!enabled) throw Error("Załącz wszystkie nieudane: " + reason);
    }
    return waitFor(s => s.tcp_jog?.fresh && s.tcp_jog.ready && !s.tcp_jog.busy && !s.busy, 8000, "gotowość JOG TCP");
}

const results = [];
const before = counters(await canStatus());
try {
    for (let round = 1; round <= rounds; ++round) for (const axis of axes) for (const direction of directions) {
        const ready = await ensureReady();
        const start = joints(ready), tcp = ready.tcp_jog;
        const rotation = axis.startsWith("r"), velocity = rotation ? angular * Math.PI / 180 : linear / 1000;
        const intent = { axis, direction, frame, impulse: false, velocity, step: 0, model_id: tcp.model_id, revision: tcp.configuration_revision };
        const record = { round, frame, axis, direction, velocity: rotation ? `${angular} °/s` : `${linear} mm/s`, started: new Date().toISOString() };
        const status0 = counters(await canStatus());
        const startAck = await send(intent, "start");
        if (!startAck.accepted) { record.result = "start odrzucony: " + startAck.message; results.push(record); console.log(JSON.stringify(record)); await delay(1500); continue; }
        const t0 = performance.now();let abort = "", maxDeviation = 0, keepaliveGapMax = 0, last = t0;
        while (performance.now() - t0 < holdMs) {
            await delay(50);
            const now = performance.now();keepaliveGapMax = Math.max(keepaliveGapMax, now - last);last = now;
            const ack = await send(intent, "keepalive");
            const q = joints(state);maxDeviation = Math.max(maxDeviation, ...q.map((v, i) => Math.abs(v - start[i])).filter(Number.isFinite));
            if (state.fault || state.tcp_jog?.state === "failed") { abort = state.tcp_jog?.message || state.message; break; }
            if (maxDeviation > maxJoint) { abort = `przekroczony zakres próby ${maxDeviation.toFixed(2)}° > ${maxJoint}°`; await send(intent, "stop", true); break; }
            if (!ack.accepted) {
                await delay(150);
                abort = `KEEPALIVE odrzucony: ${ack.message}; TCP=${state.tcp_jog?.state}: ${state.tcp_jog?.message}; właściciel: ${state.message}`;
                if (state.tcp_jog?.busy) await send(intent, "stop", true);
                break;
            }
        }
        if (!abort) await send(intent, "stop");
        try { await waitFor(s => s.fault || !s.tcp_jog?.busy, 15000, "potwierdzony postój TCP"); } catch (e) { abort ||= e.message; }
        Object.assign(record, { held_ms: Math.round(performance.now() - t0), keepalive_gap_max_ms: Math.round(keepaliveGapMax),
            joint_deviation_max_deg: Number(maxDeviation.toFixed(3)), end_state: state.tcp_jog?.state, end_message: state.tcp_jog?.message,
            fault: !!state.fault || state.tcp_jog?.state === "failed", fault_message: abort || (state.fault ? state.message : ""),
            can_delta: Object.fromEntries(Object.entries(counters(await canStatus())).map(([k, v]) => [k, v - status0[k]])) });
        results.push(record);
        console.log(JSON.stringify(record));
        await delay(400);
    }
} catch (error) {
    results.push({ result: "przerwano", error: error.message });
    console.error(error.message);
} finally {
    const after = counters(await canStatus());
    writeFileSync(output, JSON.stringify({ base, frame, linear, angular, hold_ms: holdMs, rounds, axes, max_joint_deg: maxJoint,
        can_before: before, can_after: after, trials: results }, null, 2));
    socket.close();
}
