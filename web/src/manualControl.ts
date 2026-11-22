// Podtrzymanie JOG: zegar 50 ms z jitterem; próg 40 ms nie pomija co drugiego tyknięcia.
export const KEEPALIVE_MIN_INTERVAL_MS = 40;
export type JogCommand = { action: "start" | "keepalive" | "stop"; axis: number; direction: number; rpm: number; sequence: number; space?: "joint"; velocity_degrees_s?: number; revision?: string };
export interface JogSocket { readyState: number; send(data: string): void }
export const MANUAL_STATE_TIMEOUT_MS = 500;
export type StopReason = "release" | "ack_timeout" | "state_timeout" | "socket_closed" | "state_stale" | "ack_rejected";

// Jedno niepotwierdzone podtrzymanie; brak kolejki historycznych poleceń.
export class JogControl {
    private sequence = 0;
    private held: Omit<JogCommand, "action" | "sequence"> | null = null;
    private pending = 0;
    private sentAt = 0;
    private stateAt = -Infinity;
    constructor(private socket: JogSocket, private clock: () => number = () => performance.now(),
        private onStop?: (reason: StopReason, axis: number) => void, private nextSequence?: () => number) {}
    get holding() { return this.held !== null; }
    stateReceived(fresh: boolean) { this.stateAt = fresh ? this.clock() : -Infinity; if (!fresh) this.stop("state_stale"); }
    start(axis: number, direction: number, rpm: number, revision?: string) {
        if (this.held || this.socket.readyState !== 1 || this.clock() - this.stateAt >= MANUAL_STATE_TIMEOUT_MS) return false;
        this.held = revision ? {axis, direction, rpm: 0, space: "joint", velocity_degrees_s: rpm, revision} : { axis, direction, rpm };
        this.send("start"); return true;
    }
    private send(action: JogCommand["action"]) {
        if (!this.held || this.socket.readyState !== 1) return;
        const sequence = this.nextSequence ? this.nextSequence() : ++this.sequence;
        this.socket.send(JSON.stringify({ ...this.held, action, sequence }));
        this.pending = sequence; this.sentAt = this.clock();
    }
    ack(sequence: number, accepted: boolean) {
        if (sequence !== this.pending) return;
        this.pending = 0;
        if (!accepted) this.stop("ack_rejected");
    }
    pulse() {
        if (!this.held) return;
        if (this.socket.readyState !== 1 || this.clock() - this.stateAt >= MANUAL_STATE_TIMEOUT_MS || (this.pending && this.clock() - this.sentAt >= 100)) {
            this.stop(this.socket.readyState !== 1 ? "socket_closed" : this.clock() - this.stateAt >= MANUAL_STATE_TIMEOUT_MS ? "state_timeout" : "ack_timeout"); return;
        }
        // Ten sam próg co JOG TCP: tyknięcie 50 ms z jitterem nie przesuwa podtrzymania do 100 ms.
        if (!this.pending && this.clock() - this.sentAt >= KEEPALIVE_MIN_INTERVAL_MS) this.send("keepalive");
    }
    stop(reason: StopReason = "release") {
        if (this.held) this.onStop?.(reason, this.held.axis);
        this.send("stop"); this.held = null; this.pending = 0;
    }
}

export type JogSpeedProfile = { joint_configured?: boolean; jog_min_velocity_degrees_s?: number; jog_max_velocity_degrees_s?: number };
/** Wspólna prędkość JOG: procent maksymalnej prędkości osi. Przegub: °/s w krokach minimalnej prędkości osi (jak przy
 *  ograniczaniu do limitu), w planowaniu bez kroku; wał: całe obr./min (1…maxRpm). Brak limitu przegubu → null (bez zmiany). */
export function jogSpeedAtPercent(percent: number, profile: JogSpeedProfile, maxRpm: number, planning: boolean): number | null {
    const share = Math.max(1, Math.min(100, percent)) / 100;
    if (!profile.joint_configured) return Math.max(1, Math.min(maxRpm, Math.round(maxRpm * share)));
    const max = profile.jog_max_velocity_degrees_s ?? 0;
    if (max <= 0) return null;
    const step = planning ? 0 : profile.jog_min_velocity_degrees_s ?? 0, raw = max * share;
    if (step > 0 && max >= step) return Number(Math.max(step, Math.floor(raw / step + 1e-9) * step).toPrecision(12));
    return Math.floor(raw * 1000) / 1000;
}
