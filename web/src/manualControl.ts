export type JogCommand = { action: "start" | "keepalive" | "stop"; axis: number; direction: number; rpm: number; sequence: number };
export interface JogSocket { readyState: number; send(data: string): void }
export const MANUAL_STATE_TIMEOUT_MS = 500;
export type StopReason = "release" | "ack_timeout" | "state_timeout" | "socket_closed" | "state_stale" | "ack_rejected";

// Jedno niepotwierdzone podtrzymanie; brak kolejki historycznych poleceń.
export class JogControl {
    private sequence = 0;
    private held: { axis: number; direction: number; rpm: number } | null = null;
    private pending = 0;
    private sentAt = 0;
    private stateAt = -Infinity;
    constructor(private socket: JogSocket, private clock: () => number = () => performance.now(),
        private onStop?: (reason: StopReason, axis: number) => void) {}
    get holding() { return this.held !== null; }
    stateReceived(fresh: boolean) { this.stateAt = fresh ? this.clock() : -Infinity; if (!fresh) this.stop("state_stale"); }
    start(axis: number, direction: number, rpm: number) {
        if (this.held || this.socket.readyState !== 1 || this.clock() - this.stateAt >= MANUAL_STATE_TIMEOUT_MS) return false;
        this.held = { axis, direction, rpm };
        this.send("start"); return true;
    }
    private send(action: JogCommand["action"]) {
        if (!this.held || this.socket.readyState !== 1) return;
        const sequence = ++this.sequence;
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
        if (!this.pending && this.clock() - this.sentAt >= 50) this.send("keepalive");
    }
    stop(reason: StopReason = "release") {
        if (this.held) this.onStop?.(reason, this.held.axis);
        this.send("stop"); this.held = null; this.pending = 0;
    }
}
