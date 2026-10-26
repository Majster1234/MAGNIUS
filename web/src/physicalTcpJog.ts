import type { TcpAxis, TcpFrame, TcpHeld } from "./planningTcpJog";
import type { JogSocket } from "./manualControl";
export type TcpJogStatus = {
    fresh: boolean; ready: boolean; busy: boolean; state: string; message: string;
    axis: number; direction: number; frame: string; impulse: boolean;
    progress: number; requested_step: number; velocity: number;
    max_linear_velocity: number; max_angular_velocity: number;
    model_id: string; configuration_revision: string; generation: number;
};
type Intent = TcpHeld & { frame: TcpFrame; impulse: boolean; velocity: number; step: number; model_id: string; revision: string };
export class PhysicalTcpJog {
    private active: Intent | null = null;
    private holding = false;
    private aborted = false;
    private generation = 0;
    private pending = 0;
    private sentAt = -Infinity;
    private receivedAt = -Infinity;
    private startedAt = -Infinity;
    private status: TcpJogStatus | undefined;
    constructor(private socket: JogSocket, private nextSequence: () => number,
        private clock = () => performance.now(), private error: (message: string) => void = () => {}) {}
    get held(): TcpHeld | null { return this.holding ? this.active : null; }
    get busy() { return this.active !== null; }
    receive(status: TcpJogStatus | undefined, fresh: boolean) {
        this.receivedAt = this.clock(); this.status = status;
        if (!this.active) return;
        if (!fresh || !status?.fresh || status.model_id !== this.active.model_id || status.configuration_revision !== this.active.revision) {
            this.stop(true); return;
        }
        if (status.generation > this.generation && !status.busy) { this.active = null; this.holding = false; this.pending = 0; }
        else if (!status.busy && status.ready && this.clock() - this.startedAt >= 500) { this.active = null; this.holding = false; this.pending = 0; }
    }
    start(axis: TcpAxis, direction: number, frame: TcpFrame, speed: number, step?: number) {
        const status = this.status;
        if (this.active || this.socket.readyState !== 1 || this.clock() - this.receivedAt >= 250 || !status?.fresh || !status.ready || status.busy ||
            ![1, -1].includes(direction) || !Number.isFinite(speed) || speed <= 0 || step !== undefined && (!Number.isFinite(step) || step <= 0)) return false;
        const linear = ["x", "y", "z"].includes(axis), factor = linear ? 1 / 1000 : Math.PI / 180;
        if (speed * factor > (linear ? status.max_linear_velocity : status.max_angular_velocity)) return false;
        this.generation = status.generation; this.startedAt = this.clock(); this.aborted = false;
        this.active = { axis, direction, frame, impulse: step !== undefined, velocity: speed * factor, step: (step ?? 0) * factor,
            model_id: status.model_id, revision: status.configuration_revision };
        this.holding = true; this.send("start"); return true;
    }
    private send(action: "start" | "keepalive" | "stop", emergency = false) {
        if (!this.active || this.socket.readyState !== 1) return;
        this.pending = this.nextSequence(); this.sentAt = this.clock();
        this.socket.send(JSON.stringify({ ...this.active, space: "tcp", action, emergency, sequence: this.pending }));
    }
    ack(sequence: number, accepted: boolean, message = "Komenda TCP odrzucona") {
        if (sequence !== this.pending) return;
        this.pending = 0;
        if (!accepted && this.active) { this.stop(true); this.error(message); }
    }
    pulse() {
        if (!this.active) return;
        if (this.socket.readyState !== 1 || this.clock() - this.receivedAt >= 250 || !this.status?.fresh ||
            this.pending && this.clock() - this.sentAt >= 100 || this.status.generation <= this.generation && this.clock() - this.startedAt >= 500) {
            this.stop(true); return;
        }
        if (this.holding && !this.pending && this.clock() - this.sentAt >= 50) this.send("keepalive");
    }
    stop(emergency = false) {
        if (!this.active || this.aborted || !emergency && !this.holding) return;
        this.send("stop", emergency); this.holding = false; this.aborted = emergency;
    }
}
