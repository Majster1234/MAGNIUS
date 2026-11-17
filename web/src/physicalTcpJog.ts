import type { TcpAxis, TcpFrame, TcpHeld } from "./planningTcpJog";
import { KEEPALIVE_MIN_INTERVAL_MS, type JogSocket } from "./manualControl";
export type TcpJogStatus = {
    fresh: boolean; ready: boolean; busy: boolean; state: string; message: string;
    axis: number; direction: number; frame: string; impulse: boolean;
    progress: number; requested_step: number; velocity: number;
    max_linear_velocity: number; max_angular_velocity: number;
    model_id: string; configuration_revision: string; generation: number;
};
type Intent = TcpHeld & { frame: TcpFrame; impulse: boolean; velocity: number; step: number; model_id: string; revision: string };
// Limity klienta (11.11.2026): KEEPALIVE w potoku co ≥ 40 ms bez czekania na ACK poprzedniego — opóźnione ACK
// (WiFi telefonu) nie wstrzymuje podtrzymania. Deadman pozostaje po stronie serwera: brak KEEPALIVE ≥ 150 ms ucina
// kolejkę (łagodne zatrzymanie). Klient przerywa sam dopiero przy ciszy ACK/stanu dłuższej niż poniższe progi.
export const TCP_ACK_TIMEOUT_MS = 400;
export const TCP_STATE_TIMEOUT_MS = 500;
const MAX_IN_FLIGHT = 12;
export class PhysicalTcpJog {
    private active: Intent | null = null;
    private holding = false;
    private aborted = false;
    private generation = 0;
    private inFlight: { sequence: number; at: number; action: string }[] = [];
    private sentAt = -Infinity;
    private receivedAt = -Infinity;
    private startedAt = -Infinity;
    private status: TcpJogStatus | undefined;
    constructor(private socket: JogSocket, private nextSequence: () => number,
        private clock = () => performance.now(), private error: (message: string) => void = () => {}) {}
    get held(): TcpHeld | null { return this.holding ? this.active : null; }
    get busy() { return this.active !== null; }
    private end() { this.active = null; this.holding = false; this.inFlight = []; }
    receive(status: TcpJogStatus | undefined, fresh: boolean) {
        this.receivedAt = this.clock(); this.status = status;
        if (!this.active) return;
        if (!fresh || !status?.fresh || status.model_id !== this.active.model_id || status.configuration_revision !== this.active.revision) {
            this.stop(true); return;
        }
        if (status.generation > this.generation && !status.busy) this.end();
        else if (!status.busy && status.ready && this.clock() - this.startedAt >= 500) this.end();
    }
    start(axis: TcpAxis, direction: number, frame: TcpFrame, speed: number, step?: number) {
        const status = this.status;
        // Własny ruch w dolocie (po puszczeniu) może przejąć nowy START — serwer kończy dolot atomowo.
        const preempt = !!this.active && !this.holding && !this.aborted && !!status?.busy && status.state === "settling";
        if (this.active && !preempt || this.socket.readyState !== 1 || this.clock() - this.receivedAt >= TCP_STATE_TIMEOUT_MS || !status?.fresh ||
            !preempt && (!status.ready || status.busy) ||
            ![1, -1].includes(direction) || !Number.isFinite(speed) || speed <= 0 || step !== undefined && (!Number.isFinite(step) || step <= 0)) return false;
        const linear = ["x", "y", "z"].includes(axis), factor = linear ? 1 / 1000 : Math.PI / 180;
        if (speed * factor > (linear ? status.max_linear_velocity : status.max_angular_velocity)) return false;
        this.generation = status.generation; this.startedAt = this.clock(); this.aborted = false; this.inFlight = [];
        this.active = { axis, direction, frame, impulse: step !== undefined, velocity: speed * factor, step: (step ?? 0) * factor,
            model_id: status.model_id, revision: status.configuration_revision };
        this.holding = true; this.send("start"); return true;
    }
    private send(action: "start" | "keepalive" | "stop", emergency = false) {
        if (!this.active || this.socket.readyState !== 1) return;
        const sequence = this.nextSequence(); this.sentAt = this.clock();
        this.inFlight.push({ sequence, at: this.sentAt, action }); if (this.inFlight.length > MAX_IN_FLIGHT) this.inFlight.shift();
        this.socket.send(JSON.stringify({ ...this.active, space: "tcp", action, emergency, sequence }));
    }
    ack(sequence: number, accepted: boolean, message = "Komenda TCP odrzucona") {
        const index = this.inFlight.findIndex(entry => entry.sequence === sequence);
        if (index < 0) return;
        const action = this.inFlight[index].action;
        this.inFlight.splice(0, index + 1);   // ACK w kolejności: wcześniejsze bez odpowiedzi nie blokują
        if (accepted || !this.active) return;
        // KEEPALIVE/STOP po zakończeniu sesji przez serwer (koniec drogi, łagodne zatrzymanie): bez komunikatu błędu.
        if (action !== "start" && (this.status?.generation ?? 0) > this.generation && !this.status?.busy) { this.end(); return; }
        this.stop(true); this.error(message);
    }
    pulse() {
        if (!this.active) return;
        const now = this.clock(), oldest = this.inFlight[0]?.at ?? Infinity;
        if (this.socket.readyState !== 1 || now - this.receivedAt >= TCP_STATE_TIMEOUT_MS || !this.status?.fresh ||
            now - oldest >= TCP_ACK_TIMEOUT_MS || this.status.generation <= this.generation && now - this.startedAt >= 500) {
            this.stop(true); return;
        }
        // Zegar panelu tyka co 50 ms z jitterem: próg 40 ms, żeby nie pomijać co drugiego tyknięcia (odstęp 100 ms,
        // próba 03.11: 168 ms > 150 ms ucięło kolejkę TCP mimo trzymanego przycisku).
        if (this.holding && now - this.sentAt >= KEEPALIVE_MIN_INTERVAL_MS) this.send("keepalive");
    }
    stop(emergency = false) {
        if (!this.active || this.aborted || !emergency && !this.holding) return;
        this.send("stop", emergency); this.holding = false; this.aborted = emergency;
    }
}
