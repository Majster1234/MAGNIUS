import { contextKey, planningRequest, type PreviewContext } from "./planningContext";

export type TcpAxis = "x" | "y" | "z" | "rx" | "ry" | "rz";
export type TcpFrame = "base" | "tool";
export type TcpHeld = { axis: TcpAxis; direction: number; impulse?: boolean };

// Wyłącznie obliczenia symulacji. Jedno żądanie w toku, bez nadrabiania zaległości.
export class PlanningTcpJog {
    private generation = 0;
    private pending = false;
    private active: (TcpHeld & { context: PreviewContext; frame: TcpFrame; speed: number }) | null = null;
    private previous = 0;
    private sentAt: number | null = null;
    constructor(private apply: (positions: number[]) => boolean, private error: (message: string) => void,
        private request: typeof planningRequest = planningRequest) {}
    get held(): TcpHeld | null { return this.active && { axis: this.active.axis, direction: this.active.direction }; }
    start(context: PreviewContext, axis: TcpAxis, direction: number, frame: TcpFrame, speed: number, now: number) {
        if (this.active || this.pending || ![1, -1].includes(direction) || !Number.isFinite(speed) || speed <= 0) return false;
        this.generation++; this.previous = now; this.sentAt = null; this.active = { context, axis, direction, frame, speed }; return true;
    }
    stop() { this.generation++; this.active = null; }
    pulse(now: number, positions: (number | null)[], valid: () => boolean, plannerReady: boolean) {
        const active = this.active;
        if (!active) return;
        const elapsed = now - this.previous; this.previous = now;
        if (!valid() || elapsed < 0 || elapsed > 150 || positions.length !== 6 || !positions.every(q => q !== null && Number.isFinite(q))) { this.stop(); return; }
        if (this.pending || !plannerReady) return;
        const generation = this.generation, linear = ["x", "y", "z"].includes(active.axis);
        // Nie obcinaj posuwu do 1 mm na publikację: dawniej 100 mm/s dawało
        // tylko 20 mm/s. Czas uwzględnia wolniejszą odpowiedź, najwyżej 100 ms.
        const duration = active.context.tcp_jog_segmented ?
            this.sentAt === null ? .05 : Math.min(.1, Math.max(.01, (now - this.sentAt) / 1000)) : Math.min(.05, 1 / active.speed);
        if (duration < .01 || active.speed > (linear ? 100 : 30)) { this.stop(); return; }
        this.pending = true;
        this.sentAt = now;
        void this.request("planning/tcp-jog", {
            model_id: active.context.model_id, configuration_revision: active.context.configuration_revision,
            settings_revision: active.context.settings_revision, seed_degrees: [...positions],
            simulated_start: active.context.seed_sources.map(source => source !== "measured"),
            frame: active.frame, axis: active.axis, direction: active.direction, velocity: active.speed, duration_s: duration,
        }).then(result => {
            if (generation !== this.generation || !valid()) return;
            if (!result.simulation_only || result.execution_ready !== false || contextKey(result) !== contextKey(active.context) ||
                !Array.isArray(result.positions_degrees) || result.positions_degrees.length !== 6 || !result.positions_degrees.every(Number.isFinite) ||
                result.positions_degrees.some((q: number, i: number) => q < active.context.profiles[i].lower || q > active.context.profiles[i].upper ||
                    Math.abs(q - positions[i]!) > Math.min(180 / Math.PI * .25, active.context.profiles[i].velocity * duration) + 1e-6))
                throw Error("Nieprawidłowy wynik kroku TCP; cel pozostał bez zmian.");
            if (!this.apply(result.positions_degrees)) throw Error("Cel TCP nie jest już dostępny.");
        }).catch(error => {
            if (generation === this.generation) { this.stop(); this.error(error instanceof Error ? error.message : "Obliczenie TCP nie powiodło się."); }
        }).finally(() => { this.pending = false; });
    }
}
