import { contextKey, planningRequest, type PreviewContext } from "./planningContext";
import { pointTime, trajectoryPosition, validateCartesianPlan } from "./cartesianTrajectory";
import type { TcpAxis, TcpFrame, TcpHeld } from "./planningTcpJog";

export const TCP_STEPS = [.1, 1, 5, 10, 20] as const;
type Step = { context: PreviewContext; axis: TcpAxis; direction: number; frame: TcpFrame; speed: number; amount: number; positions: number[]; valid: () => boolean };

// Jeden zatwierdzony LIN, odtworzony raz w 60 Hz. Brak kolejki impulsów i transportu sprzętu.
export class PlanningTcpStep {
    private active: Step | null = null;
    private generation = 0;
    private previous = 0;
    private pending = false;
    private animation = 0;
    private job: string | null = null;
    private cancelling: string | null = null;
    private cancellation: Promise<unknown> = Promise.resolve();
    private work: Promise<unknown> = Promise.resolve();
    constructor(private apply: (positions: number[]) => boolean, private error: (message: string) => void,
        private request: typeof planningRequest = planningRequest, private clock = () => performance.now(),
        private schedule = (frame: FrameRequestCallback) => requestAnimationFrame(frame),
        private unschedule = (id: number) => cancelAnimationFrame(id)) {}
    get held(): TcpHeld | null { return this.active && { axis: this.active.axis, direction: this.active.direction, impulse: true }; }
    get busy() { return !!this.active || this.pending || !!this.cancelling; }
    settled() { return this.work.then(() => this.cancellation); }
    private cancel(id: string) {
        if (this.cancelling === id) return this.cancellation;
        this.cancelling = id; if (this.job === id) this.job = null;
        this.cancellation = this.request(`planning/jobs/${id}`, {}, "DELETE").finally(() => { if (this.cancelling === id) this.cancelling = null; });
        void this.cancellation.catch(() => {});
        return this.cancellation;
    }
    stop() {
        this.generation++; this.active = null; this.unschedule(this.animation); this.animation = 0;
        if (this.job) void this.cancel(this.job).catch(error => this.error(`Nie potwierdzono anulowania impulsu: ${String(error)}`));
    }
    start(context: PreviewContext, axis: TcpAxis, direction: number, frame: TcpFrame, speed: number, amount: number,
        positions: (number | null)[], valid: () => boolean, now: number) {
        if (this.busy || !context.tcp_step_available || !TCP_STEPS.includes(amount as typeof TCP_STEPS[number]) ||
            ![1, -1].includes(direction) || !Number.isFinite(speed) || speed <= 0 || speed > (["x", "y", "z"].includes(axis) ? 100 : 30) ||
            positions.length !== 6 || !positions.every(q => q !== null && Number.isFinite(q)) || !valid()) return false;
        this.generation++; this.previous = now;
        this.active = { context, axis, direction, frame, speed, amount, positions: [...positions] as number[], valid }; return true;
    }
    pulse(now: number, plannerReady: boolean) {
        const active = this.active;
        if (!active) return;
        const gap = now - this.previous; this.previous = now;
        if (!active.valid() || gap < 0 || gap > 150) { this.stop(); return; }
        if (this.pending || this.animation || !plannerReady) return;
        const generation = this.generation, valid = () => generation === this.generation && !!this.active && active.valid();
        this.pending = true;
        this.work = (async () => {
            const started = await this.request("planning/tcp-step", {
                model_id: active.context.model_id, configuration_revision: active.context.configuration_revision,
                settings_revision: active.context.settings_revision, seed_degrees: [...active.positions],
                simulated_start: active.context.seed_sources.map(source => source !== "measured"), axis: active.axis,
                direction: active.direction, frame: active.frame, velocity: active.speed, step: active.amount,
            });
            if (!valid()) { await this.cancel(started.id); return; }
            this.job = started.id;
            for (;;) {
                const status = await this.request(`planning/jobs/${started.id}`);
                if (!valid()) { await this.cancel(started.id); return; }
                if (status.status === "succeeded") {
                    this.job = null;
                    const plan = await this.request(`planning/plans/${status.plan_id}`);
                    if (!valid()) return;
                    validateCartesianPlan(plan);
                    if (!plan.simulation_only || plan.execution_ready !== false || plan.motion_type !== "LIN" ||
                        contextKey(plan) !== contextKey(active.context) || !Array.isArray(plan.start_positions) || plan.start_positions.length !== 6 ||
                        plan.start_positions.some((q: number, i: number) => !Number.isFinite(q) || Math.abs(q * 180 / Math.PI - active.positions[i]) > 1e-5) ||
                        plan.trajectory.points.some((point: { positions: number[]; velocities: number[] }) => point.positions.some((q, i) =>
                            q * 180 / Math.PI < active.context.profiles[i].lower - 1e-6 || q * 180 / Math.PI > active.context.profiles[i].upper + 1e-6 ||
                            Math.abs(point.velocities[i]) * 180 / Math.PI > active.context.profiles[i].velocity + 1e-5)))
                        throw Error("Nieprawidłowa trajektoria impulsu TCP; cel pozostał bez zmian.");
                    const total = pointTime(plan.trajectory.points.at(-1)!), epoch = this.clock(); let previous = epoch;
                    const frame = (now: number) => {
                        if (!valid() || now < previous || now - previous > 150) { this.stop(); return; }
                        previous = now;
                        const seconds = Math.min(total, (now - epoch) / 1000);
                        if (!this.apply(trajectoryPosition(plan, seconds))) { this.stop(); return; }
                        if (seconds >= total) { this.animation = 0; this.active = null; return; }
                        this.animation = this.schedule(frame);
                    };
                    this.animation = this.schedule(frame); return;
                }
                if (status.status !== "running") { this.job = null; throw Error(status.message || "Impuls TCP anulowany"); }
                await new Promise(resolve => setTimeout(resolve, 100));
            }
        })().catch(error => {
            if (valid()) { this.stop(); this.error(error instanceof Error ? error.message : "Nie można obliczyć impulsu TCP."); }
        }).finally(() => { this.pending = false; });
    }
}
