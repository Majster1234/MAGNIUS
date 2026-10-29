import { expect, it, vi } from "vitest";
import { PlanningTcpStep, TCP_STEPS } from "./planningTcpStep";
import type { PreviewContext } from "./planningContext";
const context: PreviewContext = { model_id: "m", configuration_revision: "r", settings_revision: "s", planner_available: true, tcp_step_available: true,
    ignore_missing_drivers: true, available: Array(6).fill(false), preview_seed_degrees: Array(6).fill(0), seed_sources: Array(6).fill("zero"),
    profiles: Array.from({ length: 6 }, () => ({ configured: true, lower: -90, upper: 90, velocity: 30 })) };
function plan(changes = {}) {
    return { ...context, id: "p", simulation_only: true, execution_ready: false, motion_type: "LIN", start_positions: Array(6).fill(0),
        trajectory: { joint_names: Array.from({ length: 6 }, (_, i) => `joint_${i + 1}`), points: [0, 1].map(t => ({
            positions: [t * Math.PI / 180, 0, 0, 0, 0, 0], velocities: Array(6).fill(0), accelerations: Array(6).fill(0), time_from_start: { sec: t, nanosec: 0 },
        })) }, ...changes };
}
function setup(result = plan()) {
    const apply = vi.fn((_positions: number[]) => true), error = vi.fn(); let now = 0, frame: FrameRequestCallback | null = null, sequence = 0, valid = true;
    const request = vi.fn(async (path: string) => path === "planning/tcp-step" ? { id: "j" } : path === "planning/plans/p" ? result : { status: "succeeded", plan_id: "p" });
    const step = new PlanningTcpStep(apply, error, request, () => now, callback => { frame = callback; return ++sequence; }, () => { frame = null; });
    return { step, apply, error, request, valid: () => valid, invalidate: () => { valid = false; },
        tick: (time: number, stamp = time) => { now = time; const callback = frame; frame = null; callback?.(stamp); } };
}
it("czeka na anulowanie PTP, wykonuje jeden impuls z interpolacją 60 Hz i zatrzymuje się na celu", async () => {
    const s = setup();
    expect(s.step.start(context, "x", 1, "tool", 10, 1, Array(6).fill(0), s.valid, 0)).toBe(true);
    s.step.pulse(50, false); expect(s.request).not.toHaveBeenCalled();
    s.step.pulse(100, true); await s.step.settled();
    for (let time = 16; time < 1000; time += 16) s.tick(time);
    s.tick(1000);
    expect(s.apply.mock.calls.length).toBeGreaterThan(50);
    expect(s.apply.mock.calls[0][0][0]).toBeGreaterThan(0); expect(s.apply.mock.calls[0][0][0]).toBeLessThan(.001);
    expect(s.apply).toHaveBeenLastCalledWith([1, 0, 0, 0, 0, 0]); expect(s.step.held).toBeNull();
    s.step.pulse(1100, true); expect(s.request.mock.calls.filter(call => call[0] === "planning/tcp-step")).toHaveLength(1);
    expect(s.error).not.toHaveBeenCalled();
});
it.each(TCP_STEPS)("wspólny wybór %s dla przesunięcia i obrotu bez kolejkowania", async amount => {
    for (const axis of ["x", "rz"] as const) {
        const s = setup(); s.step.start(context, axis, -1, "base", 5, amount, Array(6).fill(0), s.valid, 0);
        expect(s.step.start(context, axis, 1, "base", 5, amount, Array(6).fill(0), s.valid, 0)).toBe(false);
        s.step.pulse(50, true); await s.step.settled();
        expect(s.request).toHaveBeenCalledWith("planning/tcp-step", expect.objectContaining({ axis, direction: -1, step: amount })); s.step.stop();
    }
});
it.each(["stop", "invalid", "gap"])("przerywa animację i nie wznawia jej: %s", async reason => {
    const s = setup(); s.step.start(context, "x", 1, "base", 10, 1, Array(6).fill(0), s.valid, 0);
    s.step.pulse(50, true); await s.step.settled(); s.tick(16);
    if (reason === "stop") s.step.stop(); else if (reason === "invalid") s.invalidate(); else s.tick(300);
    s.tick(320); const before = s.apply.mock.calls.length; s.tick(336);
    expect(s.apply).toHaveBeenCalledTimes(before); expect(s.step.held).toBeNull();
});
it.each([{ execution_ready: true }, { settings_revision: "old" }, { start_positions: Array(6).fill(1) }])("odrzuca błędny wynik przed ruchem", async changes => {
    const s = setup(plan(changes)); s.step.start(context, "x", 1, "base", 10, 1, Array(6).fill(0), s.valid, 0);
    s.step.pulse(50, true); await s.step.settled(); s.tick(16);
    expect(s.apply).not.toHaveBeenCalled(); expect(s.error).toHaveBeenCalledOnce();
});
it("anuluje spóźnione przyjęcie zadania po zatrzymaniu", async () => {
    let answer!: (value: unknown) => void;
    const request = vi.fn((path: string) => path === "planning/tcp-step" ? new Promise(resolve => { answer = resolve; }) : Promise.resolve({ status: "cancelled" }));
    const apply = vi.fn(() => true), step = new PlanningTcpStep(apply, vi.fn(), request, () => 0, () => 1, () => {});
    step.start(context, "x", 1, "base", 10, 1, Array(6).fill(0), () => true, 0); step.pulse(50, true); step.stop();
    answer({ id: "late" }); await step.settled();
    expect(request).toHaveBeenCalledWith("planning/jobs/late", {}, "DELETE"); expect(apply).not.toHaveBeenCalled();
});

it("starszy znacznik pierwszej klatki RAF nie przerywa zatwierdzonego impulsu", async () => {
    const s = setup(); s.step.start(context, "x", 1, "base", 10, 1, Array(6).fill(0), s.valid, 0);
    s.step.pulse(50, true); await s.step.settled();
    s.tick(16, -16); expect(s.step.held).not.toBeNull(); expect(s.apply).toHaveBeenCalledOnce();
    s.tick(32, 0); expect(s.apply.mock.calls[1][0][0]).toBeGreaterThan(s.apply.mock.calls[0][0][0]);
    s.tick(300); expect(s.step.held).toBeNull();
});
