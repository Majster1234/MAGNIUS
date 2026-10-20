import { expect, it, vi } from "vitest";
import { PlanningTcpJog } from "./planningTcpJog";
import type { PreviewContext } from "./planningContext";
const context: PreviewContext = { model_id: "m", configuration_revision: "r", settings_revision: "s", planner_available: true, tcp_jog_segmented: true,
    ignore_missing_drivers: true, available: Array(6).fill(false), preview_seed_degrees: Array(6).fill(0), seed_sources: Array(6).fill("zero"),
    profiles: Array.from({ length: 6 }, () => ({ configured: true, lower: -90, upper: 90, velocity: 30 })) };
const answer = (changes = {}) => ({ ...context, positions_degrees: [.1, 0, 0, 0, 0, 0], simulation_only: true, execution_ready: false, ...changes });
const flush = () => new Promise(resolve => setTimeout(resolve, 0));
it("czeka na anulowanie podglądu, wysyła jedno obliczenie i stosuje wspólny cel", async () => {
    let complete!: (v: unknown) => void;
    const request = vi.fn(() => new Promise(resolve => { complete = resolve; })), apply = vi.fn(() => true), error = vi.fn();
    const tcp = new PlanningTcpJog(apply, error, request);
    expect(tcp.start(context, "x", 1, "tool", 10, 0)).toBe(true);
    expect(tcp.start(context, "y", 1, "base", 10, 0)).toBe(false);
    tcp.pulse(50, Array(6).fill(0), () => true, false); expect(request).not.toHaveBeenCalled();
    tcp.pulse(100, Array(6).fill(0), () => true, true);
    tcp.pulse(150, Array(6).fill(0), () => true, true); expect(request).toHaveBeenCalledTimes(1);
    expect(request.mock.calls[0]).toEqual(["planning/tcp-jog", expect.objectContaining({ frame: "tool", axis: "x", velocity: 10, duration_s: .05 })]);
    complete(answer()); await flush(); expect(apply).toHaveBeenCalledWith([.1, 0, 0, 0, 0, 0]); expect(error).not.toHaveBeenCalled();
});
it.each(["release", "invalid"])("odrzuca spóźniony wynik: %s", async reason => {
    let complete!: (v: unknown) => void, valid = true;
    const apply = vi.fn(() => true), tcp = new PlanningTcpJog(apply, vi.fn(), () => new Promise(resolve => { complete = resolve; }));
    tcp.start(context, "rz", -1, "base", 5, 0); tcp.pulse(50, Array(6).fill(0), () => valid, true);
    if (reason === "release") tcp.stop(); else valid = false;
    complete(answer()); await flush(); expect(apply).not.toHaveBeenCalled();
});
it.each([{ execution_ready: true }, { settings_revision: "old" }, { positions_degrees: [91, 0, 0, 0, 0, 0] },
    { positions_degrees: [2, 0, 0, 0, 0, 0] }])("zatrzymuje niepoprawny wynik i zachowuje poprzedni cel", async changes => {
    const apply = vi.fn(() => true), error = vi.fn(), tcp = new PlanningTcpJog(apply, error, async () => answer(changes));
    tcp.start(context, "x", 1, "base", 10, 0); tcp.pulse(50, Array(6).fill(0), () => true, true); await flush();
    expect(tcp.held).toBeNull(); expect(apply).not.toHaveBeenCalled(); expect(error).toHaveBeenCalledOnce();
});
it("błąd IK i długa przerwa zatrzymują bez wznowienia", async () => {
    const request = vi.fn(async () => { throw Error("Brak IK"); }), error = vi.fn(), tcp = new PlanningTcpJog(() => true, error, request);
    tcp.start(context, "x", 1, "base", 10, 0); tcp.pulse(50, Array(6).fill(0), () => true, true); await flush();
    expect(error).toHaveBeenCalledWith("Brak IK"); expect(tcp.held).toBeNull();
    tcp.start(context, "x", 1, "base", 10, 100); tcp.pulse(251, Array(6).fill(0), () => true, true);
    expect(tcp.held).toBeNull(); expect(request).toHaveBeenCalledTimes(1);
});
it.each([10, 50, 100])("posuw %s mm/s nie jest ograniczany do 1 mm co 50 ms", async speed => {
    const request = vi.fn(async () => answer()), tcp = new PlanningTcpJog(() => true, vi.fn(), request);
    tcp.start(context, "x", 1, "base", speed, 0); tcp.pulse(50, Array(6).fill(0), () => true, true); await flush();
    expect(request).toHaveBeenCalledWith("planning/tcp-jog", expect.objectContaining({ velocity: speed, duration_s: .05 }));
});
it("dostosowuje czas do opóźnienia, bez kolejki i nadrabiania dłuższego niż 100 ms", async () => {
    const request = vi.fn(async () => answer()), tcp = new PlanningTcpJog(() => true, vi.fn(), request);
    tcp.start(context, "x", 1, "base", 100, 0); tcp.pulse(50, Array(6).fill(0), () => true, true); await flush();
    tcp.pulse(100, Array(6).fill(0), () => true, false);
    tcp.pulse(180, Array(6).fill(0), () => true, true); await flush();
    expect(request).toHaveBeenLastCalledWith("planning/tcp-jog", expect.objectContaining({ duration_s: .1 }));
});
it("zachowuje zgodność z backendem przed restartem", async () => {
    const request = vi.fn(async () => answer()), tcp = new PlanningTcpJog(() => true, vi.fn(), request);
    tcp.start({ ...context, tcp_jog_segmented: undefined }, "x", 1, "base", 100, 0);
    tcp.pulse(50, Array(6).fill(0), () => true, true); await flush();
    expect(request).toHaveBeenCalledWith("planning/tcp-jog", expect.objectContaining({ duration_s: .01 }));
});
