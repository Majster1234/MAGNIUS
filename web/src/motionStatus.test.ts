import { describeExecutionTuning as describeTuning } from "./motionStatus";
import { it as itTuning, expect as expectTuning } from "vitest";
itTuning("opisuje tylko zmienione testowe nastawy regulatora", () => {
    const base = { lookahead_time_sec: .085, velocity_ff_scale: 1, position_kp: 2.5, integral_gain: 2, integral_limit_rad: .0174533 };
    expectTuning(describeTuning(null)).toBeNull();
    expectTuning(describeTuning({ test: false, active: base, baseline: base })).toBeNull();
    const text = describeTuning({ test: true, active: { ...base, lookahead_time_sec: .12, position_kp: 4 }, baseline: base })!;
    expectTuning(text).toContain("wyprzedzenie celu 120 ms (plik: 85 ms)");
    expectTuning(text).toContain("Kp 4 (plik: 2.5)");
    expectTuning(text).not.toContain("FF");
});
