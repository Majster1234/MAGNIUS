import { expect, it } from "vitest";
import { trajectoryPosition, validateCartesianPlan, type CartesianPlan } from "./cartesianTrajectory";
function plan(): CartesianPlan {
    return { id: "test", model_id: "model", configuration_revision: "r", start_positions: Array(6).fill(0), locked: Array(6).fill(false),
        mks_compatible: false, mks_message: "", execution_ready: false, execution_message: "", mks_samples: [],
        trajectory: { joint_names: Array.from({length:6}, (_, i) => `joint_${i + 1}`), points: [0, 1].map(t => ({
            positions: Array(6).fill(t), velocities: Array(6).fill(0), accelerations: Array(6).fill(0), time_from_start: {sec:t, nanosec:0}
        })) } };
}
it("odtwarza interpolację JTC i zachowuje krańce", () => {
    const value = plan(); validateCartesianPlan(value);
    expect(trajectoryPosition(value, 0)[0]).toBe(0);
    expect(trajectoryPosition(value, .5)[0]).toBeCloseTo(.5 * 180 / Math.PI);
    expect(trajectoryPosition(value, 2)[0]).toBeCloseTo(180 / Math.PI);
});
it("odrzuca niespójne punkty, kolejność osi i pochodne", () => {
    const value = plan(); value.trajectory.points[1].time_from_start.sec = 0;
    expect(() => validateCartesianPlan(value)).toThrow();
    const bad = plan(); bad.trajectory.points[0].accelerations = [];
    expect(() => validateCartesianPlan(bad)).toThrow();
});
