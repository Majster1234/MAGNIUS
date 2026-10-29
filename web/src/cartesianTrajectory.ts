export type TrajectoryPoint = { positions: number[]; velocities: number[]; accelerations: number[]; time_from_start: { sec: number; nanosec: number } };
export type CartesianPlan = {
    id: string; model_id: string; configuration_revision: string; start_positions: number[]; locked: boolean[];
    trajectory: { joint_names: string[]; points: TrajectoryPoint[] };
    mks_compatible: boolean; mks_message: string; execution_ready: boolean; execution_message: string;
    mks_samples: { time_from_start: number; position_ticks: number[]; speed_rpm: number[]; acceleration_code: number[]; payloads: number[] }[];
    simulation_only?: boolean; simulated_start?: boolean[]; target_positions?: number[];
    motion_type?: "PTP" | "LIN" | "CARTESIAN"; settings_revision?: string;
};
export const pointTime = (point: TrajectoryPoint) => point.time_from_start.sec + point.time_from_start.nanosec / 1e9;
export function validateCartesianPlan(plan: CartesianPlan) {
    if (plan.trajectory.joint_names.join() !== Array.from({ length: 6 }, (_, i) => `joint_${i + 1}`).join() || plan.trajectory.points.length < 2 || plan.trajectory.points.length > 10000) throw Error("Nieprawidłowa trajektoria planu");
    let previous = -1;
    for (const point of plan.trajectory.points) {
        const time = pointTime(point);
        if (!Number.isFinite(time) || time <= previous || time > 200 || point.time_from_start.nanosec < 0 || point.time_from_start.nanosec >= 1e9 ||
            [point.positions, point.velocities, point.accelerations].some(values => values.length !== 6 || !values.every(Number.isFinite))) throw Error("Nieprawidłowe punkty trajektorii");
        previous = time;
    }
}
// Ten sam wielomian piątego stopnia co interpolacja pozycji/prędkości/przyspieszenia JTC.
export function trajectoryPosition(plan: CartesianPlan, seconds: number): number[] {
    const points = plan.trajectory.points;
    if (seconds <= pointTime(points[0])) return points[0].positions.map(value => value * 180 / Math.PI);
    if (seconds >= pointTime(points[points.length - 1])) return points[points.length - 1].positions.map(value => value * 180 / Math.PI);
    let right = 1; while (pointTime(points[right]) < seconds) right++;
    const left = points[right - 1], next = points[right], dt = pointTime(next) - pointTime(left), s = (seconds - pointTime(left)) / dt;
    return left.positions.map((q, i) => {
        const c1 = left.velocities[i] * dt, c2 = left.accelerations[i] * dt * dt / 2, d = next.positions[i] - q, v = next.velocities[i] * dt, a = next.accelerations[i] * dt * dt;
        const c3 = 10*d - 6*c1 - 3*c2 - 4*v + a/2, c4 = -15*d + 8*c1 + 3*c2 + 7*v - a, c5 = 6*d - 3*c1 - c2 - 3*v + a/2;
        return (q + s*(c1 + s*(c2 + s*(c3 + s*(c4 + s*c5))))) * 180 / Math.PI;
    });
}
