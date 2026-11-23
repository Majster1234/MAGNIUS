import type { Axis } from "./manualTypes";

export type LimitIndicator = {
    side: "lower" | "upper";
    level: "warning" | "danger" | "exceeded";
    limitDegrees: number;
    distanceDegrees: number;
    critical: boolean;
    fresh: boolean;
};
type LimitProfile = Pick<Axis, "joint_configured" | "calibration_valid" | "joint_lower_degrees" | "joint_upper_degrees">;

// Odległość dotyczy bieżącego odczytu, nie opóźnionej interpolacji modelu.
// Nie zawijamy kątów: 361° przy limicie 360° oznacza przekroczenie.
export function previewLimits(axis: LimitProfile | undefined, position: number | null, fresh: boolean): LimitIndicator[] {
    if (!axis || position === null || !Number.isFinite(position) || (axis.joint_configured && axis.calibration_valid === false)) return [];
    const lower = axis.joint_configured ? axis.joint_lower_degrees : -3600;
    const upper = axis.joint_configured ? axis.joint_upper_degrees : 3600;
    if (typeof lower !== "number" || typeof upper !== "number" || !Number.isFinite(lower) || !Number.isFinite(upper) || lower >= upper) return [];
    const result: LimitIndicator[] = [];
    for (const side of ["lower", "upper"] as const) {
        const limit = side === "lower" ? lower : upper;
        const distance = side === "lower" ? position - lower : upper - position;
        if (distance > 15) continue;
        result.push({ side, limitDegrees: limit, distanceDegrees: distance, fresh,
            level: distance < 0 ? "exceeded" : distance <= 5 ? "danger" : "warning", critical: distance <= 1 });
    }
    return result;
}
