import { Quaternion, Vector3 } from "three";
import type { URDFRobot } from "urdf-loader";

export type Singularity = { level: "clear" | "warning" | "critical"; wrist: number; arm: number; source: "wrist" | "arm" };

// Stosunek wartości singularnych przez wartości własne J·Jᵀ. Macierze do 6×6.
export function singularRatio(columns: number[][]): number {
    const size = columns[0]?.length ?? 0;
    if (!size || columns.some(column => column.length !== size || !column.every(Number.isFinite))) return 0;
    const matrix = Array.from({ length: size }, (_, i) => Array.from({ length: size }, (_, j) =>
        columns.reduce((sum, column) => sum + column[i] * column[j], 0)));
    for (let iteration = 0; iteration < 256; iteration++) {
        let p = 0, q = 0, largest = 0;
        for (let i = 0; i < size; i++) for (let j = i + 1; j < size; j++) {
            if (Math.abs(matrix[i][j]) > largest) { largest = Math.abs(matrix[i][j]); p = i; q = j; }
        }
        if (largest < 1e-13) break;
        const angle = .5 * Math.atan2(2 * matrix[p][q], matrix[q][q] - matrix[p][p]);
        const c = Math.cos(angle), s = Math.sin(angle), pp = matrix[p][p], qq = matrix[q][q], pq = matrix[p][q];
        for (let k = 0; k < size; k++) if (k !== p && k !== q) {
            const kp = matrix[k][p], kq = matrix[k][q];
            matrix[k][p] = matrix[p][k] = c * kp - s * kq;
            matrix[k][q] = matrix[q][k] = s * kp + c * kq;
        }
        matrix[p][p] = c*c*pp - 2*s*c*pq + s*s*qq;
        matrix[q][q] = s*s*pp + 2*s*c*pq + c*c*qq;
        matrix[p][q] = matrix[q][p] = 0;
    }
    const values = matrix.map((row, i) => Math.max(0, row[i]));
    const max = Math.max(...values);
    return max > 1e-12 ? Math.sqrt(Math.min(...values) / max) : 0;
}

export function classifySingularity(wrist: number, arm: number): Singularity {
    // Nadgarstek: te same progi co C++ (tan 2,5° / tan 15°).
    const wristLevel = wrist <= .04366094290851206 ? 2 : wrist <= .2679491924311227 ? 1 : 0;
    // Pełny Jacobian: bezwymiarowe progi ostrzeżenia podglądu.
    const armLevel = arm <= .005 ? 2 : arm <= .02 ? 1 : 0;
    const level = Math.max(wristLevel, armLevel);
    return { level: level === 2 ? "critical" : level === 1 ? "warning" : "clear", wrist, arm,
        source: wristLevel >= armLevel ? "wrist" : "arm" };
}

export function robotSingularity(robot: URDFRobot): Singularity | null {
    const tool = robot.joints.tool_fixed;
    if (!tool) return null;
    const tcp = tool.getWorldPosition(new Vector3());
    const origins: Vector3[] = [], directions: Vector3[] = [];
    for (let i = 1; i <= 6; i++) {
        const joint = robot.joints[`joint_${i}`];
        if (!joint || !["revolute", "continuous"].includes(joint.jointType)) return null;
        origins.push(joint.getWorldPosition(new Vector3()));
        directions.push(joint.axis.clone().applyQuaternion(joint.getWorldQuaternion(new Quaternion())).normalize());
    }
    const length = Math.max(...origins.map(origin => origin.distanceTo(tcp)));
    if (length < 1e-9) return null;
    const jacobian = directions.map((axis, i) => [
        ...axis.clone().cross(tcp.clone().sub(origins[i])).divideScalar(length).toArray(), ...axis.toArray(),
    ]);
    return classifySingularity(singularRatio(directions.slice(3).map(axis => axis.toArray())), singularRatio(jacobian));
}
