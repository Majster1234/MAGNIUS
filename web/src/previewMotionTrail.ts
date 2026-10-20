import * as THREE from "three";
import type { URDFRobot } from "urdf-loader";
import { pointTime, trajectoryPosition, type CartesianPlan } from "./cartesianTrajectory";

export type MotionTrace = { plan: CartesianPlan; seconds: number };

export function createMotionTrail(scene: THREE.Scene, robot: URDFRobot) {
    const geometry = new THREE.BufferGeometry();
    const material = new THREE.LineBasicMaterial({ color: 0xffbb65, transparent: true, opacity: .95, depthWrite: false, depthTest: false });
    const line = new THREE.Line(geometry, material); line.frustumCulled = false; line.visible = false;
    line.renderOrder = 3; scene.add(line);
    // Osobny FK nie zmienia aktualnej pozy wyświetlanego bliźniaka.
    const reference = robot.clone(true) as URDFRobot;
    let key = "", count = 0, duration = 0, maximum = 0, visibleCount = 0;
    function clear() {
        const changed = line.visible || !!key;
        line.visible = false; geometry.setDrawRange(0, 0); key = ""; count = visibleCount = maximum = 0;
        return changed;
    }
    return {
        update(trace: MotionTrace | null | undefined, planning: boolean) {
            if (!planning || !trace || !reference.joints.tool_fixed) return clear();
            const next = `${trace.plan.model_id}/${trace.plan.configuration_revision}/${trace.plan.settings_revision}/${trace.plan.id}`;
            let changed = false;
            if (next !== key) {
                key = next; maximum = 0; visibleCount = 0;
                duration = pointTime(trace.plan.trajectory.points.at(-1)!);
                count = Math.min(12001, Math.max(2, Math.ceil(duration * 60) + 1));
                const points = new Float32Array(count * 3), position = new THREE.Vector3();
                for (let i = 0; i < count; i++) {
                    trajectoryPosition(trace.plan, duration * i / (count - 1)).forEach((q, j) => reference.setJointValue(`joint_${j + 1}`, q * Math.PI / 180));
                    reference.updateMatrixWorld(true);
                    reference.joints.tool_fixed?.getWorldPosition(position);
                    position.toArray(points, i * 3);
                }
                geometry.dispose();
                geometry.setAttribute("position", new THREE.BufferAttribute(points, 3));
                changed = true;
            }
            maximum = Math.max(maximum, Math.min(duration, Math.max(0, trace.seconds)));
            const nextCount = maximum >= duration ? count : Math.floor(maximum / duration * (count - 1)) + 1;
            if (nextCount !== visibleCount) { visibleCount = nextCount; changed = true; }
            geometry.setDrawRange(0, visibleCount); line.visible = visibleCount > 1;
            return changed;
        },
        snapshot() { return { id: key, points: visibleCount, capacity: count, seconds: maximum, visible: line.visible }; },
        // Geometrię i materiał zwalnia właściciel sceny.
    };
}
