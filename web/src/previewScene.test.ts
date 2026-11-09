import { expect, it } from "vitest";
import { Scene, Vector3 } from "three";
import { URDFRobot, URDFJoint } from "urdf-loader/src/URDFClasses.js";
import { classifySingularity, robotSingularity, singularRatio } from "./previewSingularity";
import { createMotionTrail } from "./previewMotionTrail";
import type { CartesianPlan } from "./cartesianTrajectory";

// Łańcuch testowy odpowiada zatwierdzonemu preview_6dof.example.urdf.
function model() {
    const robot = new URDFRobot(); robot.joints = {}; robot.links = {}; robot.visual = {}; robot.colliders = {};
    const offsets = [[0, 0, .1], [0, 0, .1], [0, 0, .25], [.22, 0, 0], [.08, 0, 0], [.06, 0, 0], [.08, 0, 0]];
    const axes = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0]];
    let parent = robot as import("three").Object3D;
    offsets.forEach((offset, i) => {
        const joint = new URDFJoint(); joint.name = joint.urdfName = i < 6 ? `joint_${i + 1}` : "tool_fixed";
        joint.jointType = i < 6 ? "continuous" : "fixed"; joint.position.fromArray(offset);
        if (i < 6) joint.axis.fromArray(axes[i]);
        robot.joints[joint.name] = joint; parent.add(joint); parent = joint;
    });
    robot.updateMatrixWorld(true); return robot;
}

it("wartości singularne rozróżniają utratę rzędu i zachowują skalowanie/obrót", () => {
    expect(singularRatio([[1, 0, 0], [0, 1, 0], [0, 0, 1]])).toBeCloseTo(1);
    expect(singularRatio([[1, 0, 0], [0, 1, 0], [1, 0, 0]])).toBe(0);
    const columns = [[3, 0, 0], [0, 2, 0], [0, 0, .3]];
    expect(singularRatio(columns)).toBeCloseTo(.1);
    const rotated = columns.map(column => new Vector3(...column).applyAxisAngle(new Vector3(1, 2, 3).normalize(), .73).multiplyScalar(7).toArray());
    expect(singularRatio(rotated)).toBeCloseTo(.1);
});
it("nadgarstek odpowiada progom C++, pełny Jacobian wykrywa także osobliwość ramienia", () => {
    const robot = model();
    expect(robotSingularity(robot)?.level).toBe("critical");
    for (const [angle, level] of [[4, "critical"], [15, "warning"], [45, "clear"]] as const) {
        robot.setJointValue("joint_5", angle * Math.PI / 180); robot.updateMatrixWorld(true);
        const result = robotSingularity(robot)!;
        expect(result.wrist).toBeCloseTo(Math.tan(angle * Math.PI / 360), 6);
        expect(classifySingularity(result.wrist, 1).level).toBe(level);
    }
    expect(classifySingularity(1, .001)).toMatchObject({ level: "critical", source: "arm" });
    const before = robotSingularity(robot)!;
    robot.position.set(2, -1, 3); robot.rotateZ(.7); robot.updateMatrixWorld(true);
    expect(robotSingularity(robot)!.arm).toBeCloseTo(before.arm, 6);
    expect(robotSingularity(robot)!.wrist).toBeCloseTo(before.wrist, 6);
});
it("ślad rośnie, pozostaje przez powrót na start i wymienia zasoby dla nowego planu", () => {
    const robot = model(), scene = new Scene(), trail = createMotionTrail(scene, robot);
    const plan: CartesianPlan = { id: "p1", model_id: "model", configuration_revision: "r1", settings_revision: "s",
        start_positions: Array(6).fill(0), locked: Array(6).fill(false), execution_ready: false, execution_message: "",
        mks_compatible: false, mks_message: "", mks_samples: [],
        trajectory: { joint_names: Array.from({ length: 6 }, (_, i) => `joint_${i + 1}`), points: [0, 1].map(t => ({
            positions: [t * .5, 0, 0, 0, 0, 0], velocities: Array(6).fill(0), accelerations: Array(6).fill(0),
            time_from_start: { sec: t, nanosec: 0 },
        })) } };
    trail.update({ plan, seconds: .5 }, true); const half = trail.snapshot();
    expect(half.visible).toBe(true); expect(half.points).toBeGreaterThan(2);
    trail.update({ plan, seconds: 1 }, true); const full = trail.snapshot();
    expect(full.points).toBe(full.capacity); expect(full.points).toBeGreaterThan(half.points);
    trail.update({ plan, seconds: 0 }, true); expect(trail.snapshot()).toEqual(full);
    const line = scene.children[0] as import("three").Line;
    const geometry = line.geometry, vertices = geometry.getAttribute("position");
    expect(vertices.count).toBe(61);
    expect(vertices.getX(0)).not.toBe(vertices.getX(60));
    expect(robot.joints.joint_1.angle).toBe(0); // FK śladu nie zmienia modelu.
    let disposed = 0; geometry.addEventListener("dispose", () => disposed++);
    trail.update({ plan: { ...plan, id: "p2" }, seconds: 0 }, true);
    expect(disposed).toBe(1); expect(trail.snapshot().points).toBe(1);
    trail.update(null, false); expect(trail.snapshot().visible).toBe(false);
});
