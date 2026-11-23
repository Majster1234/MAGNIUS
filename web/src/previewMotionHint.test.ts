import { expect, it } from "vitest";
import { Vector3 } from "three";
import { URDFRobot, URDFJoint } from "urdf-loader/src/URDFClasses.js";
import { hintGeometry, hintLabel } from "./previewMotionHint";

// Łańcuch testowy jak w previewScene.test.ts (preview_6dof.example.urdf).
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
const tcp = (robot: URDFRobot) => { robot.updateMatrixWorld(true); return robot.joints.tool_fixed.getWorldPosition(new Vector3()); };

it("JOG osi: strzałka obrotu odpowiada ruchowi modelu przy dodatnim/ujemnym kierunku", () => {
    const robot = model();
    robot.setJointValue("joint_1", .4); robot.setJointValue("joint_2", .3); robot.setJointValue("joint_3", -.5);
    for (const axis of [1, 2, 3, 5]) for (const direction of [1, -1]) {
        const before = tcp(robot), geometry = hintGeometry(robot, { kind: "joint", axis, direction })!;
        const name = `joint_${axis}`, angle = robot.joints[name].angle as number;
        robot.setJointValue(name, angle + direction * 1e-3);
        const moved = tcp(robot).sub(before);
        robot.setJointValue(name, angle);
        // Ruch punktu przy obrocie wokół osi: ω × r; strzałka (oś ze znakiem) musi dawać ten sam kierunek.
        const predicted = geometry.axis.clone().cross(before.clone().sub(geometry.origin));
        expect(geometry.rotation).toBe(true);
        expect(moved.normalize().dot(predicted.normalize())).toBeGreaterThan(.999);
    }
    // Oś J2 po obrocie J1 o 90°: z (0, 1, 0) na (−1, 0, 0).
    robot.setJointValue("joint_1", Math.PI / 2);
    expect(hintGeometry(robot, { kind: "joint", axis: 2, direction: 1 })!.axis.toArray().map(v => +v.toFixed(6))).toEqual([-1, 0, 0]);
});

it("JOG TCP: baza = osie korzenia modelu, narzędzie = orientacja TCP; obrót wokół osi przez TCP", () => {
    const robot = model();
    robot.setJointValue("joint_1", Math.PI / 2); robot.setJointValue("joint_5", .7);
    const base = hintGeometry(robot, { kind: "tcp", axis: "x", direction: -1, frame: "base" })!;
    expect(base.rotation).toBe(false);
    expect(base.axis.toArray().map(v => +v.toFixed(6))).toEqual([-1, 0, 0]);
    expect(base.origin.distanceTo(tcp(robot))).toBeLessThan(1e-9);
    const tool = hintGeometry(robot, { kind: "tcp", axis: "x", direction: 1, frame: "tool" })!;
    const expected = new Vector3(1, 0, 0).applyQuaternion(robot.joints.tool_fixed.getWorldQuaternion(tool.frame.clone()));
    expect(tool.axis.distanceTo(expected)).toBeLessThan(1e-9);
    expect(tool.axis.distanceTo(new Vector3(1, 0, 0))).toBeGreaterThan(.5);
    const rotation = hintGeometry(robot, { kind: "tcp", axis: "rz", direction: 1, frame: "base" })!;
    expect(rotation.rotation).toBe(true);
    expect(rotation.axis.toArray().map(v => +v.toFixed(6))).toEqual([0, 0, 1]);
});

it("etykiety i odrzucenie nieprawidłowych danych", () => {
    expect(hintLabel({ kind: "joint", axis: 3, direction: -1 })).toBe("J3 −");
    expect(hintLabel({ kind: "tcp", axis: "ry", direction: 1, frame: "tool" })).toBe("TCP RY+ · narzędzie");
    expect(hintLabel({ kind: "tcp", axis: "z", direction: -1, frame: "base" })).toBe("TCP Z− · baza");
    const robot = model();
    expect(hintGeometry(robot, { kind: "joint", axis: 7, direction: 1 })).toBeNull();
    expect(hintGeometry(robot, { kind: "joint", axis: 1, direction: 0 })).toBeNull();
});
