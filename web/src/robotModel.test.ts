import { expect, it } from "vitest";
import * as THREE from "three";
import { URDFRobot, URDFJoint, URDFLink, URDFVisual } from "urdf-loader/src/URDFClasses.js";
import { assemblyMatrix } from "./robotModel";
import { attachModel } from "./robotPreviewScene";

function model() {
    const robot = new URDFRobot(); robot.joints = {}; robot.links = {}; robot.visual = {}; robot.colliders = {};
    const offsets = [[0, 0, .1], [0, 0, .1], [0, 0, .25], [.22, 0, 0], [.08, 0, 0], [.06, 0, 0], [.08, 0, 0]];
    const axes = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0]];
    const base = new URDFLink(); base.name = "base_link"; robot.add(base);
    let parent: THREE.Object3D = base;
    offsets.forEach((offset, i) => {
        const joint = new URDFJoint(); joint.name = joint.urdfName = i < 6 ? `joint_${i + 1}` : "tool_fixed";
        joint.jointType = i < 6 ? "continuous" : "fixed"; joint.position.fromArray(offset);
        if (i < 6) joint.axis.fromArray(axes[i]);
        const link = new URDFLink(); link.name = i < 6 ? `link_${i + 1}` : "tool0";
        const visual = new URDFVisual(); visual.add(new THREE.Mesh(new THREE.BoxGeometry(.01, .01, .01))); link.add(visual);
        robot.joints[joint.name] = joint; parent.add(joint); joint.add(link); parent = link;
    });
    robot.updateMatrixWorld(true); return robot;
}

it("przekształcenie złożenia: Y w górę → Z, potem przesunięcie [mm] i obrót [°]", () => {
    const yUp = assemblyMatrix({up: "y", transform: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0}});
    expect(new THREE.Vector3(0, 1, 0).applyMatrix4(yUp).toArray().map(v => +v.toFixed(9))).toEqual([0, 0, 1]);
    const moved = assemblyMatrix({up: "z", transform: {x: 100, y: -20, z: 5, roll: 0, pitch: 0, yaw: 90}});
    expect(new THREE.Vector3(.01, 0, 0).applyMatrix4(moved).toArray().map(v => +v.toFixed(9))).toEqual([.1, -.01, .005]);
});

it("siatki członów: w pozycji zerowej leżą jak w złożeniu, poruszają się z członem, bryły zastępcze ukryte", () => {
    const robot = model(), matrix = assemblyMatrix({up: "z", transform: {x: 10, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0}});
    const geometry = new THREE.BoxGeometry(.02, .02, .2).translate(0, 0, .3);   // „ramię” w układzie złożenia
    expect(attachModel(robot, {items: [{link: "link_2", id: "a", name: "ramię", geometry}, {link: "base", id: "b", name: "podstawa", geometry}], matrix})).toBe(2);
    robot.updateMatrixWorld(true);
    const mesh = robot.getObjectByName("model_link_2") as THREE.Mesh;
    expect(mesh.matrixWorld.equals(matrix) || mesh.matrixWorld.elements.every((v, i) => Math.abs(v - matrix.elements[i]) < 1e-12)).toBe(true);
    const link2 = robot.joints.joint_2.children.find(child => (child as URDFLink).isURDFLink)!;
    expect(link2.children.filter(child => (child as URDFVisual).isURDFVisual).every(child => !child.visible)).toBe(true);
    // Obrót J2 o 90° wokół Y przez początek przegubu (0, 0, 0,2): punkt siatki obraca się razem z członem.
    const point = new THREE.Vector3(0, 0, .4), before = point.clone().applyMatrix4(mesh.matrixWorld);
    robot.setJointValue("joint_2", Math.PI / 2); robot.updateMatrixWorld(true);
    const after = point.clone().applyMatrix4(mesh.matrixWorld);
    const pivot = new THREE.Vector3(0, 0, .2);
    const expected = before.clone().sub(pivot).applyAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2).add(pivot);
    expect(after.distanceTo(expected)).toBeLessThan(1e-9);
});
