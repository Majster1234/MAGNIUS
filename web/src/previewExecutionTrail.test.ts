import { expect, it } from "vitest";
import { Scene, Vector3, type LineSegments } from "three";
import { URDFRobot, URDFJoint } from "urdf-loader/src/URDFClasses.js";
import { createExecutionTrail, EXECUTION_TRAIL_CAPACITY, type ExecutionFrame } from "./previewExecutionTrail";

function model() {
    const robot = new URDFRobot(); robot.joints = {}; robot.links = {}; robot.visual = {}; robot.colliders = {};
    let parent = robot as import("three").Object3D;
    for (let i = 0; i < 7; i++) {
        const joint = new URDFJoint(); joint.name = i < 6 ? `joint_${i + 1}` : "tool_fixed";
        joint.urdfName = joint.name; joint.jointType = i < 6 ? "continuous" : "fixed";
        joint.position.set(i ? .1 : 0, 0, .1); if (i < 6) joint.axis.set(0, 0, 1);
        robot.joints[joint.name] = joint; parent.add(joint); parent = joint;
    }
    robot.updateMatrixWorld(true); return robot;
}
function sample(time: number, degrees = 0): ExecutionFrame {
    return { revision: "r", usable: true, receivedAt: time, sampleKey: String(time),
        state: { fresh: true, busy: false, fault: false, active_axis: 0, max_rpm: 30, operation: "idle", message: "", configuration_revision: "r",
            axes: Array.from({ length: 6 }, (_, i) => ({ axis: i + 1, available: true, prepared: true, enable_known: true, enabled: true,
                position_known: true, position_degrees: 999, position_age_seconds: 0, speed_known: true, speed_rpm: 0,
                can_jog: false, can_zero: false, motion: "", joint_configured: true, calibration_valid: true,
                joint_position_known: true, joint_position_degrees: i ? 0 : degrees })) },
        execution: { receivedAt: time, status: { program_id: "p1", execution_key: "k1", state: "running", busy: true,
            mode: "streamed", message: "", progress: 0, duration: 1, start_positions: Array(6).fill(.1),
            target_positions: Array(6).fill(.2), participating: Array(6).fill(true), tcp_verified: true } } };
}
function fixture(capacity?: number) {
    const robot = model(), scene = new Scene(), trail = createExecutionTrail(scene, robot, capacity);
    return { robot, scene, trail, line: scene.children[0] as LineSegments };
}
it("złoty ślad liczy FK surowych przegubów zamiast wału lub wygładzonego modelu", () => {
    const { robot, trail } = fixture(); robot.setJointValue("joint_1", 1.5);
    trail.update(sample(0, 10), 0); trail.update(sample(50, 20), 50);
    const expected = model(); expected.setJointValue("joint_1", 20 * Math.PI / 180); expected.updateMatrixWorld(true);
    const position = expected.joints.tool_fixed.getWorldPosition(new Vector3()).toArray();
    trail.snapshot().lastPoint!.forEach((value, i) => expect(value).toBeCloseTo(position[i], 6));
    expect(robot.joints.joint_1.angle).toBe(1.5);
    expect(trail.snapshot()).toMatchObject({ points: 2, edges: 1, segments: 1, partial: false, color: "#ffc857", capacity: EXECUTION_TRAIL_CAPACITY });
});
it("FK złotego śladu nie obcina odczytu enkodera do limitu URDF", () => {
    const robot = model(); robot.joints.joint_1.jointType = "revolute";
    robot.joints.joint_1.limit.lower = -.1; robot.joints.joint_1.limit.upper = .1;
    const trail = createExecutionTrail(new Scene(), robot); trail.update(sample(0, 20), 0);
    const expected = model(); expected.setJointValue("joint_1", 20 * Math.PI / 180); expected.updateMatrixWorld(true);
    const position = expected.joints.tool_fixed.getWorldPosition(new Vector3()).toArray();
    trail.snapshot().lastPoint!.forEach((value, i) => expect(value).toBeCloseTo(position[i], 6));
    expect(robot.joints.joint_1.ignoreLimits).toBe(false);
    expect(robot.joints.joint_1.angle).toBe(0);
});
it("ślad nie startuje podczas przygotowania ani JOG i zachowuje historię po sukcesie/STOP", () => {
    const { trail } = fixture();
    const ready = sample(0); ready.execution!.status.state = "ready"; ready.execution!.status.busy = false;
    trail.update(ready, 0); expect(trail.snapshot().points).toBe(0);
    const jog = sample(50, 10); jog.state!.active_axis = 1; jog.state!.operation = "jog";
    trail.update(jog, 50); expect(trail.snapshot().points).toBe(0);
    trail.update(sample(100), 100); trail.update(sample(150, 10), 150);
    const before = trail.snapshot();
    for (const state of ["succeeded", "failed"]) {
        const terminal = sample(200, 30); terminal.execution!.status.state = state; terminal.execution!.status.busy = false;
        trail.update(terminal, 200); expect(trail.snapshot()).toEqual(before);
    }
});
it("kolejne programy i klucze wykonania tworzą odrębne złote odcinki", () => {
    const { trail, line } = fixture(); trail.update(sample(0), 0); trail.update(sample(50, 10), 50);
    const second = sample(100, 90); second.execution!.status.execution_key = "k2"; trail.update(second, 100);
    const next = sample(150, 100); next.execution!.status.execution_key = "k2"; trail.update(next, 150);
    const third = sample(200, 150); third.execution!.status.program_id = "p2"; trail.update(third, 200);
    expect(trail.snapshot()).toMatchObject({ points: 5, edges: 2, segments: 3 });
    const vertices = line.geometry.getAttribute("position");
    for (const slot of [2, 4]) for (const coordinate of ["getX", "getY", "getZ"] as const)
        expect(vertices[coordinate](slot * 2)).toBe(vertices[coordinate](slot * 2 + 1));
});
it("brakująca nieuczestnicząca oś bierze kąt startowy i oznacza tor modelu", () => {
    const { trail } = fixture(); const frame = sample(0);
    frame.state!.axes[3].available = false; frame.execution!.status.participating[3] = false;
    frame.state!.axes[3].joint_position_degrees = 12345; trail.update(frame, 0);
    const expected = model(); expected.setJointValue("joint_4", .1); expected.updateMatrixWorld(true);
    const position = expected.joints.tool_fixed.getWorldPosition(new Vector3()).toArray();
    trail.snapshot().lastPoint!.forEach((value, i) => expect(value).toBeCloseTo(position[i], 6));
    expect(trail.snapshot().partial).toBe(true);
});
it("brak/utrata świeżości uczestnika i statusu przerywa linię bez mostu po wznowieniu", () => {
    for (const kind of ["participant", "age", "status", "status_age", "revision", "receipt", "fault"]) {
        const { trail } = fixture(); trail.update(sample(0), 0); trail.update(sample(50, 10), 50);
        const invalid = sample(100, 20);
        if (kind === "participant") invalid.state!.axes[0].joint_position_known = false;
        if (kind === "age") invalid.state!.axes[0].position_age_seconds = .25;
        if (kind === "status") invalid.execution = null;
        if (kind === "status_age") invalid.execution!.receivedAt = -650;
        if (kind === "revision") invalid.state!.configuration_revision = "other";
        if (kind === "receipt") invalid.receivedAt = -150;
        if (kind === "fault") invalid.state!.fault = true;
        trail.update(invalid, 100); expect(trail.snapshot().points, kind).toBe(2);
        trail.update(sample(150, 90), 150); expect(trail.snapshot(), kind).toMatchObject({ points: 3, edges: 1, segments: 2 });
    }
});
it("powtórzenia i próbki powyżej20 Hz nie powiększają bufora; luka rozdziela tor", () => {
    const { trail } = fixture(); trail.update(sample(0), 0);
    trail.update(sample(25, 10), 25); expect(trail.snapshot().points).toBe(1);
    trail.update(sample(50), 50); expect(trail.snapshot().points).toBe(1);
    trail.update(sample(100, 10), 100); expect(trail.snapshot().points).toBe(2);
    const duplicate = sample(100, 20); duplicate.receivedAt = 150; duplicate.execution!.receivedAt = 150;
    trail.update(duplicate, 150); expect(trail.snapshot().points).toBe(2);
    trail.update(sample(500, 90), 500); expect(trail.snapshot()).toMatchObject({ points: 3, edges: 1, segments: 2 });
});
it("czyszczenie podczas PTP czeka na nową próbkę i nie łączy starej historii", () => {
    const { trail } = fixture(); trail.update(sample(0), 0); trail.update(sample(50, 10), 50);
    trail.clear(); expect(trail.snapshot()).toMatchObject({ points: 0, visible: false, partial: false });
    trail.update(sample(50, 10), 75); expect(trail.snapshot().points).toBe(0);
    trail.update(sample(100, 20), 100); expect(trail.snapshot()).toMatchObject({ points: 1, edges: 0 });
    trail.update(sample(150, 30), 150); expect(trail.snapshot()).toMatchObject({ points: 2, edges: 1, segments: 1 });
});
it("bufor usuwa najstarsze punkty bez łączenia przerw i czyści metadane częściowego toru", () => {
    const { trail, line } = fixture(4);
    const partial = sample(0); partial.execution!.status.participating[3] = false; partial.state!.axes[3].available = false;
    trail.update(partial, 0);
    for (let i = 1; i <= 20; i++) {
        const frame = sample(i * 50, i * 5); frame.execution!.status.execution_key = i < 10 ? "k1" : "k2";
        trail.update(frame, i * 50);
    }
    expect(trail.snapshot()).toMatchObject({ points: 4, edges: 3, segments: 1, partial: false, dropped: 17 });
    expect(line.geometry.getAttribute("position").count).toBe(8);
    expect(line.geometry.drawRange.count).toBe(8);
    const vertices = line.geometry.getAttribute("position"), retained = [17, 18, 19, 20].map(i => {
        const reference = model(); reference.setJointValue("joint_1", i * 5 * Math.PI / 180); reference.updateMatrixWorld(true);
        return reference.joints.tool_fixed.getWorldPosition(new Vector3());
    });
    let renderedEdges = 0;
    for (let i = 0; i < 8; i += 2) {
        const a = new Vector3().fromBufferAttribute(vertices, i), b = new Vector3().fromBufferAttribute(vertices, i + 1);
        if (a.distanceTo(b) < 1e-7) continue;
        expect(retained.slice(0, -1).some((point, j) => a.distanceTo(point) < 1e-6 && b.distanceTo(retained[j + 1]) < 1e-6)).toBe(true);
        renderedEdges++;
    }
    expect(renderedEdges).toBe(3);
    expect(() => createExecutionTrail(new Scene(), model(), 1)).toThrow();
});
