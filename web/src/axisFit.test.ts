import { expect, it } from "vitest";
import * as THREE from "three";
import { fitCylinder, symmetricEigen } from "./axisFit";
import { applyJointAxis, applyTcpPoint, geometryChecks, worldAxis, worldFrames } from "./frameEdit";
import { emptyDocument, valueOf, type Values } from "./configurationData";

// Walec jak z STL (bez indeksów), osadzony w dowolnej pozie: oś Y walca → `axis`, środek w `center`.
function cylinder(radius: number, height: number, segments: number, axis: THREE.Vector3, center: THREE.Vector3) {
    const geometry = new THREE.CylinderGeometry(radius, radius, height, segments, 4).toNonIndexed();
    geometry.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis.clone().normalize()));
    geometry.translate(center.x, center.y, center.z);
    return geometry;
}
const sideFace = (geometry: THREE.BufferGeometry) => 10;  // ściany boczne są na początku bufora CylinderGeometry

it("wartości własne macierzy symetrycznej", () => {
    const eigen = symmetricEigen([[2, 1, 0], [1, 2, 0], [0, 0, 5]]);
    expect(eigen.map(e => +e.value.toFixed(9))).toEqual([1, 3, 5]);
    expect(Math.abs(eigen[0].vector.dot(new THREE.Vector3(1, -1, 0).normalize()))).toBeCloseTo(1, 9);
});

it("oś walca: kierunek, punkt na osi, promień i zasięg kątowy — także po przekształceniu złożenia", () => {
    const axis = new THREE.Vector3(.3, .2, 1).normalize(), center = new THREE.Vector3(.12, -.05, .3);
    const geometry = cylinder(.02, .08, 64, axis, center);
    const fit = fitCylinder(geometry, sideFace(geometry));
    expect(Math.abs(fit.direction.dot(axis))).toBeCloseTo(1, 6);
    const offset = fit.point.clone().sub(center);
    expect(offset.clone().addScaledVector(axis, -offset.dot(axis)).length()).toBeLessThan(1e-6);   // na osi
    expect(Math.abs(offset.dot(axis))).toBeLessThan(.002);                                           // przy środku walca
    expect(fit.radius).toBeCloseTo(.02 * Math.cos(Math.PI / 64), 5);
    expect(fit.coverageDeg).toBeGreaterThan(350);
    // Ten sam walec w układzie złożenia przekształconym do bazy (np. Y w górę → Z w górę).
    const matrix = new THREE.Matrix4().makeRotationX(Math.PI / 2).setPosition(.01, .02, .03);
    const moved = fitCylinder(geometry, sideFace(geometry), matrix);
    expect(Math.abs(moved.direction.dot(axis.clone().applyMatrix4(new THREE.Matrix4().makeRotationX(Math.PI / 2))))).toBeCloseTo(1, 6);
    expect(moved.point.distanceTo(fit.point.clone().applyMatrix4(matrix))).toBeLessThan(1e-9);
});

it("płaska powierzchnia nie wyznacza osi", () => {
    const geometry = cylinder(.02, .08, 64, new THREE.Vector3(0, 0, 1), new THREE.Vector3());
    const position = geometry.getAttribute("position");
    const capFace = position.count / 3 - 1;   // ostatnie ściany to denko
    expect(() => fitCylinder(geometry, capFace)).toThrow(/płaska/);
});

function values(draft: Record<string, string> = {}): Values {
    const result = emptyDocument().values;
    for (const [key, value] of Object.entries(draft)) result[key] = valueOf(key, value);
    return result;
}
const close = (a: THREE.Vector3, b: THREE.Vector3) => expect(a.distanceTo(b)).toBeLessThan(1e-6);

it("wskazanie osi J2: nowy układ J2, pozostałe układy i TCP bez zmian w świecie", () => {
    const before = values(), frames = worldFrames(before);
    const direction = new THREE.Vector3(.05, 1, .1).normalize(), point = new THREE.Vector3(.004, .01, .215);
    const edit = applyJointAxis(before, 2, point, direction);
    expect(Object.keys(edit.draft).sort()).toEqual([...["origin_x", "origin_y", "origin_z", "origin_roll", "origin_pitch", "origin_yaw"].map(k => `joint.2.${k}`),
        ...["origin_x", "origin_y", "origin_z", "origin_roll", "origin_pitch", "origin_yaw"].map(k => `joint.3.${k}`)].sort());
    const after = worldFrames(values(edit.draft));
    close(after.joints[1].position, point);
    expect(worldAxis(after.joints[1], 2).angleTo(direction)).toBeLessThan(1e-5);
    for (const i of [0, 2, 3, 4, 5]) {
        close(after.joints[i].position, frames.joints[i].position);
        expect(after.joints[i].quaternion.angleTo(frames.joints[i].quaternion)).toBeLessThan(1e-5);
    }
    close(after.tool.position, frames.tool.position);
    // Zwrot osi zgodny z dotychczasowym; „odwróć” daje przeciwny.
    expect(applyJointAxis(before, 2, point, direction.clone().negate()).direction.dot(direction)).toBeGreaterThan(.99);
    expect(applyJointAxis(before, 2, point, direction, true).direction.dot(direction)).toBeLessThan(-.99);
});

it("kierunek bliski osi bazy przyciągany dokładnie; TCP z punktu; kontrole geometrii", () => {
    const before = values();
    const nearly = new THREE.Vector3(Math.sin(THREE.MathUtils.degToRad(.5)), 0, 1).normalize();
    const edit = applyJointAxis(before, 1, new THREE.Vector3(0, 0, .1), nearly);
    expect(edit.snapped).toBe(true);
    expect(edit.direction.toArray()).toEqual([0, 0, 1]);
    expect(edit.draft["joint.1.origin_roll"]).toBe("0"); expect(edit.draft["joint.1.origin_pitch"]).toBe("0");
    const tcp = new THREE.Vector3(.5, .02, .3);
    const after = worldFrames(values(applyTcpPoint(before, tcp)));
    close(after.tool.position, tcp);
    const checks = geometryChecks(before);
    expect(checks.j2j3ParallelDeg).toBeCloseTo(0, 6);
    for (const item of checks.wrist) expect(item.distanceMm).toBeLessThan(1e-6);
});
