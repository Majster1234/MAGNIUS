import * as THREE from "three";
import { describe, expect, it } from "vitest";
import { lowestAlong, rootUnderBase, worldFrames } from "./frameEdit";

// J1 wskazany na modelu z CAD: oś 200/200 mm od zera złożenia, układ J1 76 mm nad zerem.
const values = { "joint.1.origin_x": 200, "joint.1.origin_y": 200, "joint.1.origin_z": 76, "joint.2.origin_z": 124, "joint.3.origin_z": 250 };
const baseBox = (bottom: number) => {
    const geometry = new THREE.BoxGeometry(.2, .2, .06); geometry.translate(.2, .2, bottom / 1000 + .03);
    return [{ geometry, matrix: new THREE.Matrix4() }];
};

describe("zero robota pod podstawą", () => {
    it("przenosi zero na oś J1 w najniższy punkt podstawy; model i J1 przesunięte o ten sam wektor", () => {
        const shift = rootUnderBase(values, { x: 0, y: 0, z: 0 }, (point, axis) => lowestAlong(baseBox(-10), point, axis));
        expect(shift.needed).toBe(true);
        expect(shift.root.toArray().map(v => Math.round(v * 1e6) / 1e3)).toEqual([200, 200, -10]);
        expect(shift.transform).toEqual({ x: -200, y: -200, z: 10 });
        expect(shift.draft).toEqual({ "joint.1.origin_x": "0", "joint.1.origin_y": "0", "joint.1.origin_z": "86" });
        // Pozostałe przeguby są względne: położenie J2 względem J1 bez zmian.
        const after = worldFrames({ ...values, ...Object.fromEntries(Object.entries(shift.draft).map(([k, v]) => [k, Number(v)])) });
        expect(after.joints[1].position.z * 1000).toBeCloseTo(86 + 124, 6);
        expect(after.joints[0].position.x).toBeCloseTo(0, 9);
    });
    it("gdy zero już jest pod podstawą — brak zmiany", () => {
        const done = { ...values, "joint.1.origin_x": 0, "joint.1.origin_y": 0, "joint.1.origin_z": 86 };
        const box = baseBox(-10); box[0].geometry.translate(-.2, -.2, .01);
        expect(rootUnderBase(done, { x: -200, y: -200, z: 10 }, (point, axis) => lowestAlong(box, point, axis)).needed).toBe(false);
    });
    it("bez siatki podstawy: zero na osi J1, wysokość bez zmiany", () => {
        const shift = rootUnderBase(values, { x: 0, y: 0, z: 0 }, null);
        expect(shift.draft).toEqual({ "joint.1.origin_x": "0", "joint.1.origin_y": "0", "joint.1.origin_z": "76" });
        expect(shift.transform).toEqual({ x: -200, y: -200, z: 0 });
    });
});
