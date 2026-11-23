import { expect, it } from "vitest";
import * as THREE from "three";
import { bestAngle, contactOffset, defaultFlip, insertElement, jointRows, mateMatrix, matrixPlacement, placementMatrix, removeElement, worldMatrices,
    withMate, zeroPlacement, type ChainElement, type Mate } from "./modelChain";
import type { MeshInfo } from "./robotModel";

const close = (a: THREE.Vector3, b: number[], digits = 6) => a.toArray().forEach((value, i) => expect(value).toBeCloseTo(b[i], digits));
// Kolumna: kołnierz wyjściowy w (0, 0, 120) mm, oś +Z; moduł: czop w (10, 0, 0) mm wzdłuż −Y pliku (inny układ pliku).
const mate: Mate = {parent_point: [0, 0, 120], parent_direction: [0, 0, 1], element_point: [10, 0, 0], element_direction: [0, -1, 0], flip: false,
    angle_deg: 0, offset_mm: 0, parent_plane: null, element_plane: null};

it("spasowanie: walec elementu współosiowy z walcem poprzednika, przesunięcie i obrót wokół osi", () => {
    const m = mateMatrix(mate);
    close(new THREE.Vector3(.01, 0, 0).applyMatrix4(m), [0, 0, .12]);                    // punkt osi elementu → punkt osi poprzednika
    close(new THREE.Vector3(0, -1, 0).applyMatrix3(new THREE.Matrix3().setFromMatrix4(m)), [0, 0, 1]);   // kierunek → kierunek
    const shifted = mateMatrix({...mate, offset_mm: 5, angle_deg: 90});
    close(new THREE.Vector3(.01, 0, 0).applyMatrix4(shifted), [0, 0, .125]);
    // Obrót wokół osi nie zmienia położenia punktów na osi, a punkt poza osią obraca się o 90°.
    const off = new THREE.Vector3(.03, 0, 0);
    const a = off.clone().applyMatrix4(m), b = off.clone().applyMatrix4(mateMatrix({...mate, angle_deg: 90}));
    expect(a.distanceTo(new THREE.Vector3(0, 0, .12))).toBeCloseTo(b.distanceTo(new THREE.Vector3(0, 0, .12)), 9);
    expect(a.clone().sub(new THREE.Vector3(0, 0, .12)).angleTo(b.clone().sub(new THREE.Vector3(0, 0, .12)))).toBeCloseTo(Math.PI / 2, 6);
});

it("styk czół: przesunięcie dosuwa płaszczyzny; zwrot przelicza styk", () => {
    // Czoło kołnierza 2 mm nad punktem osi; czoło elementu 4 mm „pod” jego punktem osi (wzdłuż −kierunku elementu).
    const withPlanes: Mate = {...mate, parent_plane: {point: [0, 5, 122], normal: [0, 0, 1]}, element_plane: {point: [10, 4, 0], normal: [0, 1, 0]}};
    expect(contactOffset(withPlanes)).toBeCloseTo(6, 9);
    const m = mateMatrix(withPlanes);
    expect(new THREE.Vector3(.01, .004, 0).applyMatrix4(m).z).toBeCloseTo(.122, 9);   // czoło elementu na czole poprzednika
    expect(contactOffset({...withPlanes, flip: true})).toBeCloseTo(-2, 9);
});

it("domyślny zwrot i obrót: element po drugiej stronie styku, bez zbędnego obrotu", () => {
    const parentCenter = new THREE.Vector3(0, 0, .06), elementCenter = new THREE.Vector3(.01, -.03, 0);   // moduł „wyrasta” wzdłuż −Y pliku
    expect(defaultFlip(mate, parentCenter, elementCenter)).toBe(false);
    expect(defaultFlip({...mate, element_direction: [0, 1, 0]}, parentCenter, elementCenter)).toBe(true);
    // Element w tym samym układzie co poprzednik: najlepszy obrót = brak obrotu (macierz jednostkowa).
    const same: Mate = {...mate, element_point: [0, 0, 0], element_direction: [0, 0, 1]};
    const m = mateMatrix({...same, angle_deg: bestAngle(same)});
    const p = matrixPlacement(m);
    expect([p.roll, p.pitch, p.yaw].map(value => Math.abs(value) < 1e-6 ? 0 : value)).toEqual([0, 0, 0]);
    expect(p.z).toBeCloseTo(120, 6);
});

it("położenie ↔ macierz (Rz·Ry·Rx), łańcuch w układzie podstawy z osią Y w górę", () => {
    const placement = {x: 10, y: -20, z: 30, roll: 15, pitch: -40, yaw: 120};
    const back = matrixPlacement(placementMatrix(placement));
    for (const key of Object.keys(placement) as (keyof typeof placement)[]) expect(back[key]).toBeCloseTo(placement[key], 4);
    const elements: ChainElement[] = [{id: "a", mesh: "base", mate: null, placement: zeroPlacement()}, {id: "b", mesh: "m", mate: null, placement: {...zeroPlacement(), y: 100}}];
    const worlds = worldMatrices({up: "y", elements});
    close(new THREE.Vector3().applyMatrix4(worlds[1]), [0, 0, .1]);   // Y pliku podstawy → Z robota
});

it("wstawienie i usunięcie zachowują miejsce następnego elementu i oznaczają go do ponownego mocowania", () => {
    const b = withMate({id: "b", mesh: "m", mate: null, placement: zeroPlacement()}, mate), c = withMate({id: "c", mesh: "m", mate: null, placement: zeroPlacement()}, {...mate, angle_deg: 30});
    const elements: ChainElement[] = [{id: "a", mesh: "base", mate: null, placement: zeroPlacement()}, b, c];
    const before = worldMatrices({up: "z", elements})[2];
    const x = withMate({id: "x", mesh: "ext", mate: null, placement: zeroPlacement()}, {...mate, offset_mm: 80});
    const inserted = insertElement(elements, 2, x);
    expect(inserted.map(e => e.id)).toEqual(["a", "b", "x", "c"]);
    expect(inserted[3].mate?.needs_parent).toBe(true);
    const after = worldMatrices({up: "z", elements: inserted})[3];
    after.elements.forEach((value, i) => expect(value).toBeCloseTo(before.elements[i], 5));
    const removed = removeElement(inserted, 1);
    expect(removed.map(e => e.id)).toEqual(["a", "x", "c"]);
    const kept = worldMatrices({up: "z", elements: inserted})[2], moved = worldMatrices({up: "z", elements: removed})[1];
    kept.elements.forEach((value, i) => expect(value).toBeCloseTo(moved.elements[i], 5));
    expect(removed[1].mate?.needs_parent).toBe(true);
    // Ponowne mocowanie (needs_parent = false) liczy położenie ze spasowania.
    expect(withMate(removed[1], {...removed[1].mate!, needs_parent: false}).placement.z).toBeCloseTo(200, 4);
});

it("odległości osi: modele osi z osią w układzie podstawy, elementy pomiędzy", () => {
    const mesh = (id: string, name: string, role: "axis" | "structural" | null, axis?: {point_mm: number[]; direction: number[]}) => [id, {id, name, state: "ready",
        element: {revision: "r", role, axis: axis ? {...axis, radius_mm: 20, residual_mm: 0, faces: 40, coverage_deg: 360} : null}} as unknown as MeshInfo] as const;
    const meshes = new Map([mesh("base", "podstawa", null), mesh("j", "moduł", "axis", {point_mm: [0, 0, 40], direction: [0, 0, 1]}), mesh("col", "kolumna", "structural")]);
    const up = (z: number) => ({...zeroPlacement(), z});
    const elements: ChainElement[] = [{id: "a", mesh: "base", mate: null, placement: zeroPlacement()}, {id: "b", mesh: "j", mate: null, placement: up(90)},
        {id: "c", mesh: "col", mate: null, placement: up(80)}, {id: "d", mesh: "j", mate: null, placement: {...up(120), roll: 90}}];
    const {rows, after} = jointRows({up: "z", elements}, meshes);
    expect(rows.map(row => row.joint)).toEqual([1, 2]);
    close(rows[0].point, [0, 0, 130], 4);
    close(rows[1].fromPrevious, [0, -40, 160], 4);   // 90+80+120 = 290; punkt osi (0,0,40) po obrocie roll 90° → (0,−40,0)
    expect(rows[1].angleDeg).toBeCloseTo(90, 4);
    expect(rows[1].via).toEqual(["kolumna"]);
    expect(after).toEqual([]);
});

it("kinematyka z łańcucha (dane z modelu operatora 23.11.2026): baza pod podstawą, X bazy = przód (strona J2), zwroty względem konwencji", async () => {
    const { chainFrames, chainLinks, motionLabel } = await import("./modelChain");
    const { applyChainFrames, continueFrames } = await import("./frameEdit");
    const axisMesh = (id: string, name: string, point: number[], direction: number[]) => [id, {id, name, state: "ready",
        element: {revision: "r", role: "axis", axis: {point_mm: point, direction, radius_mm: 20, residual_mm: 0, faces: 100, coverage_deg: 357}}} as unknown as MeshInfo] as const;
    const plain = (id: string, name: string) => [id, {id, name, state: "ready", element: {revision: "r", role: "structural", axis: null}} as unknown as MeshInfo] as const;
    const build = (j3: number[]) => new Map([plain("base", "Podstawa_robota"), plain("bearing", "Łożysko 51218"), axisMesh("platform", "Platforma", [200, 200, 85.25], [0, 0, 1]),
        plain("platformJ2", "Platforma_J2"), axisMesh("j2", "Przegub J2", [159.907, 259.81, 164.358], [1, 0, 0]), axisMesh("j3", "Przegub J3", [252.85, 259.81, 371.206], j3)]);
    // Podstawa: bryła z = 20…89 mm (jak w pliku), łożysko obrócone na podstawie, reszta w układzie złożenia.
    const geometries = new Map([["base", new THREE.BoxGeometry(.2, .2, .069).translate(.2, .2, .0545)]]);
    const p = (x: number, y: number, zz: number, roll = 0) => ({x, y, z: zz, roll, pitch: 0, yaw: 0});
    const elements: ChainElement[] = [{id: "a", mesh: "base", mate: null, placement: zeroPlacement()}, {id: "b", mesh: "bearing", mate: null, placement: p(200, 200, 90, 180)},
        {id: "c", mesh: "platform", mate: null, placement: p(-200, 200, 90, 180)}, {id: "d", mesh: "platformJ2", mate: null, placement: zeroPlacement()},
        {id: "e", mesh: "j2", mate: null, placement: zeroPlacement()}, {id: "f", mesh: "j3", mate: null, placement: zeroPlacement()}];
    const asIs = build([1, 0, 1.67e-7]);
    expect(chainLinks({elements}, asIs)).toEqual(["base", "base", "link_1", "link_1", "link_2", "link_3"]);
    const frames = chainFrames({up: "z", elements}, asIs, geometries);
    expect(frames.baseFromMesh).toBe(true);
    // Przód = +Y złożenia (tam jest J2): X bazy = (0, 1, 0) w układzie złożenia.
    close(new THREE.Vector3(1, 0, 0).transformDirection(frames.base!), [0, 1, 0], 9);
    expect(frames.joints.map(joint => [joint.snapped, joint.flipped])).toEqual([[true, false], [true, true], [true, true]]);
    const draft = applyChainFrames({}, frames.joints.map(joint => joint.frame));
    const origin = (d: Record<string, string>, n: number) => ["x", "y", "z", "roll", "pitch", "yaw"].map(key => Number(d[`joint.${n}.origin_${key}`]));
    origin(draft, 1).forEach((value, i) => expect(value).toBeCloseTo([0, 0, 65.25, 0, 0, 0][i], 3));
    origin(draft, 2).slice(0, 5).forEach((value, i) => expect(value).toBeCloseTo([59.81, 0, 79.108, 0, 0][i], 3));
    expect(Math.abs(origin(draft, 2)[5])).toBeCloseTo(180, 3);
    origin(draft, 3).forEach((value, i) => expect(value).toBeCloseTo([0, 0, 206.848, 0, 0, 0][i], 3));
    // Obserwacja operatora: J2 „−” pochyla do przodu → dodatni do tyłu (zgodne ze zwrotem z pliku).
    const values = {"joint.4.origin_x": 56, "joint.4.origin_y": 3, "joint.4.origin_z": 34, "joint.4.origin_roll": 0, "joint.4.origin_pitch": 0, "joint.4.origin_yaw": 0};
    const at = (m: THREE.Matrix4) => new THREE.Vector3().setFromMatrixPosition(m);
    let full = continueFrames(values, frames.joints.map(joint => joint.frame));
    expect(motionLabel(frames.joints[1].direction, at(full.joints[2]).sub(at(full.joints[1])))).toBe("do tyłu");
    // J3 z odwróconym zwrotem pliku → zgodny z konwencją: dodatni J3 w dół (operator: „−” podnosi), J4 przed J3, J3 yaw 180° względem J2.
    const reversed = chainFrames({up: "z", elements}, build([-1, 0, 0]), geometries);
    expect(reversed.joints[2].flipped).toBe(false);
    expect(Math.abs(origin(applyChainFrames({}, reversed.joints.map(joint => joint.frame)), 3)[5])).toBeCloseTo(180, 3);
    full = continueFrames(values, reversed.joints.map(joint => joint.frame));
    expect(at(full.joints[3]).x - at(full.joints[2]).x).toBeCloseTo(.056, 6);
    expect(motionLabel(reversed.joints[2].direction, at(full.joints[3]).sub(at(full.joints[2])))).toBe("w dół");
});

it("oś przeciwna do lokalnej osi przegubu: obrót 180° wokół Z poprzedniego układu (bez przewracania „góry”)", async () => {
    const { chainFrames } = await import("./modelChain");
    const axisMesh = (id: string, point: number[], direction: number[]) => [id, {id, name: id, state: "ready",
        element: {revision: "r", role: "axis", axis: {point_mm: point, direction, radius_mm: 20, residual_mm: 0, faces: 100, coverage_deg: 357}}} as unknown as MeshInfo] as const;
    // J1 +Z, J2 +Y (baza: Y = J2), J3 −Y — przeciwnie do lokalnego y.
    const meshes = new Map([axisMesh("j1", [0, 0, 50], [0, 0, 1]), axisMesh("j2", [0, 0, 100], [0, 1, 0]), axisMesh("j3", [0, 0, 300], [0, -1, 0])]);
    const elements: ChainElement[] = ["j1", "j2", "j3"].map((mesh, i) => ({id: String(i), mesh, mate: null, placement: zeroPlacement()}));
    const frames = chainFrames({up: "z", elements}, meshes, new Map());
    const zAxis = new THREE.Vector3(0, 0, 1).transformDirection(frames.joints[2].frame);
    close(zAxis, [0, 0, 1], 9);
    close(new THREE.Vector3(0, 1, 0).transformDirection(frames.joints[2].frame), [0, -1, 0], 9);
});

it("osie wirtualne: przeniesienie zmienia tylko J1…Jk, dalsze przeguby i TCP jadą z konfiguracji za nowym Jk", async () => {
    const { applyChainFrames, continueFrames } = await import("./frameEdit");
    const { reverseAxis } = await import("./robotModel");
    const values = {"joint.4.origin_x": 220, "joint.4.origin_y": 0, "joint.4.origin_z": 0, "joint.4.origin_roll": 0, "joint.4.origin_pitch": 0, "joint.4.origin_yaw": 0,
        "tool.tcp_x": 80, "tool.tcp_y": 0, "tool.tcp_z": 0, "tool.tcp_roll": 0, "tool.tcp_pitch": 0, "tool.tcp_yaw": 0};
    const frames = [new THREE.Matrix4().makeTranslation(0, 0, .065), new THREE.Matrix4().makeTranslation(-.06, 0, .144), new THREE.Matrix4().makeTranslation(-.06, 0, .351)];
    const draft = applyChainFrames(values, frames);
    expect(Object.keys(draft).every(key => /^joint\.[123]\./.test(key))).toBe(true);
    expect(Number(draft["joint.3.origin_z"])).toBeCloseTo(207, 6);
    const full = continueFrames(values, frames);
    close(new THREE.Vector3().setFromMatrixPosition(full.joints[3]), [.16, 0, .351]);   // J4 = J3 + (220, 0, 0) mm
    expect(full.joints).toHaveLength(6);
    const axis = {point_mm: [0, 0, 0], direction: [1, 0, 0], radius_mm: 5, residual_mm: 0, faces: 10, coverage_deg: 360, reversed: false};
    expect(reverseAxis(axis)).toMatchObject({direction: [-1, -0, -0], reversed: true});
    expect(reverseAxis(reverseAxis(axis))).toMatchObject({direction: [1, 0, 0], reversed: false});
});
