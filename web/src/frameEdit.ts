import * as THREE from "three";
import { demoTranslations, jointAxes, originKeys, tcpKeys, type Values } from "./configurationData";

// Układy przegubów w pozycji zerowej z wartości konfiguracji (jak build_urdf i podgląd konfiguracji):
// T_0 = baza, T_i = T_(i−1)·origin_i (translacja [mm], obrót roll/pitch/yaw [°], Rz·Ry·Rx), TCP = T_6·tool.
// Lokalny kierunek osi przegubu jest stały (jointAxes); wskazanie osi zmienia orientację i położenie układu.
export type Frame = { position: THREE.Vector3; quaternion: THREE.Quaternion };
const rad = THREE.MathUtils.degToRad, deg = THREE.MathUtils.radToDeg;
const number = (value: Values[string], fallback: number) => typeof value === "number" && Number.isFinite(value) ? value : fallback;

function local(values: Values, keys: string[], prefix: string, translation: number[]) {
    const t = keys.slice(0, 3).map((key, i) => number(values[prefix + key], translation[i]) / 1000);
    const r = keys.slice(3).map(key => rad(number(values[prefix + key], 0)));
    return new THREE.Matrix4().compose(new THREE.Vector3(...t), new THREE.Quaternion().setFromEuler(new THREE.Euler(r[0], r[1], r[2], "ZYX")), new THREE.Vector3(1, 1, 1));
}
const frameOf = (matrix: THREE.Matrix4): Frame => {
    const position = new THREE.Vector3(), quaternion = new THREE.Quaternion(); matrix.decompose(position, quaternion, new THREE.Vector3());
    return {position, quaternion};
};
const matrixOf = (frame: Frame) => new THREE.Matrix4().compose(frame.position, frame.quaternion, new THREE.Vector3(1, 1, 1));

export function worldFrames(values: Values) {
    let current = new THREE.Matrix4();
    const joints = Array.from({length: 6}, (_, i) => frameOf(current = current.clone().multiply(local(values, originKeys, `joint.${i + 1}.`, demoTranslations[i]))));
    const tool = frameOf(current.clone().multiply(local(values, tcpKeys, "tool.", [80, 0, 0])));
    return {joints, tool};
}
export function worldAxis(frame: Frame, joint: number) {
    return new THREE.Vector3(...jointAxes[joint - 1]).applyQuaternion(frame.quaternion).normalize();
}

// Względne przekształcenie parent → child jako pola konfiguracji (mm, °), zaokrąglone do 0,0001.
function relative(parent: Frame, child: Frame, keys: string[], prefix: string) {
    const matrix = matrixOf(parent).invert().multiply(matrixOf(child));
    const {position, quaternion} = frameOf(matrix);
    const euler = new THREE.Euler().setFromQuaternion(quaternion, "ZYX");
    const values = [position.x * 1000, position.y * 1000, position.z * 1000, deg(euler.x), deg(euler.y), deg(euler.z)];
    return Object.fromEntries(keys.map((key, i) => [prefix + key, format(values[i])]));
}
const format = (value: number) => { const rounded = Math.round(value * 1e4) / 1e4; return String(Object.is(rounded, -0) || Math.abs(rounded) < 1e-9 ? 0 : rounded); };

const base: Frame = {position: new THREE.Vector3(), quaternion: new THREE.Quaternion()};
const SNAP_DEG = 1;

// Kierunek osi bliski (< 1°) osi bazy albo osi układu poprzedniego członu → dokładnie ta oś.
export function snapDirection(direction: THREE.Vector3, references: THREE.Vector3[]) {
    const d = direction.clone().normalize();
    for (const reference of references) for (const sign of [1, -1]) {
        const candidate = reference.clone().normalize().multiplyScalar(sign);
        if (d.angleTo(candidate) < rad(SNAP_DEG)) return {direction: candidate, snapped: true};
    }
    return {direction: d, snapped: false};
}

export type AxisEdit = { draft: Record<string, string>; direction: THREE.Vector3; position: THREE.Vector3; snapped: boolean; reversed: boolean; tiltDeg: number };
// Nowy układ przegubu `joint` (1–6) z osią przez `point` o kierunku `direction` (zwrot zgodny z obecną osią, chyba
// że reverse). Orientacja = najmniejszy obrót obecnego układu (zachowuje „ułożenie” wokół osi), położenie = punkt.
// Pozostałe układy i TCP zostają w świecie — zmieniają się pola origin tego przegubu i następnego (albo TCP).
export function applyJointAxis(values: Values, joint: number, point: THREE.Vector3, direction: THREE.Vector3, reverse = false): AxisEdit {
    const {joints, tool} = worldFrames(values);
    const frame = joints[joint - 1], parent = joint > 1 ? joints[joint - 2] : base;
    const current = worldAxis(frame, joint);
    const parentAxes = [0, 1, 2].map(i => new THREE.Vector3().setComponent(i, 1).applyQuaternion(parent.quaternion));
    const baseAxes = [0, 1, 2].map(i => new THREE.Vector3().setComponent(i, 1));
    const snap = snapDirection(direction, [...baseAxes, ...parentAxes]);
    let target = snap.direction;
    if (target.dot(current) < 0) target = target.clone().negate();
    if (reverse) target = target.clone().negate();
    const quaternion = new THREE.Quaternion().setFromUnitVectors(current, target).multiply(frame.quaternion);
    const edited: Frame = {position: point.clone(), quaternion};
    const draft = relative(parent, edited, originKeys, `joint.${joint}.`);
    Object.assign(draft, joint < 6 ? relative(edited, joints[joint], originKeys, `joint.${joint + 1}.`) : relative(edited, tool, tcpKeys, "tool."));
    return {draft, direction: target, position: point.clone(), snapped: snap.snapped, reversed: reverse, tiltDeg: deg(current.angleTo(target))};
}

// Układy J1…Jk z łańcucha modelu 3D (układ bazy robota) → pola konfiguracji J1…Jk. Dalsze przeguby i TCP zachowują
// zapisane wartości (względem poprzedniego przegubu) — jako bryły wirtualne przesuwają się razem z nowym Jk.
export function applyChainFrames(_values: Values, frames: THREE.Matrix4[]) {
    const draft: Record<string, string> = {};
    frames.slice(0, 6).forEach((frame, i) => Object.assign(draft, relative(i ? frameOf(frames[i - 1]) : base, frameOf(frame), originKeys, `joint.${i + 1}.`)));
    return draft;
}
// Pełny łańcuch układów (baza robota): J1…Jk z modelu, dalej przeguby i TCP z konfiguracji (wirtualne).
export function continueFrames(values: Values, frames: THREE.Matrix4[]) {
    const joints: THREE.Matrix4[] = [];
    for (let i = 0; i < 6; i++) joints.push(frames[i]?.clone() ?? (i ? joints[i - 1] : new THREE.Matrix4()).clone().multiply(local(values, originKeys, `joint.${i + 1}.`, demoTranslations[i])));
    return {joints, tool: joints[5].clone().multiply(local(values, tcpKeys, "tool.", [80, 0, 0]))};
}

// TCP w punkcie `point` (orientacja narzędzia względem kołnierza bez zmian).
export function applyTcpPoint(values: Values, point: THREE.Vector3) {
    const {joints} = worldFrames(values);
    const flange = joints[5];
    const localPoint = point.clone().applyMatrix4(matrixOf(flange).invert());
    return Object.fromEntries(tcpKeys.slice(0, 3).map((key, i) => [`tool.${key}`, format(localPoint.getComponent(i) * 1000)]));
}

// Kontrole geometrii: równoległość J2/J3, odległości osi nadgarstka J4–J6 (nadgarstek sferyczny ⇔ ~0).
export function geometryChecks(values: Values) {
    const {joints} = worldFrames(values);
    const axes = joints.map((frame, i) => ({point: frame.position, direction: worldAxis(frame, i + 1)}));
    const lineDistance = (a: typeof axes[number], b: typeof axes[number]) => {
        const cross = new THREE.Vector3().crossVectors(a.direction, b.direction), delta = b.point.clone().sub(a.point);
        if (cross.length() < 1e-9) return delta.clone().addScaledVector(a.direction, -delta.dot(a.direction)).length();
        return Math.abs(delta.dot(cross.normalize()));
    };
    return {
        j2j3ParallelDeg: deg(Math.min(axes[1].direction.angleTo(axes[2].direction), axes[1].direction.angleTo(axes[2].direction.clone().negate()))),
        wrist: [[3, 4], [4, 5], [3, 5]].map(([a, b]) => ({axes: `J${a + 1}–J${b + 1}`, distanceMm: lineDistance(axes[a], axes[b]) * 1000})),
    };
}

// ---------- Zero robota pod podstawą ----------
// Układ bazy robota (zero całego robota) leży zawsze na osi J1, w najniższym punkcie siatki podstawy (wzdłuż osi J1).
// Przeniesienie zera = przesunięcie układu bazy o wektor `root`: model 3D (złożenie → baza) i położenie J1 w konfiguracji
// zmieniają się o −root, pozostałe przeguby i TCP są względne (bez zmian). Kąty osi i fizyczny robot pozostają te same.
export type GeometrySource = { geometry: THREE.BufferGeometry; matrix: THREE.Matrix4 };
/** Najmniejsza współrzędna punktów siatek wzdłuż osi (axis jednostkowa) względem `point`; null — brak punktów. */
export function lowestAlong(sources: GeometrySource[], point: THREE.Vector3, axis: THREE.Vector3): number | null {
    let lowest = Infinity;
    const v = new THREE.Vector3();
    for (const {geometry, matrix} of sources) {
        const positions = geometry.getAttribute("position");
        if (!positions) continue;
        for (let i = 0; i < positions.count; i++) {
            v.fromBufferAttribute(positions, i).applyMatrix4(matrix).sub(point);
            const along = v.dot(axis);
            if (along < lowest) lowest = along;
        }
    }
    return Number.isFinite(lowest) ? lowest : null;
}
export type RootShift = { root: THREE.Vector3; needed: boolean; transform: { x: number; y: number; z: number }; draft: Record<string, string> };
/** Przesunięcie zera robota pod podstawę. `lowest` — funkcja najniższego punktu podstawy wzdłuż osi J1 (null — brak
 *  siatki podstawy: zero na osi J1 bez zmiany wysokości). `transform` — przesunięcie złożenie → baza [mm]. */
export function rootUnderBase(values: Values, transform: { x: number; y: number; z: number },
    lowest: ((point: THREE.Vector3, axis: THREE.Vector3) => number | null) | null): RootShift {
    const {joints} = worldFrames(values);
    const point = joints[0].position, axis = worldAxis(joints[0], 1);
    const along = lowest ? lowest(point, axis) : null;
    // Bez siatki podstawy: punkt osi J1 najbliższy obecnemu zeru (przesunięcie tylko w poprzek osi).
    const root = along !== null ? point.clone().addScaledVector(axis, along) : point.clone().addScaledVector(axis, -point.dot(axis));
    const mm = root.clone().multiplyScalar(1000);
    const draft = Object.fromEntries(originKeys.slice(0, 3).map((key, i) => [`joint.1.${key}`, format(point.getComponent(i) * 1000 - mm.getComponent(i))]));
    return {root, needed: mm.length() > .05,
        transform: {x: Number(format(transform.x - mm.x)), y: Number(format(transform.y - mm.y)), z: Number(format(transform.z - mm.z))}, draft};
}
