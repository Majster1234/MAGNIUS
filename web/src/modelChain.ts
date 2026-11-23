import * as THREE from "three";
import type { AssemblyTransform, MeshInfo } from "./robotModel";

// Tryb modułowy: łańcuch wystąpień elementów od podstawy (backend: robot_model.py, chain.json). Każde wystąpienie
// leży względem poprzednika: placement (x/y/z [mm], roll/pitch/yaw [°], Rz·Ry·Rx) wyliczany ze spasowania (mate):
// walec na poprzedniku ↔ walec na elemencie (współosiowo), obrót wokół wspólnej osi, przesunięcie wzdłuż niej
// i opcjonalny styk czół (płaszczyzny prostopadłe do osi). Punkty w mm w układach plików.
export type Vec3 = number[];
export type MatePlane = { point: Vec3; normal: Vec3 };
export type Mate = { parent_point: Vec3; parent_direction: Vec3; element_point: Vec3; element_direction: Vec3; flip: boolean;
    angle_deg: number; offset_mm: number; parent_plane: MatePlane | null; element_plane: MatePlane | null; needs_parent?: boolean };
export type Placement = AssemblyTransform;
export type ChainElement = { id: string; mesh: string; mate: Mate | null; placement: Placement };
export type ModelChain = { revision: string; up: "z" | "y"; elements: ChainElement[] };
export const zeroPlacement = (): Placement => ({x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0});
export const emptyChain = (): ModelChain => ({revision: "initial", up: "z", elements: []});

export function newId() {
    const bytes = new Uint8Array(16); crypto.getRandomValues(bytes);
    return Array.from(bytes, b => b.toString(16).padStart(2, "0")).join("");
}
export const saveChain = async (chain: Omit<ModelChain, "revision">, expected: string): Promise<ModelChain> => {
    const response = await fetch("/api/v1/robot/model/chain", {method: "PUT", headers: {"Content-Type": "application/json"},
        body: JSON.stringify({expected_revision: expected, chain})});
    const body = await response.json().catch(() => ({}));
    if (!response.ok) throw Error(typeof body?.detail === "string" ? body.detail : `Błąd ${response.status}`);
    return body as ModelChain;
};

const v = (values: Vec3) => new THREE.Vector3(values[0], values[1], values[2]);
const rad = THREE.MathUtils.degToRad, deg = THREE.MathUtils.radToDeg;
export function placementMatrix(p: Placement) {
    return new THREE.Matrix4().compose(new THREE.Vector3(p.x / 1000, p.y / 1000, p.z / 1000),
        new THREE.Quaternion().setFromEuler(new THREE.Euler(rad(p.roll), rad(p.pitch), rad(p.yaw), "ZYX")), new THREE.Vector3(1, 1, 1));
}
const round = (value: number, digits: number) => { const f = 10 ** digits, r = Math.round(value * f) / f; return Object.is(r, -0) ? 0 : r; };
export function matrixPlacement(m: THREE.Matrix4): Placement {
    const position = new THREE.Vector3(), quaternion = new THREE.Quaternion(), scale = new THREE.Vector3();
    m.decompose(position, quaternion, scale);
    const euler = new THREE.Euler().setFromQuaternion(quaternion, "ZYX");
    return {x: round(position.x * 1000, 4), y: round(position.y * 1000, 4), z: round(position.z * 1000, 4),
        roll: round(deg(euler.x), 5), pitch: round(deg(euler.y), 5), yaw: round(deg(euler.z), 5)};
}
// Układ spasowania: początek w punkcie osi [mm → m], Z wzdłuż kierunku; X deterministycznie prostopadły.
export function axisFrame(point: Vec3, direction: Vec3) {
    const z = v(direction).normalize();
    const x = new THREE.Vector3().crossVectors(Math.abs(z.x) < .9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0), z).normalize();
    const y = new THREE.Vector3().crossVectors(z, x);
    return new THREE.Matrix4().makeBasis(x, y, z).setPosition(v(point).multiplyScalar(.001));
}
const elementDirection = (mate: Pick<Mate, "element_direction" | "flip">) => mate.element_direction.map(value => mate.flip ? -value : value);
// Styk czół: położenie płaszczyzn wzdłuż wspólnej osi (poprzednik i element) → przesunięcie dosuwające je do siebie [mm].
export function contactOffset(mate: Mate) {
    if (!mate.parent_plane || !mate.element_plane) return null;
    const d1 = v(mate.parent_direction).normalize(), d2 = v(elementDirection(mate)).normalize();
    const s1 = v(mate.parent_plane.point).sub(v(mate.parent_point)).dot(d1);
    const z2 = v(mate.element_plane.point).sub(v(mate.element_point)).dot(d2);
    return s1 - z2;
}
// Odchylenie normalnej czoła od osi [°] (styk działa wzdłuż osi — ostrzeżenie przy dużym odchyleniu).
export function planeTiltDeg(plane: MatePlane, direction: Vec3) {
    return deg(Math.acos(Math.min(1, Math.abs(v(plane.normal).normalize().dot(v(direction).normalize())))));
}
export function mateMatrix(mate: Mate) {
    const offset = (contactOffset(mate) ?? 0) + mate.offset_mm;
    return axisFrame(mate.parent_point, mate.parent_direction)
        .multiply(new THREE.Matrix4().makeRotationZ(rad(mate.angle_deg)))
        .multiply(new THREE.Matrix4().makeTranslation(0, 0, offset / 1000))
        .multiply(axisFrame(mate.element_point, elementDirection(mate)).invert());
}
// Domyślny obrót wokół osi: najbliższy „bez obrotu” względem poprzednika (pliki z jednego projektu CAD zwykle mają
// zgodne osie) — maksimum śladu macierzy obrotu, krok 0,5°.
export function bestAngle(mate: Mate) {
    let best = 0, score = -Infinity;
    for (let step = 0; step < 720; step++) {
        const angle = step / 2, m = mateMatrix({...mate, angle_deg: angle}).elements, trace = m[0] + m[5] + m[10];
        if (trace > score + 1e-9) { score = trace; best = angle; }
    }
    return best > 180 ? best - 360 : best;
}
// Domyślny zwrot: element po przeciwnej stronie styku niż poprzednik (środki brył wzdłuż osi).
export function defaultFlip(mate: Pick<Mate, "parent_point" | "parent_direction" | "element_point" | "element_direction">, parentCenter: THREE.Vector3, elementCenter: THREE.Vector3) {
    const c1 = parentCenter.clone().multiplyScalar(1000).sub(v(mate.parent_point)).dot(v(mate.parent_direction));
    const c2 = elementCenter.clone().multiplyScalar(1000).sub(v(mate.element_point)).dot(v(mate.element_direction));
    return Math.sign(c1 || 1) * Math.sign(c2 || 1) > 0;
}
export const upMatrix = (up: "z" | "y") => up === "y" ? new THREE.Matrix4().makeRotationX(Math.PI / 2) : new THREE.Matrix4();
// Położenia wystąpień w układzie podstawy (pozycja zerowa): iloczyn położeń względem poprzedników.
export function worldMatrices(chain: Pick<ModelChain, "up" | "elements">) {
    const result: THREE.Matrix4[] = [];
    let current = upMatrix(chain.up);
    chain.elements.forEach((element, index) => {
        current = index === 0 ? current.clone() : current.clone().multiply(placementMatrix(element.placement));
        result.push(current);
    });
    return result;
}
// Element z aktualnym spasowaniem: położenie ze spasowania (chyba że czeka na nowe mocowanie u poprzednika).
export function withMate(element: ChainElement, mate: Mate | null): ChainElement {
    return {...element, mate, placement: mate && !mate.needs_parent ? matrixPlacement(mateMatrix(mate)) : element.placement};
}
// Wstawienie: następny element zachowuje miejsce w przestrzeni (przeliczone położenie względem nowego poprzednika)
// i czeka na wskazanie mocowania na nowym elemencie.
export function insertElement(elements: ChainElement[], at: number, element: ChainElement) {
    const result = [...elements.slice(0, at), element, ...elements.slice(at)];
    const next = result[at + 1];
    if (next && at > 0) result[at + 1] = {...next, mate: next.mate && {...next.mate, needs_parent: true},
        placement: matrixPlacement(placementMatrix(element.placement).invert().multiply(placementMatrix(next.placement)))};
    return result;
}
// Usunięcie: następny dołącza się do poprzednika w tym samym miejscu (złożone położenie) i czeka na nowe mocowanie.
export function removeElement(elements: ChainElement[], at: number) {
    const removed = elements[at], result = [...elements.slice(0, at), ...elements.slice(at + 1)], next = result[at];
    if (next && at > 0) result[at] = {...next, mate: next.mate && {...next.mate, needs_parent: true},
        placement: matrixPlacement(placementMatrix(removed.placement).multiply(placementMatrix(next.placement)))};
    if (at === 0 && result[0]) result[0] = {...result[0], mate: null, placement: zeroPlacement()};
    return result;
}
export const roleOf = (mesh: MeshInfo | undefined) => mesh?.element?.role ?? null;
export type JointRow = { joint: number; index: number; point: THREE.Vector3; direction: THREE.Vector3; fromPrevious: THREE.Vector3; angleDeg: number | null; via: string[] };
// Osie z łańcucha: modele osi z wskazaną osią, w układzie podstawy [mm]; odległość od poprzedniej osi (dla J1 —
// od początku pliku podstawy) i elementy pomiędzy (liczą się do kinematyki).
export function jointRows(chain: Pick<ModelChain, "up" | "elements">, meshes: Map<string, MeshInfo>) {
    const worlds = worldMatrices(chain), rows: JointRow[] = [];
    let previous = new THREE.Vector3(), previousDirection: THREE.Vector3 | null = null, via: string[] = [];
    chain.elements.forEach((element, index) => {
        const mesh = meshes.get(element.mesh), axis = mesh?.element?.role === "axis" ? mesh.element.axis : null;
        if (!axis) { via.push(mesh?.name ?? "?"); return; }
        const point = v(axis.point_mm).multiplyScalar(.001).applyMatrix4(worlds[index]).multiplyScalar(1000);
        const direction = v(axis.direction).applyMatrix3(new THREE.Matrix3().setFromMatrix4(worlds[index])).normalize();
        rows.push({joint: rows.length + 1, index, point, direction, fromPrevious: point.clone().sub(previous),
            angleDeg: previousDirection ? deg(previousDirection.angleTo(direction)) : null, via});
        previous = point; previousDirection = direction; via = [];
    });
    return {rows, after: via};
}
// Kierunek w opisie: „+Z”, gdy niemal zgodny z osią układu, inaczej składowe.
export function directionLabel(direction: THREE.Vector3) {
    const axes: [string, THREE.Vector3][] = [["X", new THREE.Vector3(1, 0, 0)], ["Y", new THREE.Vector3(0, 1, 0)], ["Z", new THREE.Vector3(0, 0, 1)]];
    for (const [name, axis] of axes) {
        const dot = direction.dot(axis);
        if (Math.abs(dot) > Math.cos(rad(.5))) return `${dot > 0 ? "+" : "−"}${name}`;
    }
    return direction.toArray().map(value => value.toFixed(3)).join(" / ");
}

// ── Etap 3: kinematyka z łańcucha ──
// Człony: element z osią Jn obraca się z Jn i zaczyna człon n (część z gniazdem łożyska / czopem); elementy przed
// pierwszą osią — podstawa; po J6 — człon 6 (kołnierz, narzędzie).
export type ModelLinkName = "base" | "link_1" | "link_2" | "link_3" | "link_4" | "link_5" | "link_6";
export function chainLinks(chain: Pick<ModelChain, "elements">, meshes: Map<string, MeshInfo>): ModelLinkName[] {
    let n = 0;
    return chain.elements.map(element => { if (meshes.get(element.mesh)?.element?.role === "axis") n++; return (n ? `link_${Math.min(n, 6)}` : "base") as ModelLinkName; });
}
// flipped — zwrot osi z pliku przeciwny do konwencji (dodatni obrót odwrotnie: J1 zgodnie z zegarem z góry, J2/J3/J5 do tyłu / w górę).
export type ChainJointFrame = { joint: number; index: number; frame: THREE.Matrix4; direction: THREE.Vector3; snapped: boolean; flipped: boolean };
// Opis dodatniego obrotu w układzie bazy (X przód, Y lewo, Z góra): ruch punktu `probe` (np. następnego przegubu)
// względem osi; oś ∥ ramieniu (J4, J6) — ruch punktu powyżej osi.
export function motionLabel(axis: THREE.Vector3, probe: THREE.Vector3) {
    let motion = new THREE.Vector3().crossVectors(axis, probe);
    if (motion.length() < .2 * probe.length() || probe.length() < 1e-6) motion = new THREE.Vector3().crossVectors(axis, Math.abs(axis.z) > .9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 0, 1));
    const names = [["do przodu", "do tyłu"], ["w lewo", "w prawo"], ["w górę", "w dół"]];
    const k = [0, 1, 2].reduce((best, i) => Math.abs(motion.getComponent(i)) > Math.abs(motion.getComponent(best)) ? i : best, 0);
    return names[k][motion.getComponent(k) >= 0 ? 0 : 1];
}
export type ChainFrames = { base: THREE.Matrix4 | null; baseFromMesh: boolean; joints: ChainJointFrame[]; missingAxis: number | null; links: ModelLinkName[] };
const JOINT_AXES = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0]];
const SNAP = Math.cos(rad(1));
function snap(direction: THREE.Vector3, references: THREE.Vector3[]) {
    for (const reference of references) for (const sign of [1, -1]) {
        const candidate = reference.clone().normalize().multiplyScalar(sign);
        if (direction.dot(candidate) > SNAP) return {direction: candidate, snapped: true};
    }
    return {direction: direction.clone().normalize(), snapped: false};
}
// Układy przegubów (pozycja zerowa) w układzie bazy robota:
// - baza: na osi J1, w najniższym punkcie siatek podstawy wzdłuż osi; Z bazy = oś J1 zwrócona „w górę” podglądu,
//   Y bazy = oś J2 (zwrot z pliku) rzutowana prostopadle do Z, X = Y × Z (bez J2: najmniejszy obrót układu podglądu);
// - J1: punkt wskazanego walca; Jn (n > 1): rzut początku poprzedniego układu na oś n (osie równoległe → czyste
//   przesunięcie, prostopadłe → wspólna normalna w tym punkcie);
// - orientacja Jn: najmniejszy obrót układu poprzedniego, by jego lokalna oś przegubu (J1 z, J2–J3 y, J4 x, J5 y, J6 x)
//   pokryła się z osią z pliku (zwrot z pliku = dodatni kąt); kierunek < 1° od osi bazy/poprzedniego układu → dokładnie.
export function chainFrames(chain: Pick<ModelChain, "up" | "elements">, meshes: Map<string, MeshInfo>, geometries: Map<string, THREE.BufferGeometry>): ChainFrames {
    const worlds = worldMatrices(chain), links = chainLinks(chain, meshes);
    const axes: {index: number; point: THREE.Vector3; direction: THREE.Vector3}[] = [];
    let missingAxis: number | null = null;
    chain.elements.forEach((element, index) => {
        const element_ = meshes.get(element.mesh)?.element;
        if (element_?.role !== "axis" || missingAxis !== null) return;
        if (!element_.axis) { missingAxis = axes.length + 1; return; }
        axes.push({index, point: v(element_.axis.point_mm).multiplyScalar(.001).applyMatrix4(worlds[index]),
            direction: v(element_.axis.direction).applyMatrix3(new THREE.Matrix3().setFromMatrix4(worlds[index])).normalize()});
    });
    if (!axes.length) return {base: null, baseFromMesh: false, joints: [], missingAxis, links};
    const z = new THREE.Vector3(0, 0, 1), first = axes[0], unit = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), z];
    const up = snap(first.direction, unit).direction;
    const baseZ = up.dot(z) >= 0 ? up : up.clone().negate();
    // X bazy = przód robota: strona, po której oś J2 jest odsunięta od osi J1 (wspólna prostopadła); osie J1 i J2 się
    // przecinają → strona, w którą wychodzi dalszy łańcuch (środek siatek za J2). Y = Z × X (w lewo), Z w górę.
    let baseRotation = new THREE.Matrix4().makeRotationFromQuaternion(new THREE.Quaternion().setFromUnitVectors(z, baseZ));
    if (axes[1]) {
        const d2 = axes[1].direction, delta = axes[1].point.clone().sub(first.point), normal = new THREE.Vector3().crossVectors(baseZ, d2);
        let front: THREE.Vector3 | null = null;
        if (normal.length() > 1e-6) { normal.normalize(); const side = delta.dot(normal); if (Math.abs(side) > .0005) front = normal.multiplyScalar(Math.sign(side)); }
        else { const offset = delta.clone().addScaledVector(baseZ, -delta.dot(baseZ)); if (offset.length() > .0005) front = offset.normalize(); }
        if (!front) {
            const box = new THREE.Box3();
            chain.elements.forEach((element, index) => { const geometry = geometries.get(element.mesh); if (geometry?.boundingBox && index > axes[1].index) box.union(geometry.boundingBox.clone().applyMatrix4(worlds[index])); });
            if (!box.isEmpty()) {
                const reach = box.getCenter(new THREE.Vector3()).sub(first.point);
                reach.addScaledVector(baseZ, -reach.dot(baseZ)).addScaledVector(d2, -reach.dot(d2));
                if (reach.length() > .001) front = reach.normalize();
            }
        }
        front ??= new THREE.Vector3().crossVectors(d2, baseZ);
        front.addScaledVector(baseZ, -front.dot(baseZ));
        if (front.length() > .1) {
            front = snap(front.normalize(), unit).direction;
            baseRotation = new THREE.Matrix4().makeBasis(front, new THREE.Vector3().crossVectors(baseZ, front), baseZ);
        }
    }
    // Najniższy punkt siatek podstawy wzdłuż osi J1.
    let lowest = Infinity;
    const vertex = new THREE.Vector3();
    chain.elements.forEach((element, index) => {
        const geometry = geometries.get(element.mesh), positions = geometry?.getAttribute("position");
        if (links[index] !== "base" || !positions) return;
        for (let i = 0; i < positions.count; i++) lowest = Math.min(lowest, vertex.fromBufferAttribute(positions, i).applyMatrix4(worlds[index]).sub(first.point).dot(baseZ));
    });
    const baseFromMesh = Number.isFinite(lowest);
    const origin = baseFromMesh ? first.point.clone().addScaledVector(baseZ, lowest) : first.point.clone().addScaledVector(baseZ, -first.point.dot(baseZ));
    const base = baseRotation.clone().setPosition(origin), toBase = base.clone().invert();
    const joints: ChainJointFrame[] = [];
    // Układ konwencji C: najmniejszy obrót poprzedniego, lokalna oś przegubu wzdłuż osi z modelu ze zwrotem najbliższym
    // poprzedniemu (bez obrotów o 180° — J1 w górę, J2/J3/J5 dodatnio pochylają do przodu / w dół, przy bazie X = przód).
    // Zwrot z pliku przeciwny do konwencji → układ przegubu F = C·obrót 180° wokół osi poprzecznej (J1: X, pozostałe: Z);
    // dalsze przeguby z modelu liczone są od C, więc geometria za osią się nie zmienia.
    let previous = base.clone();
    axes.slice(0, 6).forEach((axis, i) => {
        const rotation = new THREE.Matrix4().extractRotation(previous);
        const references = [0, 1, 2].flatMap(k => [new THREE.Vector3().setComponent(k, 1).applyMatrix4(baseRotation), new THREE.Vector3().setComponent(k, 1).applyMatrix4(rotation)]);
        const {direction, snapped} = snap(axis.direction, references);
        const previousOrigin = new THREE.Vector3().setFromMatrixPosition(previous);
        const point = i === 0 ? axis.point.clone() : axis.point.clone().addScaledVector(direction, previousOrigin.clone().sub(axis.point).dot(direction));
        const local = v(JOINT_AXES[i]).applyMatrix4(rotation).normalize();
        const flipped = direction.dot(local) < 0, conventional = flipped ? direction.clone().negate() : direction;
        const convention = new THREE.Matrix4().makeRotationFromQuaternion(new THREE.Quaternion().setFromUnitVectors(local, conventional)).multiply(rotation).setPosition(point);
        const frame = flipped ? convention.clone().multiply(i === 0 ? new THREE.Matrix4().makeRotationX(Math.PI) : new THREE.Matrix4().makeRotationZ(Math.PI)) : convention;
        joints.push({joint: i + 1, index: axis.index, frame: toBase.clone().multiply(frame), direction: direction.clone().transformDirection(toBase), snapped, flipped});
        previous = convention;
    });
    return {base, baseFromMesh, joints, missingAxis, links};
}
// Siatki łańcucha do podglądów (konfiguracja, sterowanie): człon i położenie w układzie bazy robota.
export async function loadChainMeshes(model: { chain?: ModelChain; meshes: MeshInfo[] }, safety = false) {
    const chain = model.chain;
    if (!chain?.elements.length) return [];
    const meshes = new Map(model.meshes.map(mesh => [mesh.id, mesh]));
    const ready = chain.elements.filter(element => meshes.get(element.mesh)?.state === "ready");
    const { meshGeometry } = await import("./robotModel");
    const geometries = new Map<string, THREE.BufferGeometry>();
    await Promise.all([...new Set(ready.map(element => element.mesh))].map(async id => geometries.set(id, await meshGeometry(id))));
    const frames = chainFrames(chain, meshes, geometries), worlds = worldMatrices(chain), toBase = frames.base ? frames.base.clone().invert() : new THREE.Matrix4();
    return Promise.all(chain.elements.map(async (element, index) => {
        const mesh = meshes.get(element.mesh), geometry = geometries.get(element.mesh);
        if (!mesh || !geometry) return null;
        return {link: frames.links[index], id: element.id, name: mesh.name, geometry, matrix: toBase.clone().multiply(worlds[index]),
            safety: safety && mesh.safety.state === "ready" ? await meshGeometry(element.mesh, "safety").catch(() => undefined) : undefined};
    })).then(items => items.filter(item => item !== null));
}
