import * as THREE from "three";
import type { URDFRobot } from "urdf-loader";

export type HintTcpAxis = "x" | "y" | "z" | "rx" | "ry" | "rz";
export type MotionHint =
    | { kind: "joint"; axis: number; direction: number }
    | { kind: "tcp"; axis: HintTcpAxis; direction: number; frame: "base" | "tool" };

export type HintGeometry = { origin: THREE.Vector3; axis: THREE.Vector3; rotation: boolean; frame: THREE.Quaternion };

// Kierunek ruchu w układzie świata sceny.
// JOG osi: oś obrotu przegubu z URDF ze znakiem kierunku — dodatni kąt obraca model prawoskrętnie wokół tej osi.
// JOG TCP jak robot_kinematics/src/tcp_jog.cpp: oś = Unit(axis)·kierunek w układzie bazy (korzeń modelu), w układzie
// „Narzędzie” obrócona orientacją TCP; obrót wokół tej osi przechodzącej przez TCP.
export function hintGeometry(robot: URDFRobot, hint: MotionHint): HintGeometry | null {
    if (![1, -1].includes(hint.direction)) return null;
    robot.updateMatrixWorld(true);
    if (hint.kind === "joint") {
        const joint = robot.joints[`joint_${hint.axis}`];
        if (!joint) return null;
        const axis = joint.axis.clone().transformDirection(joint.matrixWorld).multiplyScalar(hint.direction);
        return { origin: joint.getWorldPosition(new THREE.Vector3()), axis, rotation: true, frame: joint.getWorldQuaternion(new THREE.Quaternion()) };
    }
    const tool = robot.joints.tool_fixed;
    if (!tool) return null;
    const index = { x: 0, y: 1, z: 2 }[hint.axis.slice(-1) as "x" | "y" | "z"];
    const frame = hint.frame === "tool" ? tool.getWorldQuaternion(new THREE.Quaternion()) : robot.getWorldQuaternion(new THREE.Quaternion());
    const axis = new THREE.Vector3().setComponent(index, hint.direction).applyQuaternion(frame);
    return { origin: tool.getWorldPosition(new THREE.Vector3()), axis, rotation: hint.axis.startsWith("r"), frame };
}

export function hintLabel(hint: MotionHint) {
    const sign = hint.direction > 0 ? "+" : "−";
    if (hint.kind === "joint") return `J${hint.axis} ${sign}`;
    const name = hint.axis.startsWith("r") ? `R${hint.axis[1].toUpperCase()}` : hint.axis.toUpperCase();
    return `TCP ${name}${sign} · ${hint.frame === "base" ? "baza" : "narzędzie"}`;
}

// Stonowane barwy zbliżone do palety panelu; nakładka półprzezroczysta, rysowana nad modelem.
const HINT_COLOR = 0xe2c27d, HINT_OPACITY = .78, AXIS_OPACITY = .62;
const AXIS_COLORS = [0xd98585, 0x86c493, 0x8ea6dd];
// Pojawianie/znikanie: zanik przezroczystości z lekkim powiększeniem; przy „ogranicz animacje” natychmiast.
const FADE_IN_MS = 180, FADE_OUT_MS = 140, GROW_FROM = .8;
const Y = new THREE.Vector3(0, 1, 0), Z = new THREE.Vector3(0, 0, 1);

type Faded = { material: THREE.MeshBasicMaterial; opacity: number };
function overlay(color: number, opacity: number, faded: Faded[]) {
    const material = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0, depthTest: false, depthWrite: false, side: THREE.DoubleSide });
    faded.push({ material, opacity }); return material;
}
// Strzałka wzdłuż +Y lokalnego: trzon + grot; czubek w (0, length, 0).
function arrow(length: number, radius: number, color: number, opacity: number, faded: Faded[]) {
    const group = new THREE.Group(), material = overlay(color, opacity, faded), head = Math.min(length * .28, radius * 6);
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length - head, 8), material);
    shaft.position.y = (length - head) / 2;
    const cone = new THREE.Mesh(new THREE.ConeGeometry(radius * 2.4, head, 12), material);
    cone.position.y = length - head / 2;
    group.add(shaft, cone); return group;
}
// Łuk 270° wokół +Z lokalnego, przeciwnie do zegara patrząc od +Z (obrót dodatni), grot na końcu łuku.
function arc(radius: number, tube: number, color: number, opacity: number, faded: Faded[]) {
    const group = new THREE.Group(), material = overlay(color, opacity, faded), span = Math.PI * 1.5;
    group.add(new THREE.Mesh(new THREE.TorusGeometry(radius, tube, 6, 40, span), material));
    const head = tube * 6, cone = new THREE.Mesh(new THREE.ConeGeometry(tube * 2.5, head, 12), material);
    // Koniec łuku w kącie span: punkt (0, −r, 0), styczna (1, 0, 0); grot wysunięty o połowę długości.
    cone.position.set(head / 2, -radius, 0); cone.rotation.z = -Math.PI / 2; group.add(cone);
    return { group, tip: new THREE.Vector3(head, -radius, 0) };
}

export function createMotionHint(scene: THREE.Scene, host: HTMLElement, size: number) {
    const s = THREE.MathUtils.clamp(size / .8, .5, 2.5), faded: Faded[] = [];
    const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
    const root = new THREE.Group(); root.visible = false; scene.add(root);
    const jointArc = arc(.042 * s, .0018 * s, HINT_COLOR, HINT_OPACITY, faded), tcpArc = arc(.03 * s, .0016 * s, HINT_COLOR, HINT_OPACITY, faded);
    const moveLength = .085 * s, move = arrow(moveLength, .0019 * s, HINT_COLOR, HINT_OPACITY, faded);
    const triad = new THREE.Group(), triadLength = .055 * s;
    AXIS_COLORS.forEach((color, i) => {
        const a = arrow(triadLength, .0011 * s, color, AXIS_OPACITY, faded); a.quaternion.setFromUnitVectors(Y, new THREE.Vector3().setComponent(i, 1)); triad.add(a);
    });
    for (const object of [jointArc.group, tcpArc.group, move, triad]) { object.visible = false; root.add(object); }
    root.traverse(object => { object.renderOrder = 7; });
    const label = window.document.createElement("span"); label.className = "preview-motion-hint"; label.hidden = true; host.append(label);
    const triadLabels = (["X", "Y", "Z"] as const).map((text, i) => {
        const element = window.document.createElement("span"); element.className = `preview-tcp-axis preview-motion-hint-axis preview-tcp-${text.toLowerCase()}`;
        element.textContent = text; element.dataset.subject = "hint"; element.hidden = true; host.append(element);
        return { element, point: new THREE.Vector3().setComponent(i, triadLength + .012 * s) };
    });
    let key = "", current: (HintGeometry & { hint: MotionHint }) | null = null, shown = false;
    let fade = 0, fadeFrom = 0, fadeStarted = 0, fadeTarget = 0;
    const tip = new THREE.Vector3();
    // Pokazuje geometrię podpowiedzi (bez zmiany przezroczystości).
    function place(hint: MotionHint, geometry: HintGeometry) {
        current = { ...geometry, hint };
        const tcp = hint.kind === "tcp", orientation = new THREE.Quaternion().setFromUnitVectors(current.rotation ? Z : Y, current.axis);
        jointArc.group.visible = !tcp; tcpArc.group.visible = tcp && current.rotation; move.visible = tcp && !current.rotation; triad.visible = tcp;
        root.position.copy(current.origin);
        for (const object of [jointArc.group, tcpArc.group, move]) object.quaternion.copy(orientation);
        triad.quaternion.copy(current.frame);
        const active = tcp ? current.rotation ? tcpArc : null : jointArc;
        if (active) tip.copy(active.tip).applyQuaternion(orientation).add(current.origin);
        // Przesuw: etykieta w ok. 60% strzałki — czubek przy krawędzi kadru bywa poza widokiem.
        else tip.copy(current.axis).multiplyScalar(moveLength * .62).add(current.origin);
        label.textContent = hintLabel(hint);
    }
    function applyFade() {
        const eased = fade * fade * (3 - 2 * fade);
        for (const { material, opacity } of faded) material.opacity = opacity * eased;
        root.scale.setScalar(GROW_FROM + (1 - GROW_FROM) * eased);
        root.visible = fade > 0 && !!current;
        label.style.opacity = triadLabels[0].element.style.opacity = triadLabels[1].element.style.opacity = triadLabels[2].element.style.opacity = String(eased);
    }
    // Zwraca true, gdy potrzebny nowy rysunek (zmiana podpowiedzi, pozy albo trwająca animacja).
    function update(hint: MotionHint | null, robot: URDFRobot, now = performance.now()) {
        const geometry = hint ? hintGeometry(robot, hint) : null;
        const visible = !!(hint && geometry);
        const nextKey = visible ? `${hintLabel(hint!)}/${geometry!.origin.toArray().map(v => v.toFixed(4))}/${geometry!.axis.toArray().map(v => v.toFixed(4))}/${geometry!.frame.toArray().map(v => v.toFixed(4))}` : "";
        let changed = false;
        if (visible && nextKey !== key) { place(hint!, geometry!); changed = true; }
        key = nextKey;
        if (visible !== shown) { shown = visible; fadeFrom = fade; fadeTarget = visible ? 1 : 0; fadeStarted = now; changed = true; }
        if (fade !== fadeTarget) {
            const duration = fadeTarget ? FADE_IN_MS : FADE_OUT_MS;
            fade = reducedMotion.matches ? fadeTarget : THREE.MathUtils.clamp(fadeFrom + (fadeTarget - fadeFrom) * Math.min(1, (now - fadeStarted) / duration), 0, 1);
            if (fade === 0 && fadeTarget === 0) { current = null; label.hidden = true; triadLabels.forEach(({ element }) => { element.hidden = true; }); }
            applyFade(); changed = true;
        }
        return changed;
    }
    function project(camera: THREE.Camera, width: number, height: number) {
        const put = (element: HTMLElement, world: THREE.Vector3) => {
            const p = world.clone().project(camera);
            element.style.left = `${(p.x + 1) * width / 2}px`; element.style.top = `${(1 - p.y) * height / 2}px`;
            element.hidden = p.z < -1 || p.z > 1 || Math.abs(p.x) > 1 || Math.abs(p.y) > 1;
        };
        if (!current || fade === 0) return;
        put(label, tip);
        // W prawej części kadru etykieta po lewej stronie punktu, żeby nie wychodziła poza widok.
        label.classList.toggle("flip", tip.clone().project(camera).x > .35);
        for (const { element, point } of triadLabels) {
            if (!triad.visible) { element.hidden = true; continue; }
            put(element, point.clone().multiplyScalar(root.scale.x).applyQuaternion(current.frame).add(current.origin));
        }
    }
    function snapshot() {
        return current && shown ? { ...current.hint, label: hintLabel(current.hint), origin: current.origin.toArray(), axis: current.axis.toArray(), rotation: current.rotation, tip: tip.toArray(), opacity: +fade.toFixed(3) } : null;
    }
    return { update, project, snapshot };
}
