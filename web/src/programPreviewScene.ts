// Podgląd 3D pozycji programu: robot z prymitywów według geometrii aktywnej konfiguracji (jak configurationScene),
// ustawiony w kątach wybranej pozycji; znaczniki TCP wszystkich pozycji programu z etykietami.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { demoTranslations, jointAxes } from "./configurationData";
import type { Position } from "./programModel";

type Values = Record<string, unknown> | null;
const num = (v: unknown, fb: number) => typeof v === "number" && Number.isFinite(v) ? v : fb;
const rad = (d: number) => d * Math.PI / 180;

function chain(values: Values) {
    const joints = Array.from({ length: 6 }, (_, i) => ({
        xyz: [0, 1, 2].map(c => num(values?.[`joint.${i + 1}.${["origin_x", "origin_y", "origin_z"][c]}`], demoTranslations[i][c]) / 1000),
        rpy: ["origin_roll", "origin_pitch", "origin_yaw"].map(k => rad(num(values?.[`joint.${i + 1}.${k}`], 0))),
        axis: new THREE.Vector3(...jointAxes[i]).normalize(),
    }));
    const tool = { xyz: ["tcp_x", "tcp_y", "tcp_z"].map((k, c) => num(values?.[`tool.${k}`], c === 0 ? 80 : 0) / 1000),
        rpy: ["tcp_roll", "tcp_pitch", "tcp_yaw"].map(k => rad(num(values?.[`tool.${k}`], 0))) };
    return { joints, tool };
}
export function tcpOf(values: Values, deg: number[]): THREE.Vector3 {
    const { joints, tool } = chain(values); const m = new THREE.Matrix4(), t = new THREE.Matrix4();
    joints.forEach((j, i) => {
        m.multiply(t.makeTranslation(j.xyz[0], j.xyz[1], j.xyz[2])).multiply(t.makeRotationFromEuler(new THREE.Euler(j.rpy[0], j.rpy[1], j.rpy[2], "ZYX")))
         .multiply(t.makeRotationAxis(j.axis, rad(deg[i] ?? 0)));
    });
    m.multiply(t.makeTranslation(tool.xyz[0], tool.xyz[1], tool.xyz[2]));
    return new THREE.Vector3().setFromMatrixPosition(m);
}

export function createProgramScene(host: HTMLElement) {
    const renderer = new THREE.WebGLRenderer({ antialias: true }); renderer.setPixelRatio(Math.min(2, devicePixelRatio)); renderer.setClearColor(0x131b22);
    host.appendChild(renderer.domElement);
    const labels = document.createElement("div"); labels.className = "prg-scene-labels"; host.appendChild(labels);
    const scene = new THREE.Scene(), camera = new THREE.PerspectiveCamera(36, 1, .01, 20); camera.up.set(0, 0, 1); camera.position.set(1.05, -1, .75);
    const controls = new OrbitControls(camera, renderer.domElement); controls.target.set(.22, 0, .22); controls.update();
    scene.add(new THREE.HemisphereLight(0xdbe8f0, 0x364940, 2)); const light = new THREE.DirectionalLight(0xffffff, 2); light.position.set(1, -2, 3); scene.add(light);
    const grid = new THREE.GridHelper(2, 20, 0x40574f, 0x25313a); grid.rotation.x = Math.PI / 2; scene.add(grid);
    const linkMat = new THREE.MeshStandardMaterial({ color: 0x8096a3, metalness: .25, roughness: .55 }), jointMat = new THREE.MeshStandardMaterial({ color: 0x77c8a7, roughness: .5 });
    const markerMat = new THREE.MeshBasicMaterial({ color: 0x86a9c8 }), selMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const base = new THREE.Mesh(new THREE.CylinderGeometry(.09, .1, .1, 32), new THREE.MeshStandardMaterial({ color: 0x4b616c })); base.rotation.x = Math.PI / 2; base.position.z = .05; scene.add(base);
    let robot = new THREE.Group(), pivots: THREE.Group[] = [], axes: THREE.Vector3[] = [], values: Values = null;
    scene.add(robot);
    const disposables: { dispose(): void }[] = [];
    function cylinder(a: THREE.Vector3, b: THREE.Vector3, r: number, mat: THREE.Material) {
        const geo = new THREE.CylinderGeometry(r, r, Math.max(1e-4, a.distanceTo(b)), 20); disposables.push(geo);
        const mesh = new THREE.Mesh(geo, mat); mesh.position.copy(a).add(b).multiplyScalar(.5);
        mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), b.clone().sub(a).normalize()); return mesh;
    }
    function build(v: Values) {
        values = v; scene.remove(robot); robot = new THREE.Group(); scene.add(robot); pivots = []; axes = [];
        const { joints, tool } = chain(v); let parent: THREE.Object3D = robot;
        joints.forEach((j, i) => {
            const g = new THREE.Group(); g.position.set(j.xyz[0], j.xyz[1], j.xyz[2]); g.rotation.set(j.rpy[0], j.rpy[1], j.rpy[2], "ZYX"); parent.add(g);
            const pivot = new THREE.Group(); g.add(pivot); pivots.push(pivot); axes.push(j.axis);
            const jm = new THREE.Mesh(new THREE.CylinderGeometry(.032, .032, .075, 24), jointMat); jm.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), j.axis); pivot.add(jm);
            const next = i < 5 ? joints[i + 1].xyz : tool.xyz; pivot.add(cylinder(new THREE.Vector3(), new THREE.Vector3(...next), i < 3 ? .024 : .018, linkMat)); parent = pivot;
        });
        const t = new THREE.Group(); t.position.set(tool.xyz[0], tool.xyz[1], tool.xyz[2]); parent.add(t); t.add(new THREE.AxesHelper(.06));
    }
    let markers: { mesh: THREE.Mesh; label: HTMLSpanElement }[] = [];
    function setPositions(list: Position[], selected: string | null) {
        for (const m of markers) { scene.remove(m.mesh); m.label.remove(); } markers = [];
        for (const p of list) {
            const mesh = new THREE.Mesh(new THREE.SphereGeometry(p.name === selected ? .012 : .008, 16, 12), p.name === selected ? selMat : markerMat);
            mesh.position.copy(tcpOf(values, p.joints_deg)); scene.add(mesh);
            const label = document.createElement("span"); label.className = `prg-scene-label${p.name === selected ? " selected" : ""}`; label.textContent = p.name; labels.appendChild(label);
            markers.push({ mesh, label });
        }
    }
    function setPose(deg: number[] | null) { pivots.forEach((p, i) => { p.quaternion.setFromAxisAngle(axes[i], rad(deg?.[i] ?? 0)); }); }
    let frame = 0;
    const resize = () => { const w = host.clientWidth, h = host.clientHeight; if (!w || !h) return; renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix(); };
    const observer = new ResizeObserver(resize); observer.observe(host); resize();
    (function loop() {
        frame = requestAnimationFrame(loop); controls.update();
        const w = host.clientWidth, h = host.clientHeight;
        for (const m of markers) { const p = m.mesh.position.clone().project(camera); m.label.style.transform = `translate(${(p.x * .5 + .5) * w}px,${(-p.y * .5 + .5) * h}px)`; m.label.style.display = p.z < 1 ? "" : "none"; }
        renderer.render(scene, camera);
    })();
    build(null);
    return { setGeometry: build, setPositions, setPose,
        dispose() { cancelAnimationFrame(frame); observer.disconnect(); controls.dispose(); renderer.dispose(); disposables.forEach(d => d.dispose()); host.replaceChildren(); } };
}
