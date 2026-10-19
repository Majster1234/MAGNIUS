import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";
import { createLimitGlows } from "./previewLimitGlow";
import type { LimitIndicator } from "./previewLimits";

export type CameraView = "iso" | "front" | "side" | "top" | "fit";
export type PoseFrame = { angles: number[]; valid: boolean[]; active: number; limits: LimitIndicator[][]; planning: boolean; plannedAngles: (number | null)[]; motionAngles?: number[] | null };
export function createPreviewScene(host: HTMLElement, xml: string, frame: () => PoseFrame) {
    const document = new DOMParser().parseFromString(xml, "application/xml");
    if (document.querySelector("parsererror, mesh, texture") || document.documentElement.tagName !== "robot")
        throw Error("Nieprawidłowy model URDF");
    const robot = new URDFLoader().parse(xml);
    for (let i = 1; i <= 6; i++) if (!robot.joints[`joint_${i}`]) throw Error("Brak sześciu osi w URDF");
    const twin = robot.clone(true) as typeof robot;
    const motion = robot.clone(true) as typeof robot;
    const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "low-power" });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
    renderer.setClearColor(0x131b22); host.append(renderer.domElement);
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(36, 1, .01, 30); camera.up.set(0, 0, 1);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.minDistance = .3; controls.maxDistance = 5; controls.enableDamping = false;
    const grid = new THREE.GridHelper(2, 20, 0x40574f, 0x25313a); grid.rotation.x = Math.PI / 2; scene.add(grid);
    scene.add(new THREE.HemisphereLight(0xdbe8f0, 0x364940, 2));
    const light = new THREE.DirectionalLight(0xffffff, 2); light.position.set(1, -2, 3); scene.add(light);
    scene.add(robot, twin, motion); twin.visible = false; motion.visible = false;
    const materials: { axis: number; material: THREE.MeshStandardMaterial; mesh: THREE.Mesh }[] = [];
    function replaceMaterials(subject: THREE.Object3D, planning = false, playback = false) { subject.traverse(object => {
        if (!(object instanceof THREE.Mesh)) return;
        let ancestor: THREE.Object3D | null = object, axis = 0;
        while (ancestor && !axis) {
            const match = /^(?:link|joint)_([1-6])$/.exec(ancestor.name); if (match) axis = Number(match[1]);
            ancestor = ancestor.parent;
        }
        const material = new THREE.MeshStandardMaterial({ color: playback ? 0xf3a347 : planning ? 0x4a92bc : axis ? 0x8096a3 : 0x4b616c, roughness: .8,
            transparent: planning, opacity: playback ? .55 : planning ? .3 : 1, depthWrite: !planning,
            polygonOffset: planning, polygonOffsetFactor: -1, polygonOffsetUnits: -1 });
        const old = Array.isArray(object.material) ? object.material : [object.material]; old.forEach(m => m.dispose());
        object.material = material; if (!planning) materials.push({ axis, material, mesh: object });
    }); }
    replaceMaterials(robot); replaceMaterials(twin, true); replaceMaterials(motion, true, true);
    twin.renderOrder = 1; motion.renderOrder = 2;
    const labels: { object: THREE.Object3D; element: HTMLSpanElement; point: THREE.Vector3 }[] = [];
    for (const [name, object] of [...Array.from({ length: 6 }, (_, i) => [`J${i + 1}`, robot.joints[`joint_${i + 1}`]] as const), ["TCP", robot.joints.tool_fixed] as const]) {
        if (!object) continue;
        const element = window.document.createElement("span"); element.className = "preview-joint-label";
        element.textContent = name; host.append(element); labels.push({ object, element, point: new THREE.Vector3() });
    }
    robot.joints.tool_fixed?.add(new THREE.AxesHelper(.06));
    const limitGlows = createLimitGlows(robot, host);
    let width = 1, height = 1, dirty = true, disposed = false, lastRender = -Infinity;
    const angles = Array(6).fill(NaN) as number[], twinAngles = Array(6).fill(NaN) as number[];
    const motionAngles = Array(6).fill(NaN) as number[];
    let signature = "", currentView: CameraView = "iso", initialPoseFit = false;
    function view(next: CameraView) {
        if (next !== "fit") currentView = next;
        scene.updateMatrixWorld(true);
        // Poświaty i znaczniki nie zmieniają kadru właściwego modelu.
        const box = new THREE.Box3(); materials.forEach(({ mesh }) => box.expandByObject(mesh));
        if (twin.visible) box.expandByObject(twin);
        if (motion.visible) box.expandByObject(motion);
        const center = box.getCenter(new THREE.Vector3());
        const radius = Math.max(.2, box.getSize(new THREE.Vector3()).length() * .6);
        const directions = { iso: [1.4, -1.8, 1.2], front: [0, -1, .05], side: [1, 0, .05], top: [.001, 0, 1] };
        camera.position.copy(center).add(new THREE.Vector3(...directions[currentView as Exclude<CameraView, "fit">]).normalize().multiplyScalar(radius / Math.tan(THREE.MathUtils.degToRad(18))));
        controls.target.copy(center); controls.update(); dirty = true;
    }
    const resize = new ResizeObserver(() => {
        width = Math.max(1, host.clientWidth); height = Math.max(1, host.clientHeight);
        renderer.setSize(width, height); camera.aspect = width / height; camera.updateProjectionMatrix(); dirty = true;
    }); resize.observe(host); controls.addEventListener("change", () => { dirty = true; });
    view("iso");
    let request = 0;
    function draw(now: number) {
        if (disposed) return;
        request = requestAnimationFrame(draw);
        if (window.document.hidden || now - lastRender < 1000 / 30) return;
        const state = frame();
        state.angles.forEach((angle, i) => {
            if (angle !== angles[i]) { robot.setJointValue(`joint_${i + 1}`, angle); angles[i] = angle; dirty = true; }
        });
        if (twin.visible !== state.planning) { twin.visible = state.planning; dirty = true; }
        if (state.planning) state.plannedAngles.forEach((angle, i) => {
            const value = angle === null ? state.angles[i] : angle;
            if (Number.isFinite(value) && value !== twinAngles[i]) {
                twin.setJointValue(`joint_${i + 1}`, value); twinAngles[i] = value; dirty = true;
            }
        });
        const visibleMotion = state.planning && !!state.motionAngles;
        if (motion.visible !== visibleMotion) { motion.visible = visibleMotion; dirty = true; }
        if (visibleMotion) state.motionAngles!.forEach((value, i) => {
            if (Number.isFinite(value) && value !== motionAngles[i]) {
                motion.setJointValue(`joint_${i + 1}`, value); motionAngles[i] = value; dirty = true;
            }
        });
        if (!initialPoseFit && (state.valid.some(Boolean) || state.planning)) { view("iso"); initialPoseFit = true; }
        const nextSignature = `${state.valid.join()}/${state.active}`;
        if (signature !== nextSignature) {
            materials.forEach(({ axis, material }) => material.color.setHex(!axis ? 0x4b616c : !state.valid[axis - 1] ? 0x53606a : axis === state.active ? 0x77c8a7 : 0x8096a3));
            signature = nextSignature; dirty = true;
        }
        if (limitGlows.update(state.limits, now)) dirty = true;
        if (!dirty) return;
        scene.updateMatrixWorld(true); renderer.render(scene, camera);
        limitGlows.project(camera, width, height);
        labels.forEach(({ object, element, point }) => {
            object.getWorldPosition(point).project(camera);
            element.style.left = `${(point.x + 1) * width / 2}px`; element.style.top = `${(1 - point.y) * height / 2}px`;
            element.hidden = point.z < -1 || point.z > 1 || Math.abs(point.x) > 1 || Math.abs(point.y) > 1;
        });
        // Bieżący stan elementu, bez historii; używany także w teście zgodności renderowanej pozy.
        renderer.domElement.dataset.jointAngles = JSON.stringify(angles);
        renderer.domElement.dataset.plannedJointAngles = state.planning ? JSON.stringify(twinAngles) : "";
        renderer.domElement.dataset.planningOpacity = twin.visible ? ".3" : "";
        renderer.domElement.dataset.motionJointAngles = motion.visible ? JSON.stringify(motionAngles) : "";
        renderer.domElement.dataset.motionOpacity = motion.visible ? ".55" : "";
        dirty = false; lastRender = now;
    }
    const restore = (event: Event) => { event.preventDefault(); dirty = true; };
    renderer.domElement.addEventListener("webglcontextlost", restore);
    renderer.domElement.addEventListener("webglcontextrestored", restore);
    request = requestAnimationFrame(draw);
    return { view, dispose() {
        disposed = true; cancelAnimationFrame(request); resize.disconnect(); controls.dispose();
        renderer.domElement.removeEventListener("webglcontextlost", restore);
        renderer.domElement.removeEventListener("webglcontextrestored", restore);
        const geometries = new Set<THREE.BufferGeometry>(), allMaterials = new Set<THREE.Material>();
        scene.traverse(object => {
            const renderable = object as THREE.Mesh;
            if (renderable.geometry) geometries.add(renderable.geometry);
            if (renderable.material) (Array.isArray(renderable.material) ? renderable.material : [renderable.material]).forEach(m => allMaterials.add(m));
        });
        geometries.forEach(g => g.dispose()); allMaterials.forEach(m => m.dispose());
        limitGlows.dispose(); renderer.dispose(); renderer.forceContextLoss(); host.replaceChildren();
    } };
}
