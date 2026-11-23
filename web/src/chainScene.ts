import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

// Podgląd łańcucha elementów (tryb modułowy): siatki w układzie podstawy (pozycja zerowa), osie modeli osi (linia,
// strzałka zwrotu, etykieta Jn), znaczniki wskazań spasowania. Wskazanie działa tylko na wskazanych siatkach
// (poprzednik / nowy element); bez trybu wskazywania klik wybiera element.
export type ChainItemKind = "base" | "structural" | "axis" | "unset" | "draft" | "staging";
export type ChainSceneItem = { key: string; geometry: THREE.BufferGeometry; matrix: THREE.Matrix4; kind: ChainItemKind; selected?: boolean; warn?: boolean };
export type ChainAxis = { point: THREE.Vector3; direction: THREE.Vector3; length: number; label: string; draft?: boolean; virtual?: boolean };
// Przeguby wirtualne (brakujące w modelu — z konfiguracji): walce przegubów, pręty między nimi i TCP.
export type ChainVirtual = { joints: { point: THREE.Vector3; direction: THREE.Vector3 }[]; segments: [THREE.Vector3, THREE.Vector3][]; tcp: THREE.Vector3 | null };
export type ChainMarker = { point: THREE.Vector3; direction: THREE.Vector3; radius: number; kind: "ring" | "disc" };
export type ChainPick = { key: string; face: number; geometry: THREE.BufferGeometry; matrix: THREE.Matrix4 };
const colors: Record<ChainItemKind, number> = {base: 0x6f7f88, structural: 0x8a97a3, axis: 0x5f8db3, unset: 0x7a7f86, draft: 0x52f2a0, staging: 0x52f2a0};
const axisColor = 0xffd166, draftColor = 0x52f2a0, virtualColor = 0xb48cff;

export function createChainScene(host: HTMLElement) {
    const renderer = new THREE.WebGLRenderer({antialias: true, powerPreference: "low-power"});
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5)); renderer.setClearColor(0x0b0f13); host.append(renderer.domElement);
    const world = new THREE.Scene(), camera = new THREE.PerspectiveCamera(36, 1, .001, 100); camera.up.set(0, 0, 1);
    const controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = false;
    world.add(new THREE.HemisphereLight(0xdbe8f0, 0x364940, 2));
    const light = new THREE.DirectionalLight(0xffffff, 2); light.position.set(1, -2, 3); world.add(light);
    let grid: THREE.GridHelper | null = null;
    const origin = new THREE.AxesHelper(.05); world.add(origin);
    const meshes = new THREE.Group(), overlay = new THREE.Group(); world.add(meshes, overlay);
    const labels = document.createElement("div"); labels.className = "chain-labels"; host.append(labels);
    let axisLabels: {span: HTMLSpanElement; point: THREE.Vector3}[] = [];
    let pickKeys: Set<string> | null = null, pickHandler: ((pick: ChainPick) => void) | null = null, selectHandler: ((key: string | null) => void) | null = null;
    let down: {x: number; y: number} | null = null, dirty = true, disposed = false, request = 0, width = 1, height = 1, fitted = false, size = .3;
    const raycaster = new THREE.Raycaster();
    renderer.domElement.addEventListener("pointerdown", event => { down = {x: event.clientX, y: event.clientY}; });
    renderer.domElement.addEventListener("pointerup", event => {
        if (!down || Math.hypot(event.clientX - down.x, event.clientY - down.y) > 4) return;
        const rect = renderer.domElement.getBoundingClientRect();
        raycaster.setFromCamera(new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1, -(event.clientY - rect.top) / rect.height * 2 + 1), camera);
        const candidates = meshes.children.filter(object => object.visible && (!pickKeys || pickKeys.has(object.userData.key))) as THREE.Mesh[];
        const hit = raycaster.intersectObjects(candidates, false)[0];
        const mesh = hit?.object as THREE.Mesh | undefined;
        if (pickHandler && pickKeys) { if (mesh && hit.faceIndex != null) pickHandler({key: mesh.userData.key, face: hit.faceIndex, geometry: mesh.geometry, matrix: mesh.matrix.clone()}); }
        else selectHandler?.(mesh ? mesh.userData.key : null);
    });
    function fit() {
        const box = new THREE.Box3();
        meshes.children.forEach(object => box.expandByObject(object));
        if (box.isEmpty()) box.setFromCenterAndSize(new THREE.Vector3(), new THREE.Vector3(.3, .3, .3));
        const center = box.getCenter(new THREE.Vector3()), radius = Math.max(.05, box.getSize(new THREE.Vector3()).length() * .6);
        size = radius * 2;
        const distance = radius / Math.tan(THREE.MathUtils.degToRad(18));
        camera.near = Math.max(.0001, distance / 1000); camera.far = Math.max(20, distance * 20); camera.updateProjectionMatrix();
        controls.minDistance = radius * .05; controls.maxDistance = distance * 6;
        camera.position.copy(center).add(new THREE.Vector3(1.4, -1.8, 1.0).normalize().multiplyScalar(distance));
        controls.target.copy(center); controls.update();
        if (grid) { world.remove(grid); grid.geometry.dispose(); (grid.material as THREE.Material).dispose(); }
        const step = 10 ** Math.floor(Math.log10(size / 4));
        grid = new THREE.GridHelper(step * 40, 40, 0x40574f, 0x1f2a31); grid.rotation.x = Math.PI / 2; grid.position.z = Math.min(0, box.min.z); world.add(grid);
        origin.scale.setScalar(size * .15); dirty = true;
    }
    function setItems(items: ChainSceneItem[], refit = false) {
        const previous = meshes.children.length;
        for (const object of [...meshes.children]) { meshes.remove(object); ((object as THREE.Mesh).material as THREE.Material).dispose(); }
        for (const item of items) {
            const transparent = item.kind === "draft" || item.kind === "staging";
            const material = new THREE.MeshStandardMaterial({color: colors[item.kind], roughness: .75, metalness: .05, side: THREE.DoubleSide,
                transparent, opacity: transparent ? .55 : 1, emissive: item.selected ? 0x7a4d00 : item.warn ? 0x5a1a10 : 0x000000});
            const mesh = new THREE.Mesh(item.geometry, material);
            mesh.matrixAutoUpdate = false; mesh.matrix.copy(item.matrix); mesh.userData.key = item.key;
            meshes.add(mesh);
        }
        applyPickHighlight();
        renderer.domElement.dataset.chainItems = String(items.length);
        if (refit || !fitted || (previous === 0 && items.length)) { fit(); fitted = items.length > 0; }
        dirty = true;
    }
    function applyPickHighlight() {
        for (const object of meshes.children) {
            const material = (object as THREE.Mesh).material as THREE.MeshStandardMaterial;
            const allowed = !pickKeys || pickKeys.has(object.userData.key);
            if (pickKeys) { material.transparent = true; material.opacity = allowed ? Math.max(material.opacity, .9) : .18; if (allowed) material.emissive.setHex(0x14385a); }
        }
        dirty = true;
    }
    function clearOverlay() {
        for (const object of [...overlay.children]) { overlay.remove(object); object.traverse(child => { const mesh = child as THREE.Mesh; mesh.geometry?.dispose(); (mesh.material as THREE.Material | undefined)?.dispose?.(); }); }
        labels.replaceChildren(); axisLabels = [];
    }
    function setOverlay(axes: ChainAxis[], markers: ChainMarker[], virtual: ChainVirtual | null = null) {
        clearOverlay();
        if (virtual) {
            const scale = Math.max(.01, size * .022), material = new THREE.MeshStandardMaterial({color: virtualColor, transparent: true, opacity: .5, roughness: .6, depthWrite: false});
            for (const joint of virtual.joints) {
                const body = new THREE.Mesh(new THREE.CylinderGeometry(scale, scale, scale * 2.6, 24), material.clone());
                body.position.copy(joint.point); body.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), joint.direction.clone().normalize()); overlay.add(body);
            }
            for (const [from, to] of virtual.segments) {
                const length = from.distanceTo(to);
                if (length < 1e-6) continue;
                const rod = new THREE.Mesh(new THREE.CylinderGeometry(scale * .45, scale * .45, length, 12), material.clone());
                rod.position.copy(from).add(to).multiplyScalar(.5); rod.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), to.clone().sub(from).normalize()); overlay.add(rod);
            }
            if (virtual.tcp) { const tcp = new THREE.Mesh(new THREE.SphereGeometry(scale * .6, 16, 12), new THREE.MeshBasicMaterial({color: draftColor})); tcp.position.copy(virtual.tcp); overlay.add(tcp);
                const span = document.createElement("span"); span.className = "chain-label tcp"; span.textContent = "TCP"; labels.append(span); axisLabels.push({span, point: virtual.tcp.clone()}); }
            material.dispose();
        }
        for (const axis of axes) {
            const color = axis.virtual ? virtualColor : axis.draft ? draftColor : axisColor, material = new THREE.LineBasicMaterial({color, depthTest: false, transparent: true});
            const direction = axis.direction.clone().normalize(), half = direction.clone().multiplyScalar(axis.length / 2);
            const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints([axis.point.clone().sub(half), axis.point.clone().add(half)]), material);
            const head = new THREE.Mesh(new THREE.ConeGeometry(axis.length * .03, axis.length * .09, 16).translate(0, axis.length * .045, 0), new THREE.MeshBasicMaterial({color, depthTest: false, transparent: true}));
            head.position.copy(axis.point).add(half); head.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction);
            // Łukowa strzałka dodatniego obrotu (reguła prawej dłoni: patrząc od grotu — przeciwnie do wskazówek zegara).
            const u = new THREE.Vector3().crossVectors(direction, Math.abs(direction.x) < .9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0)).normalize();
            const w = new THREE.Vector3().crossVectors(direction, u), radius = axis.length * .09, center = axis.point.clone().addScaledVector(direction, axis.length * .32);
            const arc = Array.from({length: 49}, (_, i) => { const angle = i / 48 * Math.PI * 1.5; return center.clone().addScaledVector(u, radius * Math.cos(angle)).addScaledVector(w, radius * Math.sin(angle)); });
            const arcLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(arc), material);
            const end = Math.PI * 1.5, tangent = u.clone().multiplyScalar(-Math.sin(end)).addScaledVector(w, Math.cos(end)).normalize();
            const arcHead = new THREE.Mesh(new THREE.ConeGeometry(radius * .32, radius * .8, 12).translate(0, radius * .4, 0), new THREE.MeshBasicMaterial({color, depthTest: false, transparent: true}));
            arcHead.position.copy(arc[48]); arcHead.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), tangent);
            for (const object of [line, head, arcLine, arcHead]) { object.renderOrder = 8; overlay.add(object); }
            const span = document.createElement("span"); span.className = `chain-label${axis.draft ? " draft" : ""}${axis.virtual ? " virtual" : ""}`; span.textContent = axis.label; labels.append(span);
            axisLabels.push({span, point: axis.point.clone().add(half.clone().multiplyScalar(1.15))});
        }
        for (const marker of markers) {
            const material = new THREE.MeshBasicMaterial({color: 0xffb020, depthTest: false, transparent: true, opacity: marker.kind === "disc" ? .45 : 1, side: THREE.DoubleSide});
            const geometry = marker.kind === "disc" ? new THREE.CircleGeometry(1, 40) : new THREE.RingGeometry(.92, 1.08, 48);
            const mesh = new THREE.Mesh(geometry, material); mesh.renderOrder = 9;
            mesh.position.copy(marker.point); mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), marker.direction.clone().normalize());
            mesh.scale.setScalar(Math.max(marker.radius, size * .01)); overlay.add(mesh);
        }
        dirty = true;
    }
    function setPick(keys: string[] | null, handler: ((pick: ChainPick) => void) | null) {
        pickKeys = keys ? new Set(keys) : null; pickHandler = handler;
        renderer.domElement.style.cursor = keys ? "crosshair" : ""; renderer.domElement.dataset.picking = keys ? keys.join(",") : "";
    }
    function onSelect(handler: ((key: string | null) => void) | null) { selectHandler = handler; }
    const resize = new ResizeObserver(() => {
        width = Math.max(1, host.clientWidth); height = Math.max(1, host.clientHeight);
        renderer.setSize(width, height); camera.aspect = width / height; camera.updateProjectionMatrix(); dirty = true;
        if (!disposed) renderer.render(world, camera);
    });
    resize.observe(host);
    controls.addEventListener("change", () => { dirty = true; });
    const projected = new THREE.Vector3();
    function draw() {
        if (disposed) return;
        request = requestAnimationFrame(draw);
        if (document.hidden || !dirty) return;
        renderer.render(world, camera);
        for (const {span, point} of axisLabels) {
            projected.copy(point).project(camera);
            span.hidden = projected.z > 1 || Math.abs(projected.x) > 1 || Math.abs(projected.y) > 1;
            span.style.left = `${(projected.x + 1) * width / 2}px`; span.style.top = `${(1 - projected.y) * height / 2}px`;
        }
        dirty = false;
    }
    request = requestAnimationFrame(draw);
    return {setItems, setOverlay, setPick, onSelect, fit, dispose() {
        disposed = true; cancelAnimationFrame(request); resize.disconnect(); controls.dispose(); clearOverlay();
        // Geometrie siatek należą do wspólnej pamięci podręcznej (meshGeometry) — zwalniane są tylko materiały.
        for (const object of meshes.children) ((object as THREE.Mesh).material as THREE.Material).dispose();
        if (grid) { grid.geometry.dispose(); (grid.material as THREE.Material).dispose(); }
        renderer.dispose(); renderer.forceContextLoss(); host.replaceChildren();
    }};
}
