import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

// Podgląd pojedynczego elementu (tryb modułowy) w układzie jego pliku: wskazanie walca osi kliknięciem i nakładka
// osi — linia, okrąg promienia dopasowania, strzałka zwrotu (dodatni obrót wg reguły prawej dłoni).
export type ElementPick = {point: THREE.Vector3; face: number; geometry: THREE.BufferGeometry};
export type ElementOverlay = {point: THREE.Vector3; direction: THREE.Vector3; radius: number} | null;
const overlayColor = 0xffd166;

export function createElementScene(host: HTMLElement) {
    const renderer = new THREE.WebGLRenderer({antialias: true, powerPreference: "low-power"});
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5)); renderer.setClearColor(0x131b22); host.append(renderer.domElement);
    const world = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(36, 1, .0001, 100); camera.up.set(0, 0, 1);
    const controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = false;
    world.add(new THREE.HemisphereLight(0xdbe8f0, 0x364940, 2));
    const light = new THREE.DirectionalLight(0xffffff, 2); light.position.set(1, -2, 3); world.add(light);
    const axes = new THREE.AxesHelper(1); world.add(axes);
    let grid: THREE.GridHelper | null = null;
    const surface = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshStandardMaterial({color: 0x8fa6b8, roughness: .75, metalness: .05, side: THREE.DoubleSide}));
    const safety = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({color: 0xf0a85a, wireframe: true, transparent: true, opacity: .28, depthWrite: false}));
    surface.visible = safety.visible = false; safety.renderOrder = 3; world.add(surface, safety);
    const lineMaterial = new THREE.LineBasicMaterial({color: overlayColor, depthTest: false, transparent: true});
    const axisLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), lineMaterial);
    const ring = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(Array.from({length: 64}, (_, i) => new THREE.Vector3(Math.cos(i / 64 * 2 * Math.PI), Math.sin(i / 64 * 2 * Math.PI), 0))), lineMaterial);
    const head = new THREE.Mesh(new THREE.ConeGeometry(.5, 1.6, 20).translate(0, .8, 0), new THREE.MeshBasicMaterial({color: overlayColor, depthTest: false, transparent: true}));
    for (const object of [axisLine, ring, head]) { object.renderOrder = 8; object.visible = false; world.add(object); }
    let size = .1, dirty = true, disposed = false, request = 0, width = 1, height = 1;
    let pickHandler: ((pick: ElementPick) => void) | null = null, down: {x: number; y: number} | null = null;
    const raycaster = new THREE.Raycaster();
    renderer.domElement.addEventListener("pointerdown", event => { down = {x: event.clientX, y: event.clientY}; });
    renderer.domElement.addEventListener("pointerup", event => {
        // Klik bez przeciągnięcia (obrót kamerą nie wskazuje).
        if (!pickHandler || !surface.visible || !down || Math.hypot(event.clientX - down.x, event.clientY - down.y) > 4) return;
        const rect = renderer.domElement.getBoundingClientRect();
        raycaster.setFromCamera(new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1, -(event.clientY - rect.top) / rect.height * 2 + 1), camera);
        const hit = raycaster.intersectObject(surface, false)[0];
        if (hit && hit.faceIndex != null) pickHandler({point: hit.point.clone(), face: hit.faceIndex, geometry: surface.geometry});
    });
    function fit() {
        const box = new THREE.Box3().setFromObject(surface.visible ? surface : axes), center = box.getCenter(new THREE.Vector3());
        size = Math.max(.01, box.getSize(new THREE.Vector3()).length());
        const distance = size * .65 / Math.tan(THREE.MathUtils.degToRad(18));
        camera.near = Math.max(.00001, distance / 10000); camera.far = Math.max(10, distance * 20); camera.updateProjectionMatrix();
        controls.minDistance = size * .05; controls.maxDistance = distance * 5;
        camera.position.copy(center).add(new THREE.Vector3(1.4, -1.8, 1.2).normalize().multiplyScalar(distance));
        controls.target.copy(center); controls.update(); dirty = true;
    }
    function setElement(geometry: THREE.BufferGeometry | null, safetyGeometry?: THREE.BufferGeometry | null) {
        surface.geometry = geometry ?? new THREE.BufferGeometry(); surface.visible = !!geometry;
        safety.geometry = safetyGeometry ?? new THREE.BufferGeometry(); safety.userData.available = !!safetyGeometry;
        safety.visible = !!safetyGeometry && !!safety.userData.show;
        // Siatka i osie układu pliku w skali elementu (początek układu = początek pliku CAD).
        const box = geometry ? (geometry.boundingBox ?? new THREE.Box3().setFromBufferAttribute(geometry.getAttribute("position") as THREE.BufferAttribute)) : null;
        const extent = box ? Math.max(.01, box.getSize(new THREE.Vector3()).length()) : .1;
        if (grid) { world.remove(grid); grid.geometry.dispose(); (grid.material as THREE.Material).dispose(); }
        const step = 10 ** Math.floor(Math.log10(extent / 4));
        grid = new THREE.GridHelper(step * 20, 20, 0x40574f, 0x25313a); grid.rotation.x = Math.PI / 2; world.add(grid);
        axes.scale.setScalar(extent * .25);
        renderer.domElement.dataset.elementLoaded = geometry ? "1" : "";
        fit();
    }
    function setSafety(show: boolean) { safety.userData.show = show; safety.visible = show && !!safety.userData.available; dirty = true; }
    function setPick(handler: ((pick: ElementPick) => void) | null) {
        pickHandler = handler; renderer.domElement.style.cursor = handler ? "crosshair" : ""; renderer.domElement.dataset.picking = handler ? "1" : "";
    }
    function setOverlay(axis: ElementOverlay) {
        axisLine.visible = ring.visible = head.visible = !!axis;
        if (axis) {
            const direction = axis.direction.clone().normalize(), half = direction.clone().multiplyScalar(size * .45);
            axisLine.geometry.setFromPoints([axis.point.clone().sub(half), axis.point.clone().add(half)]);
            ring.position.copy(axis.point); ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), direction); ring.scale.setScalar(Math.max(axis.radius, size * .01) * 1.15);
            head.position.copy(axis.point).add(half); head.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction); head.scale.setScalar(size * .025);
        }
        dirty = true;
    }
    const resize = new ResizeObserver(() => {
        width = Math.max(1, host.clientWidth); height = Math.max(1, host.clientHeight);
        renderer.setSize(width, height); camera.aspect = width / height; camera.updateProjectionMatrix(); dirty = true;
        if (!disposed) renderer.render(world, camera);
    });
    resize.observe(host);
    controls.addEventListener("change", () => { dirty = true; });
    function draw() {
        if (disposed) return;
        request = requestAnimationFrame(draw);
        if (document.hidden || !dirty) return;
        renderer.render(world, camera); dirty = false;
    }
    request = requestAnimationFrame(draw);
    return {setElement, setSafety, setPick, setOverlay, fit, dispose() {
        disposed = true; cancelAnimationFrame(request); resize.disconnect(); controls.dispose();
        // Geometrie elementu należą do wspólnej pamięci podręcznej (meshGeometry) — nie są zwalniane.
        surface.geometry = new THREE.BufferGeometry(); safety.geometry = new THREE.BufferGeometry();
        world.traverse(object => {
            const mesh = object as THREE.Mesh;
            if (mesh.geometry) mesh.geometry.dispose();
            if (mesh.material) (Array.isArray(mesh.material) ? mesh.material : [mesh.material]).forEach(item => item.dispose());
        });
        renderer.dispose(); renderer.forceContextLoss(); host.replaceChildren();
    }};
}
