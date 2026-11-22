import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { demoTranslations, jointAxes, originKeys, tcpKeys, type Values } from "./configurationData";
import type { LoadedLinkMesh } from "./robotModel";
type Input = {values: Values; view: "geometry" | "limits"; selected: number};
// Wskazanie na siatce modelu: punkt w układzie bazy, trójkąt geometrii i przekształcenie złożenie → baza.
export type ModelPick = {point: THREE.Vector3; face: number; geometry: THREE.BufferGeometry; matrix: THREE.Matrix4; name: string};
export type PickOverlay = {axis?: {point: THREE.Vector3; direction: THREE.Vector3; radius: number} | null; point?: THREE.Vector3 | null};
// Odcienie członów (podstawa … narzędzie) — rozróżnienie przypisania siatek na podglądzie.
const linkColors = [0x6f7f88, 0x8aa0ad, 0x93a89a, 0xa7a08c, 0x9a92ad, 0x8fa6b8, 0xa89090, 0xc2b07a];
const rad = THREE.MathUtils.degToRad;
const z = new THREE.Vector3(0, 0, 1), y = new THREE.Vector3(0, 1, 0);
export function createConfigurationScene(host: HTMLElement) {
    const renderer = new THREE.WebGLRenderer({antialias: true, powerPreference: "low-power"});
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5)); renderer.setClearColor(0x131b22); host.append(renderer.domElement);
    const world = new THREE.Scene(), root = new THREE.Group(); world.add(root);
    const camera = new THREE.PerspectiveCamera(36, 1, .001, 100); camera.up.set(0, 0, 1);
    const controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = false;
    const grid = new THREE.GridHelper(2, 20, 0x40574f, 0x25313a); grid.rotation.x = Math.PI / 2; world.add(grid);
    world.add(new THREE.HemisphereLight(0xdbe8f0, 0x364940, 2));
    const light = new THREE.DirectionalLight(0xffffff, 2); light.position.set(1, -2, 3); world.add(light);
    const material = new THREE.MeshStandardMaterial({color:0x8096a3, roughness:.8});
    const jointGeometry = new THREE.CylinderGeometry(.035,.035,.07,24), boxGeometry = new THREE.BoxGeometry(1,1,1);
    const baseGeometry = new THREE.CylinderGeometry(.09,.09,.1,32), base = new THREE.Mesh(baseGeometry, material);
    base.rotation.x = Math.PI / 2; base.position.z = .05; root.add(base);
    let parent: THREE.Object3D = root;
    const joints = Array.from({length:6}, (_, i) => {
        const origin = new THREE.Group(); parent.add(origin); parent = origin;
        const body = new THREE.Mesh(jointGeometry, material.clone()); origin.add(body);
        const axis = new THREE.Vector3(...jointAxes[i]); body.quaternion.setFromUnitVectors(y, axis);
        const link = new THREE.Mesh(boxGeometry, material); origin.add(link);
        const axes = new THREE.AxesHelper(.08); origin.add(axes);
        const range = new THREE.Group(); range.quaternion.setFromUnitVectors(z, axis); origin.add(range);
        const arc = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color:0xc9a96c, transparent:true, opacity:.65}));
        arc.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(129*3),3)); range.add(arc);
        const ends = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color:0xe0c994}));
        ends.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(12),3)); range.add(ends);
        const label = document.createElement('span'); label.className = 'preview-joint-label configuration-bubble'; host.append(label);
        return {origin, body, link, axes, range, arc, ends, label, point:new THREE.Vector3()};
    });
    const tcp = new THREE.Group(); parent.add(tcp); tcp.add(new THREE.AxesHelper(.07));
    const tcpLabel = document.createElement('span'); tcpLabel.className='preview-joint-label'; tcpLabel.textContent='TCP'; host.append(tcpLabel);
    // Model 3D z CAD (układ złożenia → baza) i siatki bezpieczeństwa; bryły zastępcze można ukryć.
    const model = new THREE.Group(); world.add(model);
    const safetyMaterial = new THREE.MeshBasicMaterial({color:0xf0a85a, wireframe:true, transparent:true, opacity:.28, depthWrite:false});
    const surfaces: THREE.Mesh[] = [], safeties: THREE.Mesh[] = [];
    let display = {safety:false, primitives:true};
    const primitives = [base, ...joints.flatMap(joint => [joint.body, joint.link])];
    // Nakładka wskazanej osi (linia przez punkt, okrąg promienia dopasowania) i punktu TCP.
    const overlay = new THREE.Group(); overlay.renderOrder = 8; world.add(overlay);
    const overlayMaterial = new THREE.LineBasicMaterial({color:0xffd166, depthTest:false, transparent:true});
    const axisLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), overlayMaterial);
    const ring = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(Array.from({length:64}, (_, i) => new THREE.Vector3(Math.cos(i/64*2*Math.PI), Math.sin(i/64*2*Math.PI), 0))), overlayMaterial);
    const marker = new THREE.Mesh(new THREE.SphereGeometry(1, 16, 12), new THREE.MeshBasicMaterial({color:0xffd166, depthTest:false}));
    for (const object of [axisLine, ring, marker]) { object.renderOrder = 8; object.visible = false; overlay.add(object); }
    let pickHandler: ((pick: ModelPick) => void) | null = null, down: {x:number; y:number} | null = null;
    const raycaster = new THREE.Raycaster();
    function pointer(event: PointerEvent) {
        const rect = renderer.domElement.getBoundingClientRect();
        return new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1, -(event.clientY - rect.top) / rect.height * 2 + 1);
    }
    renderer.domElement.addEventListener('pointerdown', event => { down = {x:event.clientX, y:event.clientY}; });
    renderer.domElement.addEventListener('pointerup', event => {
        // Klik bez przeciągnięcia (obrót kamerą nie wskazuje).
        if (!pickHandler || !down || Math.hypot(event.clientX - down.x, event.clientY - down.y) > 4) return;
        raycaster.setFromCamera(pointer(event), camera);
        const hit = raycaster.intersectObjects(surfaces.filter(mesh => mesh.visible), false)[0];
        if (!hit || hit.faceIndex == null) return;
        const mesh = hit.object as THREE.Mesh;
        pickHandler({point: hit.point.clone(), face: hit.faceIndex, geometry: mesh.geometry, matrix: model.matrix.clone(), name: String(mesh.userData.name ?? '')});
    });
    let dirty = true, width=1, height=1, disposed=false, first=true, last=-Infinity, request=0;
    function fit() {
        root.updateMatrixWorld(true); model.updateMatrixWorld(true);
        const box = new THREE.Box3().setFromObject(root); if (surfaces.length) box.expandByObject(model);
        const center = box.getCenter(new THREE.Vector3());
        const radius=Math.max(.15,box.getSize(new THREE.Vector3()).length()*.65), distance=radius/Math.tan(rad(18));
        camera.far=Math.max(30,distance*20); camera.near=Math.max(.0001,distance/10000); camera.updateProjectionMatrix();
        controls.minDistance=Math.max(.01,radius*.1); controls.maxDistance=Math.max(5,distance*5);
        camera.position.copy(center).add(new THREE.Vector3(1.4,-1.8,1.2).normalize().multiplyScalar(distance));
        controls.target.copy(center); controls.update(); dirty=true;
    }
    function update({values, view, selected}: Input) {
        const translations = joints.map((_, i) => originKeys.slice(0,3).map((key,c)=>(Number(values[`joint.${i+1}.${key}`] ?? demoTranslations[i][c]))/1000));
        const tool = tcpKeys.slice(0,3).map((key,c)=>Number(values[`tool.${key}`] ?? (c===0?80:0))/1000);
        joints.forEach((joint,i)=>{
            const prefix=`joint.${i+1}.`;
            joint.origin.position.set(...translations[i] as [number,number,number]);
            const rpy=originKeys.slice(3).map(key=>rad(Number(values[prefix+key] ?? 0)));
            joint.origin.rotation.set(rpy[0],rpy[1],rpy[2],'ZYX');
            const end=new THREE.Vector3(...(i<5?translations[i+1]:tool));
            const length=end.length(); joint.link.visible=length>1e-9 && (display.primitives || !surfaces.length);
            joint.link.position.copy(end).multiplyScalar(.5); joint.link.scale.set(.045,.045,Math.max(length,.001));
            if(length>1e-9)joint.link.quaternion.setFromUnitVectors(z,end.normalize());
            joint.axes.visible=view==='geometry'; joint.range.visible=view==='limits';
            joint.body.material.color.setHex(selected===i+1?0x77c8a7:0x8096a3);
            const lower=values[prefix+'lower'], upper=values[prefix+'upper'], ratio=values[prefix+'gear_ratio'];
            const known=typeof lower==='number' && typeof upper==='number' && lower<upper;
            joint.arc.visible=joint.ends.visible=known;
            if(known){
                const lo=rad(lower), span=Math.min(2*Math.PI,rad(upper-lower));
                const points=joint.arc.geometry.getAttribute('position') as THREE.BufferAttribute;
                for(let n=0;n<=128;n++){const angle=lo+span*n/128;points.setXYZ(n,.11*Math.cos(angle),.11*Math.sin(angle),0);}
                points.needsUpdate=true; joint.arc.geometry.computeBoundingSphere();
                const ends=joint.ends.geometry.getAttribute('position') as THREE.BufferAttribute;
                [lo,rad(upper)].forEach((angle,c)=>{ends.setXYZ(c*2,.09*Math.cos(angle),.09*Math.sin(angle),0);ends.setXYZ(c*2+1,.13*Math.cos(angle),.13*Math.sin(angle),0);});
                ends.needsUpdate=true; joint.ends.geometry.computeBoundingSphere();
            }
            joint.label.textContent=view==='geometry'?`J${i+1}`:`J${i+1} · ${ratio===null?'przełożenie —':`${ratio}:1`} · ${known?`${lower}°…${upper}°${upper-lower>360?` (${Number(((upper-lower)/360).toFixed(2))} obr.)`:''}`:'limity —'}`;
            joint.label.classList.toggle('selected',selected===i+1);
        });
        tcp.position.set(...tool as [number,number,number]);
        const angles=tcpKeys.slice(3).map(key=>rad(Number(values[`tool.${key}`]??0)));tcp.rotation.set(angles[0],angles[1],angles[2],'ZYX');
        tcp.children[0].visible=view==='geometry';
        root.updateMatrixWorld(true);
        renderer.domElement.dataset.jointOrigins=JSON.stringify(joints.map(joint=>joint.origin.getWorldPosition(new THREE.Vector3()).toArray()));
        renderer.domElement.dataset.view=view;
        if(first){fit();first=false;}dirty=true;
    }
    // Rysunek od razu po zmianie rozmiaru (kanwa jest wtedy czyszczona) — model nie znika podczas animacji menu.
    const resize=new ResizeObserver(()=>{width=Math.max(1,host.clientWidth);height=Math.max(1,host.clientHeight);renderer.setSize(width,height);camera.aspect=width/height;camera.updateProjectionMatrix();dirty=true;if(!disposed){world.updateMatrixWorld(true);renderer.render(world,camera);}});resize.observe(host);
    controls.addEventListener('change',()=>{dirty=true;});
    function draw(now:number){
        if(disposed)return;request=requestAnimationFrame(draw);
        if(document.hidden||!dirty||now-last<1000/30)return;
        world.updateMatrixWorld(true);renderer.render(world,camera);
        const occupied:{x:number;y:number;w:number;h:number}[]=[];
        for(const joint of [...joints,{origin:tcp,label:tcpLabel,point:new THREE.Vector3()}]){
            joint.origin.getWorldPosition(joint.point).project(camera);
            const x=(joint.point.x+1)*width/2, originalY=(1-joint.point.y)*height/2;
            let labelY=originalY;const w=joint.label.offsetWidth,h=joint.label.offsetHeight;
            for(let step=0;step<7&&occupied.some(box=>Math.abs(box.x-x)<Math.max(box.w,w)+8&&Math.abs(box.y-labelY)<Math.max(box.h,h)+3);step++)labelY+=h+4;
            joint.label.style.left=`${Math.max(0,Math.min(width-w-12,x))}px`;joint.label.style.top=`${Math.max(15,Math.min(height-h,labelY))}px`;
            joint.label.hidden=joint.point.z < -1||joint.point.z>1||Math.abs(joint.point.x)>1||Math.abs(joint.point.y)>1;
            if(!joint.label.hidden)occupied.push({x,y:labelY,w,h});
        }
        dirty=false;last=now;
    }request=requestAnimationFrame(draw);
    function setModel(items: LoadedLinkMesh[], matrix = new THREE.Matrix4()) {
        for (const mesh of [...surfaces, ...safeties]) { model.remove(mesh); if (!(mesh.material === safetyMaterial)) (mesh.material as THREE.Material).dispose(); }
        surfaces.length = safeties.length = 0;
        model.matrixAutoUpdate = false; model.matrix.copy(matrix); model.matrixWorldNeedsUpdate = true;
        const links = ['base','link_1','link_2','link_3','link_4','link_5','link_6','tool'];
        for (const item of items) {
            const surface = new THREE.Mesh(item.geometry, new THREE.MeshStandardMaterial({color: linkColors[links.indexOf(item.link)] ?? 0x8096a3, roughness:.75, metalness:.05, side:THREE.DoubleSide}));
            surface.userData = {name:item.name, link:item.link}; surfaces.push(surface); model.add(surface);
            if (item.safety) { const safety = new THREE.Mesh(item.safety, safetyMaterial); safety.renderOrder = 3; safeties.push(safety); model.add(safety); }
        }
        applyDisplay(); renderer.domElement.dataset.modelMeshes = String(surfaces.length);
        if (items.length) fit(); dirty = true;
    }
    function applyDisplay() {
        for (const mesh of safeties) mesh.visible = display.safety;
        for (const object of primitives) object.visible = display.primitives || !surfaces.length;
        // Pręty łączące przeguby pokazują się tylko z bryłami zastępczymi.
        if (!display.primitives && surfaces.length) joints.forEach(joint => { joint.link.visible = false; });
        dirty = true;
    }
    function setDisplay(next: {safety:boolean; primitives:boolean}) { display = next; applyDisplay(); }
    function setPick(handler: ((pick: ModelPick) => void) | null) {
        pickHandler = handler; renderer.domElement.style.cursor = handler ? 'crosshair' : ''; renderer.domElement.dataset.picking = handler ? '1' : '';
    }
    function setOverlay(next: PickOverlay) {
        const size = Math.max(.15, new THREE.Box3().setFromObject(root).getSize(new THREE.Vector3()).length());
        axisLine.visible = ring.visible = !!next.axis; marker.visible = !!(next.point ?? next.axis);
        if (next.axis) {
            const {point, direction, radius} = next.axis, half = direction.clone().normalize().multiplyScalar(size * .35);
            axisLine.geometry.setFromPoints([point.clone().sub(half), point.clone().add(half)]);
            ring.position.copy(point); ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), direction.clone().normalize()); ring.scale.setScalar(Math.max(radius, .002) * 1.15);
        }
        const at = next.point ?? next.axis?.point;
        if (at) { marker.position.copy(at); marker.scale.setScalar(size * .006); }
        dirty = true;
    }
    return {update,fit,setModel,setDisplay,setPick,setOverlay,dispose(){
        disposed=true;cancelAnimationFrame(request);resize.disconnect();controls.dispose();
        const geometries=new Set<THREE.BufferGeometry>(),materials=new Set<THREE.Material>();
        world.traverse(object=>{const mesh=object as THREE.Mesh;if(mesh.geometry)geometries.add(mesh.geometry);if(mesh.material)(Array.isArray(mesh.material)?mesh.material:[mesh.material]).forEach(item=>materials.add(item));});
        geometries.forEach(item=>item.dispose());materials.forEach(item=>item.dispose());
        renderer.dispose();renderer.forceContextLoss();host.replaceChildren();
    }};
}
