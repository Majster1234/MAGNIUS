import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { demoTranslations, jointAxes, originKeys, tcpKeys, type Values } from "./configurationData";
type Input = {values: Values; view: "geometry" | "limits"; selected: number};
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
    let dirty = true, width=1, height=1, disposed=false, first=true, last=-Infinity, request=0;
    function fit() {
        root.updateMatrixWorld(true);
        const box = new THREE.Box3().setFromObject(root), center = box.getCenter(new THREE.Vector3());
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
            const length=end.length(); joint.link.visible=length>1e-9;
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
    const resize=new ResizeObserver(()=>{width=Math.max(1,host.clientWidth);height=Math.max(1,host.clientHeight);renderer.setSize(width,height);camera.aspect=width/height;camera.updateProjectionMatrix();dirty=true;});resize.observe(host);
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
    return {update,fit,dispose(){
        disposed=true;cancelAnimationFrame(request);resize.disconnect();controls.dispose();
        const geometries=new Set<THREE.BufferGeometry>(),materials=new Set<THREE.Material>();
        world.traverse(object=>{const mesh=object as THREE.Mesh;if(mesh.geometry)geometries.add(mesh.geometry);if(mesh.material)(Array.isArray(mesh.material)?mesh.material:[mesh.material]).forEach(item=>materials.add(item));});
        geometries.forEach(item=>item.dispose());materials.forEach(item=>item.dispose());
        renderer.dispose();renderer.forceContextLoss();host.replaceChildren();
    }};
}
