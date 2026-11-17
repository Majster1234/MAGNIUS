import * as THREE from "three";

// Model 3D robota: siatki członów z CAD (układ złożenia, pozycja zerowa), siatki bezpieczeństwa (2 mm, rzadsze)
// i przekształcenie układu złożenia do układu bazy robota. Backend: robot_web_bridge/robot_model.py.
export const modelLinks = ["base", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "tool"] as const;
export type ModelLink = typeof modelLinks[number];
export const modelLinkLabels: Record<ModelLink, string> = {base: "Podstawa", link_1: "Człon 1 (za J1)", link_2: "Człon 2 (za J2)",
    link_3: "Człon 3 (za J3)", link_4: "Człon 4 (za J4)", link_5: "Człon 5 (za J5)", link_6: "Człon 6 / kołnierz", tool: "Narzędzie"};
export type AssemblyTransform = { x: number; y: number; z: number; roll: number; pitch: number; yaw: number };
export type Assembly = { revision: string; up: "z" | "y"; transform: AssemblyTransform; links: Record<ModelLink, string[]> };
export type SafetyInfo = { state: "queued" | "processing" | "ready" | "failed"; message?: string; triangles?: number; surface_triangles?: number;
    offset_mm?: number; cell_mm?: number; min_mm?: number; mean_mm?: number; max_mm?: number; coarser_than_surface?: boolean; seconds?: number; simplified_mm?: number; error_mm?: number };
export type MeshInfo = { id: string; name: string; units: string; size_bytes: number; created_at: string;
    state: "queued" | "processing" | "ready" | "failed"; message?: string; triangles?: number; bbox_min?: number[]; bbox_max?: number[]; safety: SafetyInfo };
export type RobotModel = { assembly: Assembly; meshes: MeshInfo[] };
export const emptyAssembly = (): Assembly => ({revision: "initial", up: "z", transform: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0},
    links: Object.fromEntries(modelLinks.map(link => [link, []])) as unknown as Record<ModelLink, string[]>});

async function reply<T>(response: Response): Promise<T> {
    const body = await response.json().catch(() => ({}));
    if (!response.ok) throw Error(typeof body?.detail === "string" ? body.detail : `Błąd ${response.status}`);
    return body as T;
}
export const fetchRobotModel = async (): Promise<RobotModel> => reply(await fetch("/api/v1/robot/model", {cache: "no-store"}));
export const uploadMesh = async (file: File, units: string): Promise<MeshInfo> =>
    reply(await fetch(`/api/v1/robot/model/meshes?${new URLSearchParams({name: file.name, units})}`,
        {method: "POST", body: file, headers: {"Content-Type": "application/octet-stream"}}));
export const saveAssembly = async (assembly: Omit<Assembly, "revision">, expected: string): Promise<Assembly> =>
    reply(await fetch("/api/v1/robot/model", {method: "PUT", headers: {"Content-Type": "application/json"},
        body: JSON.stringify({expected_revision: expected, assembly})}));
export const deleteMesh = async (id: string) => reply(await fetch(`/api/v1/robot/model/meshes/${id}`, {method: "DELETE"}));

// Przekształcenie złożenie → baza: najpierw oś „w górę” (Fusion często Y w górę → obrót +90° wokół X), potem
// przesunięcie [mm] i obrót roll/pitch/yaw [°] w konwencji URDF (Rz·Ry·Rx).
export function assemblyMatrix(assembly: Pick<Assembly, "up" | "transform">) {
    const up = new THREE.Matrix4();
    if (assembly.up === "y") up.makeRotationX(Math.PI / 2);
    const t = assembly.transform, rad = THREE.MathUtils.degToRad;
    const offset = new THREE.Matrix4().compose(new THREE.Vector3(t.x / 1000, t.y / 1000, t.z / 1000),
        new THREE.Quaternion().setFromEuler(new THREE.Euler(rad(t.roll), rad(t.pitch), rad(t.yaw), "ZYX")), new THREE.Vector3(1, 1, 1));
    return offset.multiply(up);
}

// Pliki siatek są niezmienne dla identyfikatora — jedna geometria na sesję strony.
const geometries = new Map<string, Promise<THREE.BufferGeometry>>();
export function meshGeometry(id: string, kind: "surface" | "safety" = "surface") {
    const key = `${id}/${kind}`;
    if (!geometries.has(key)) geometries.set(key, (async () => {
        const response = await fetch(`/api/v1/robot/model/meshes/${id}/${kind}.stl`);
        if (!response.ok) throw Error(`Brak pliku siatki (${response.status})`);
        const { STLLoader } = await import("three/addons/loaders/STLLoader.js");
        const geometry = new STLLoader().parse(await response.arrayBuffer());
        geometry.computeBoundingSphere(); geometry.computeBoundingBox();
        return geometry;
    })().catch(error => { geometries.delete(key); throw error; }));
    return geometries.get(key)!;
}
export type LoadedLinkMesh = { link: ModelLink; id: string; name: string; geometry: THREE.BufferGeometry; safety?: THREE.BufferGeometry };
// Geometrie przypisanych siatek (gotowych); siatka bezpieczeństwa opcjonalnie.
export async function loadAssemblyMeshes(model: RobotModel, safety = false): Promise<LoadedLinkMesh[]> {
    const ready = new Map(model.meshes.filter(mesh => mesh.state === "ready").map(mesh => [mesh.id, mesh]));
    const items = modelLinks.flatMap(link => model.assembly.links[link].filter(id => ready.has(id)).map(id => ({link, id, mesh: ready.get(id)!})));
    return Promise.all(items.map(async ({link, id, mesh}) => ({link, id, name: mesh.name, geometry: await meshGeometry(id),
        safety: safety && mesh.safety.state === "ready" ? await meshGeometry(id, "safety").catch(() => undefined) : undefined})));
}
