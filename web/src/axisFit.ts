import * as THREE from "three";

// Dopasowanie osi przegubu do powierzchni walcowej siatki CAD (czop, gniazdo łożyska, otwór):
// 1) region gładkich ścian od wskazanego trójkąta (sąsiedzi przez wspólną krawędź, kąt normalnych < maxStepDeg),
// 2) kierunek osi = wektor własny najmniejszej wartości macierzy Σ a·n·nᵀ (normalne walca są prostopadłe do osi),
// 3) punkt osi = punkt najbliższy prostym normalnych (przechodzą przez oś), składowa wzdłuż osi = środek regionu,
// 4) odrzucenie ścian odchylonych od walca (zaokrąglenia przechodzące w płaszczyzny) i ponowne dopasowanie.
export type CylinderFit = { point: THREE.Vector3; direction: THREE.Vector3; radius: number; residual: number; faces: number; coverageDeg: number };
type Topology = { normals: Float64Array; centroids: Float64Array; areas: Float64Array; offsets: Int32Array; neighbors: Int32Array };
const cache = new WeakMap<THREE.BufferGeometry, Topology>();

export function topology(geometry: THREE.BufferGeometry): Topology {
    const cached = cache.get(geometry);
    if (cached) return cached;
    const position = geometry.getAttribute("position"), index = geometry.getIndex();
    const faceCount = (index ? index.count : position.count) / 3;
    const vertex = (f: number, k: number) => index ? index.getX(f * 3 + k) : f * 3 + k;
    // Spawanie wierzchołków (STL nie ma indeksów): klucz z zaokrąglenia do 1 µm.
    const welded = new Int32Array(position.count), ids = new Map<string, number>();
    for (let i = 0; i < position.count; i++) {
        const key = `${Math.round(position.getX(i) * 1e6)},${Math.round(position.getY(i) * 1e6)},${Math.round(position.getZ(i) * 1e6)}`;
        let id = ids.get(key); if (id === undefined) { id = ids.size; ids.set(key, id); }
        welded[i] = id;
    }
    const normals = new Float64Array(faceCount * 3), centroids = new Float64Array(faceCount * 3), areas = new Float64Array(faceCount);
    const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3(), n = new THREE.Vector3();
    const edges = new Map<number, number[]>(), count = ids.size;
    for (let f = 0; f < faceCount; f++) {
        a.fromBufferAttribute(position, vertex(f, 0)); b.fromBufferAttribute(position, vertex(f, 1)); c.fromBufferAttribute(position, vertex(f, 2));
        n.crossVectors(b.clone().sub(a), c.clone().sub(a));
        areas[f] = n.length() / 2; if (areas[f] > 0) n.normalize();
        normals.set([n.x, n.y, n.z], f * 3); centroids.set([(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3], f * 3);
        for (let k = 0; k < 3; k++) {
            const p = welded[vertex(f, k)], q = welded[vertex(f, (k + 1) % 3)];
            const key = Math.min(p, q) * count + Math.max(p, q);
            const list = edges.get(key); if (list) list.push(f); else edges.set(key, [f]);
        }
    }
    const degree = new Int32Array(faceCount + 1);
    for (const list of edges.values()) if (list.length > 1) for (const f of list) degree[f + 1] += list.length - 1;
    for (let f = 0; f < faceCount; f++) degree[f + 1] += degree[f];
    const neighbors = new Int32Array(degree[faceCount]), fill = degree.slice(0, faceCount);
    for (const list of edges.values()) if (list.length > 1) for (const f of list) for (const g of list) if (g !== f) neighbors[fill[f]++] = g;
    const result = {normals, centroids, areas, offsets: degree, neighbors};
    cache.set(geometry, result); return result;
}

export function smoothRegion(top: Topology, start: number, maxStepDeg = 30, maxFaces = 50_000) {
    const limit = Math.cos(THREE.MathUtils.degToRad(maxStepDeg)), seen = new Uint8Array(top.areas.length), region = [start], queue = [start];
    seen[start] = 1;
    while (queue.length && region.length < maxFaces) {
        const f = queue.pop()!;
        for (let k = top.offsets[f]; k < top.offsets[f + 1]; k++) {
            const g = top.neighbors[k];
            if (seen[g]) continue;
            const dot = top.normals[f * 3] * top.normals[g * 3] + top.normals[f * 3 + 1] * top.normals[g * 3 + 1] + top.normals[f * 3 + 2] * top.normals[g * 3 + 2];
            if (dot < limit) continue;
            seen[g] = 1; region.push(g); queue.push(g);
        }
    }
    return region;
}

// Wartości i wektory własne macierzy symetrycznej 3×3 (Jacobi), rosnąco.
export function symmetricEigen(m: number[][]) {
    const a = m.map(row => [...row]), v = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    for (let sweep = 0; sweep < 50; sweep++) {
        let off = 0;
        for (let p = 0; p < 3; p++) for (let q = p + 1; q < 3; q++) off += a[p][q] ** 2;
        if (off < 1e-30) break;
        for (let p = 0; p < 3; p++) for (let q = p + 1; q < 3; q++) {
            if (Math.abs(a[p][q]) < 1e-300) continue;
            const theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
            const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1)), c = 1 / Math.sqrt(t * t + 1), s = t * c;
            for (let k = 0; k < 3; k++) { const akp = a[k][p], akq = a[k][q]; a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq; }
            for (let k = 0; k < 3; k++) { const apk = a[p][k], aqk = a[q][k]; a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk; }
            for (let k = 0; k < 3; k++) { const vkp = v[k][p], vkq = v[k][q]; v[k][p] = c * vkp - s * vkq; v[k][q] = s * vkp + c * vkq; }
        }
    }
    return [0, 1, 2].map(i => ({value: a[i][i], vector: new THREE.Vector3(v[0][i], v[1][i], v[2][i]).normalize()})).sort((x, y) => x.value - y.value);
}

function fitFaces(top: Topology, faces: number[]) {
    const m = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    let total = 0;
    for (const f of faces) {
        const w = top.areas[f], n = [top.normals[f * 3], top.normals[f * 3 + 1], top.normals[f * 3 + 2]];
        total += w;
        for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) m[i][j] += w * n[i] * n[j];
    }
    const eigen = symmetricEigen(m), direction = eigen[0].vector;
    // Punkt najbliższy prostym normalnych: Σ w(I − n nᵀ) p = Σ w(I − n nᵀ) c.
    const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], B = [0, 0, 0];
    for (const f of faces) {
        const w = top.areas[f], n = [top.normals[f * 3], top.normals[f * 3 + 1], top.normals[f * 3 + 2]], c = [top.centroids[f * 3], top.centroids[f * 3 + 1], top.centroids[f * 3 + 2]];
        for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
            const value = w * ((i === j ? 1 : 0) - n[i] * n[j]);
            A[i][j] += value; B[i] += value * c[j];
        }
    }
    const matrix = new THREE.Matrix3().set(A[0][0], A[0][1], A[0][2], A[1][0], A[1][1], A[1][2], A[2][0], A[2][1], A[2][2]);
    const point = new THREE.Vector3(...B).applyMatrix3(matrix.invert());
    return {direction, point, eigen, total};
}

export function fitCylinder(geometry: THREE.BufferGeometry, face: number, matrix = new THREE.Matrix4(), maxStepDeg = 30): CylinderFit {
    const top = topology(geometry);
    if (!(face >= 0 && face < top.areas.length)) throw Error("Nie wskazano ściany siatki");
    let faces = smoothRegion(top, face, maxStepDeg);
    let fit = fitFaces(top, faces);
    // Płaszczyzna: normalne niemal równoległe — oś nieokreślona.
    if (fit.eigen[1].value < .02 * fit.total) throw Error("Wskazana powierzchnia jest płaska — wskaż powierzchnię walcową (czop, otwór, gniazdo łożyska)");
    for (let pass = 0; pass < 2; pass++) {
        const keep = faces.filter(f => Math.abs(top.normals[f * 3] * fit.direction.x + top.normals[f * 3 + 1] * fit.direction.y + top.normals[f * 3 + 2] * fit.direction.z) < Math.sin(THREE.MathUtils.degToRad(15)));
        if (keep.length < 6 || keep.length === faces.length) break;
        faces = keep; fit = fitFaces(top, faces);
    }
    if (faces.length < 6 || fit.eigen[1].value < .02 * fit.total) throw Error("Za mały fragment walca — wskaż większą powierzchnię walcową");
    // Promień, odchyłka i zasięg kątowy w płaszczyźnie prostopadłej do osi.
    const u = new THREE.Vector3().crossVectors(fit.direction, Math.abs(fit.direction.x) < .9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0)).normalize();
    const v = new THREE.Vector3().crossVectors(fit.direction, u);
    const radial: number[] = [], weights: number[] = [], angles: number[] = [];
    const c = new THREE.Vector3(), r = new THREE.Vector3();
    for (const f of faces) {
        c.set(top.centroids[f * 3], top.centroids[f * 3 + 1], top.centroids[f * 3 + 2]).sub(fit.point);
        r.copy(c).addScaledVector(fit.direction, -c.dot(fit.direction));
        radial.push(r.length()); weights.push(top.areas[f]); angles.push(Math.atan2(r.dot(v), r.dot(u)));
    }
    const weight = weights.reduce((x, y) => x + y, 0);
    const radius = radial.reduce((sum, value, i) => sum + value * weights[i], 0) / weight;
    const residual = Math.sqrt(radial.reduce((sum, value, i) => sum + (value - radius) ** 2 * weights[i], 0) / weight);
    angles.sort((x, y) => x - y);
    let gap = angles.length ? angles[0] + 2 * Math.PI - angles[angles.length - 1] : 2 * Math.PI;
    for (let i = 1; i < angles.length; i++) gap = Math.max(gap, angles[i] - angles[i - 1]);
    const rotation = new THREE.Matrix3().setFromMatrix4(matrix);
    return {point: fit.point.clone().applyMatrix4(matrix), direction: fit.direction.clone().applyMatrix3(rotation).normalize(),
        radius, residual, faces: faces.length, coverageDeg: THREE.MathUtils.radToDeg(2 * Math.PI - gap)};
}

// Płaszczyzna styku (czoło kołnierza, powierzchnia oporowa): region ścian o niemal równoległych normalnych od wskazanego
// trójkąta, normalna i punkt (środek) ważone polem. Ściana zakrzywiona → komunikat.
export type PlaneFit = { point: THREE.Vector3; normal: THREE.Vector3; faces: number; flatnessDeg: number };
export function fitPlane(geometry: THREE.BufferGeometry, face: number, maxStepDeg = 3): PlaneFit {
    const top = topology(geometry);
    if (!(face >= 0 && face < top.areas.length)) throw Error("Nie wskazano ściany siatki");
    const faces = smoothRegion(top, face, maxStepDeg);
    const normal = new THREE.Vector3(), point = new THREE.Vector3();
    let total = 0;
    for (const f of faces) {
        const w = top.areas[f];
        normal.x += w * top.normals[f * 3]; normal.y += w * top.normals[f * 3 + 1]; normal.z += w * top.normals[f * 3 + 2];
        point.x += w * top.centroids[f * 3]; point.y += w * top.centroids[f * 3 + 1]; point.z += w * top.centroids[f * 3 + 2];
        total += w;
    }
    if (!(total > 0) || normal.length() < 1e-12) throw Error("Nie wyznaczono płaszczyzny — wskaż płaską ścianę");
    normal.normalize(); point.multiplyScalar(1 / total);
    let worst = 0;
    for (const f of faces) worst = Math.max(worst, Math.acos(Math.min(1, top.normals[f * 3] * normal.x + top.normals[f * 3 + 1] * normal.y + top.normals[f * 3 + 2] * normal.z)));
    const flatnessDeg = THREE.MathUtils.radToDeg(worst);
    if (flatnessDeg > 6) throw Error("Wskazana powierzchnia nie jest płaska — wskaż czoło (płaską ścianę) styku");
    return {point, normal, faces: faces.length, flatnessDeg};
}
