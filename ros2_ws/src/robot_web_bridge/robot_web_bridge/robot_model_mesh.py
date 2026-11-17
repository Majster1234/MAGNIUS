"""Siatki modelu 3D robota: odczyt STL/OBJ, zapis binarnego STL, siatka bezpieczeństwa.

Moduł działa w osobnym procesie (``python3 -m robot_web_bridge.robot_model_mesh <zadanie.json>``, nice 10):
obliczenia numpy w procesie mostu blokowałyby GIL i opóźniały KEEPALIVE JOG. Jednostki wewnętrzne: metry.

Siatka bezpieczeństwa (przyszłe kolizje): powierzchnia odsunięta o ``offset`` (domyślnie 2 mm) na zewnątrz bryły,
rzadsza niż siatka wizualna. Metoda: pole odległości bez znaku na siatce wokselowej (KD-drzewo próbek powierzchni),
obszar zewnętrzny z wypełnienia od brzegu siatki, powierzchnia izo „surface nets”, rzutowanie wierzchołków dokładnie
na odległość ``offset`` od najbliższego trójkąta i dosunięcie ścian, których środki/krawędzie leżą bliżej.
"""
import json
import math
import os
from pathlib import Path
import re
import struct
import sys
import time

import numpy as np

UNITS = {'mm': .001, 'cm': .01, 'm': 1., 'in': .0254}
MAX_TRIANGLES = 3_000_000
# Budżet próbek powierzchni (pamięć procesu importu); powyżej — wstępne uproszczenie siatki.
SAMPLE_BUDGET = 1_400_000


class MeshError(ValueError):
    pass


def parse_stl(data):
    if len(data) >= 84:
        count = struct.unpack_from('<I', data, 80)[0]
        if len(data) == 84 + 50 * count:
            if count > MAX_TRIANGLES:
                raise MeshError(f'Za dużo trójkątów ({count}); limit {MAX_TRIANGLES}')
            records = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=count, offset=84)
            return records['v'].astype(np.float64).reshape(-1, 3), np.arange(count * 3, dtype=np.int64).reshape(-1, 3)
    text = data.decode('ascii', errors='replace')
    if 'vertex' not in text:
        raise MeshError('Plik STL nie zawiera trójkątów (ani binarnych, ani ASCII)')
    values = re.findall(r'vertex\s+(\S+)\s+(\S+)\s+(\S+)', text)
    if len(values) % 3 or not values:
        raise MeshError('Uszkodzony STL ASCII: liczba wierzchołków nie jest wielokrotnością 3')
    vertices = np.array(values, dtype=np.float64)
    if len(vertices) // 3 > MAX_TRIANGLES:
        raise MeshError(f'Za dużo trójkątów; limit {MAX_TRIANGLES}')
    return vertices, np.arange(len(vertices), dtype=np.int64).reshape(-1, 3)


def parse_obj(data):
    vertices, faces = [], []
    for raw in data.decode('utf-8', errors='replace').splitlines():
        parts = raw.split()
        if not parts:
            continue
        if parts[0] == 'v' and len(parts) >= 4:
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif parts[0] == 'f' and len(parts) >= 4:
            index = []
            for item in parts[1:]:
                value = int(item.split('/')[0])
                index.append(value - 1 if value > 0 else len(vertices) + value)
            for i in range(1, len(index) - 1):  # wielokąt → wachlarz trójkątów
                faces.append((index[0], index[i], index[i + 1]))
            if len(faces) > MAX_TRIANGLES:
                raise MeshError(f'Za dużo trójkątów; limit {MAX_TRIANGLES}')
    if not faces:
        raise MeshError('Plik OBJ nie zawiera ścian')
    vertices, faces = np.array(vertices, dtype=np.float64), np.array(faces, dtype=np.int64)
    if faces.min() < 0 or faces.max() >= len(vertices):
        raise MeshError('Plik OBJ odwołuje się do nieistniejących wierzchołków')
    return vertices, faces


def load_mesh(path, units):
    if units not in UNITS:
        raise MeshError('Jednostki: mm, cm, m albo in')
    data = Path(path).read_bytes()
    suffix = Path(path).suffix.lower()
    vertices, faces = parse_obj(data) if suffix == '.obj' else parse_stl(data)
    if not np.isfinite(vertices).all():
        raise MeshError('Siatka zawiera wartości nieskończone lub NaN')
    vertices = vertices * UNITS[units]
    a, b, c = (vertices[faces[:, i]] for i in range(3))
    keep = np.linalg.norm(np.cross(b - a, c - a), axis=1) > 1e-14  # bez trójkątów zdegenerowanych
    faces = faces[keep]
    if not len(faces):
        raise MeshError('Siatka nie zawiera niezdegenerowanych trójkątów')
    return vertices, faces


def write_stl(path, vertices, faces):
    triangles = vertices[faces].astype(np.float32)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, length, out=np.zeros_like(normals), where=length > 0)
    records = np.zeros(len(faces), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    records['n'], records['v'] = normals, triangles
    temporary = Path(str(path) + '.tmp')
    with temporary.open('wb') as stream:
        stream.write(b'robot model mesh'.ljust(80, b' ')); stream.write(struct.pack('<I', len(faces)))
        stream.write(records.tobytes()); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)


def surface_area(vertices, faces):
    a, b, c = (vertices[faces[:, i]] for i in range(3))
    return float(.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1).sum())


def signed_volume(vertices, faces):
    a, b, c = (vertices[faces[:, i]] for i in range(3))
    return float(np.einsum('ij,ij->i', a, np.cross(b, c)).sum() / 6)


def shell_components(vertices, faces):
    """Indeks spójnej bryły (wspólne wierzchołki po zespawaniu do 1 µm) dla każdego trójkąta."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    welded = weld(vertices)
    corners = welded[faces]
    edges = np.concatenate([corners[:, [0, 1]], corners[:, [1, 2]], corners[:, [2, 0]]])
    count = int(welded.max()) + 1
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(count, count))
    _, labels = connected_components(graph, directed=False)
    return labels[corners[:, 0]]


def sample_layout(vertices, faces):
    """Parametryzacja próbkowania: wierzchołek wspólny p0, krawędzie e1 (najdłuższa) i e2 (najkrótsza)."""
    tri = vertices[faces]
    lengths = np.stack([np.linalg.norm(tri[:, 1] - tri[:, 0], axis=1), np.linalg.norm(tri[:, 2] - tri[:, 1], axis=1),
                        np.linalg.norm(tri[:, 0] - tri[:, 2], axis=1)], axis=1)   # krawędź k: wierzchołek k → k+1
    longest, shortest = lengths.argmax(axis=1), lengths.argmin(axis=1)
    shortest = np.where(shortest == longest, (longest + 1) % 3, shortest)
    # Wspólny wierzchołek krawędzi longest (k→k+1) i shortest (m→m+1): k == m+1 → k, inaczej k+1 == m → m.
    shared = np.where(longest == (shortest + 1) % 3, longest, shortest)
    rows = np.arange(len(tri))
    p0 = tri[rows, shared]
    far1 = np.where(shared == longest, (longest + 1) % 3, longest)
    far2 = np.where(shared == shortest, (shortest + 1) % 3, shortest)
    return p0, tri[rows, far1] - p0, tri[rows, far2] - p0


def sample_count(vertices, faces, spacing):
    _, e1, e2 = sample_layout(vertices, faces)
    n1 = np.maximum(1, np.ceil(np.linalg.norm(e1, axis=1) / spacing)); n2 = np.maximum(1, np.ceil(np.linalg.norm(e2, axis=1) / spacing))
    return float(((n1 + 1) * (n2 + 1) / 2 + n1 + n2).sum())


def cluster_simplify(vertices, faces, resolution):
    """Uproszczenie przez klasteryzację wierzchołków (siatka `resolution`): wierzchołek = środek klastra,
    trójkąty zdegenerowane i powtórzone usunięte. Przesunięcie każdego punktu powierzchni ≤ resolution·√3."""
    cluster = weld(vertices, resolution)
    count = int(cluster.max()) + 1
    centers = np.stack([np.bincount(cluster, weights=vertices[:, i], minlength=count) for i in range(3)], axis=1)
    centers /= np.bincount(cluster, minlength=count)[:, None]
    mapped = cluster[faces]
    keep = (mapped[:, 0] != mapped[:, 1]) & (mapped[:, 1] != mapped[:, 2]) & (mapped[:, 2] != mapped[:, 0])
    mapped = mapped[keep]
    # Powtórzone trójkąty (te same wierzchołki, ta sama orientacja) — jeden.
    key = np.sort(mapped, axis=1)
    _, unique = np.unique(key[:, 0].astype(np.int64) * count * count + key[:, 1] * count + key[:, 2], return_index=True)
    return centers, mapped[np.sort(unique)]


def triangle_samples(vertices, faces, spacing, max_points=1_500_000):
    """Punkty na trójkątach, indeks trójkąta każdego punktu i faktyczny krok siatki próbek.

    Siatka w parametryzacji P0 + a·e1 + b·e2 (e1 — najdłuższa krawędź, e2 — najkrótsza, wspólny wierzchołek P0),
    krok ≤ spacing wzdłuż obu krawędzi: każdy punkt trójkąta leży ≤ spacing od próbki. Liczba punktów ∝ polu
    także dla długich, wąskich trójkątów walców z CAD. Przy przekroczeniu budżetu krok rośnie i jest zwracany
    (od niego zależy dokładność pola odległości). Generowanie grupami o tej samej liczbie podziałów — pamięć
    ograniczona budżetem punktów.
    """
    p0, e1, e2 = sample_layout(vertices, faces)
    l1, l2 = np.linalg.norm(e1, axis=1), np.linalg.norm(e2, axis=1)
    while True:
        n1 = np.maximum(1, np.ceil(l1 / spacing).astype(np.int64)); n2 = np.maximum(1, np.ceil(l2 / spacing).astype(np.int64))
        counts = (n1 + 1) * (n2 + 1) // 2 + n1 + n2
        if counts.sum() <= max_points:
            break
        spacing *= math.sqrt(counts.sum() / max_points) * 1.05
    points, owners = [], []
    key = n1 * 100_000 + n2
    for value in np.unique(key):
        index = np.nonzero(key == value)[0]
        a1, a2 = int(value // 100_000), int(value % 100_000)
        i, j = np.meshgrid(np.arange(a1 + 1), np.arange(a2 + 1), indexing='ij')
        u, v = (i / a1).ravel(), (j / a2).ravel()
        keep = u + v <= 1 + 1e-12
        # Punkty przy przeciwległej krawędzi (u + v > 1 tuż za nią) zastąpione rzutem na nią — pokrycie całego trójkąta.
        over = ~keep & (u + v <= 1 + 1 / min(a1, a2) + 1e-12)
        su = u[over] / (u[over] + v[over]); sv = 1 - su
        u, v = np.concatenate([u[keep], su]), np.concatenate([v[keep], sv])
        for start in range(0, len(index), max(1, 500_000 // len(u))):
            part = index[start:start + max(1, 500_000 // len(u))]
            grid = p0[part, None, :] + u[None, :, None] * e1[part, None, :] + v[None, :, None] * e2[part, None, :]
            points.append(grid.reshape(-1, 3)); owners.append(np.repeat(part, len(u)))
    return np.concatenate(points), np.concatenate(owners), spacing


def closest_points(p, a, b, c, regions=False):
    """Najbliższe punkty trójkątów (a, b, c) dla punktów p — wektorowo (Ericson, Real-Time Collision Detection 5.1.5).

    regions=True zwraca też cechę: 0 wnętrze, 1/2/3 wierzchołek a/b/c, 4/5/6 krawędź ab/bc/ca.
    """
    ab, ac, ap = b - a, c - a, p - a
    dot = lambda x, y: np.einsum('ij,ij->i', x, y)
    d1, d2 = dot(ab, ap), dot(ac, ap)
    bp = p - b; d3, d4 = dot(ab, bp), dot(ac, bp)
    cp = p - c; d5, d6 = dot(ab, cp), dot(ac, cp)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
    region = np.zeros(len(p), dtype=np.int8)
    with np.errstate(divide='ignore', invalid='ignore'):
        denom = va + vb + vc
        v = np.where(denom != 0, vb / denom, 0.); w = np.where(denom != 0, vc / denom, 0.)
        result = a + ab * v[:, None] + ac * w[:, None]
        done = np.zeros(len(p), dtype=bool)

        def take(mask, value, code):
            nonlocal done
            mask = mask & ~done
            result[mask] = value[mask]; region[mask] = code; done |= mask

        take((d1 <= 0) & (d2 <= 0), a, 1)
        take((d3 >= 0) & (d4 <= d3), b, 2)
        t = np.where(d1 - d3 != 0, d1 / (d1 - d3), 0.); take((vc <= 0) & (d1 >= 0) & (d3 <= 0), a + ab * t[:, None], 4)
        take((d6 >= 0) & (d5 <= d6), c, 3)
        t = np.where(d2 - d6 != 0, d2 / (d2 - d6), 0.); take((vb <= 0) & (d2 >= 0) & (d6 <= 0), a + ac * t[:, None], 6)
        e = (d4 - d3) + (d5 - d6); t = np.where(e != 0, (d4 - d3) / e, 0.)
        take((va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0), b + (c - b) * t[:, None], 5)
    return (result, region) if regions else result


def weld(vertices, resolution=1e-6):
    """Indeks zespawanego wierzchołka (wspólne położenie z dokładnością `resolution`)."""
    grid = np.round(vertices / resolution).astype(np.int64)
    grid -= grid.min(axis=0)
    span = grid.max(axis=0) + 1
    if float(span[0]) * float(span[1]) * float(span[2]) < 9e18:
        return np.unique((grid[:, 0] * span[1] + grid[:, 1]) * span[2] + grid[:, 2], return_inverse=True)[1]
    return np.unique(grid, axis=0, return_inverse=True)[1]


class SurfaceDistance:
    """Dokładna odległość do siatki: kandydaci z KD-drzewa próbek, potem najbliższy punkt trójkątów.

    Strona (na zewnątrz/wewnątrz) z pseudonormalnych ważonych kątem (Bærentzen–Aanæs): wnętrze ściany — normalna
    ściany, krawędź — suma normalnych sąsiednich ścian, wierzchołek — normalne ścian ważone kątem. Poprawne także
    przy ostrych kątach dwuściennych, gdzie normalna jednej ściany daje zły znak.
    """
    def __init__(self, vertices, faces, spacing):
        from scipy.spatial import cKDTree
        self.vertices, self.faces = vertices, faces
        self.samples, self.owners, self.spacing = triangle_samples(vertices, faces, spacing)
        self.tree = cKDTree(self.samples)
        a, b, c = (vertices[faces[:, i]] for i in range(3))
        normal = np.cross(b - a, c - a)
        normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-300)
        # Orientacja z objętości ze znakiem — osobno dla każdej spójnej bryły (komponent z kilkoma bryłami, część
        # z odwróconymi normalnymi): bryła o ujemnej objętości ma normalne do wewnątrz i jest odwracana.
        component = shell_components(vertices, faces)
        volume = np.bincount(component, weights=np.einsum('ij,ij->i', a, np.cross(b, c)))
        self.normals = normal * np.where(volume[component] < 0, -1., 1.)[:, None]
        welded = weld(vertices)
        self.corners = welded[faces]
        count = int(welded.max()) + 1
        corner_points = [a, b, c]
        self.vertex_normals = np.zeros((count, 3))
        for k in range(3):
            u = corner_points[(k + 1) % 3] - corner_points[k]; w = corner_points[(k + 2) % 3] - corner_points[k]
            cosine = np.einsum('ij,ij->i', u, w) / np.maximum(np.linalg.norm(u, axis=1) * np.linalg.norm(w, axis=1), 1e-300)
            angle = np.arccos(np.clip(cosine, -1, 1))
            np.add.at(self.vertex_normals, self.corners[:, k], self.normals * angle[:, None])
        # Krawędź k ściany: wierzchołek k → k+1; suma normalnych ścian dzielących zespawaną krawędź.
        pairs = np.sort(np.stack([self.corners[:, [0, 1]], self.corners[:, [1, 2]], self.corners[:, [2, 0]]], axis=1), axis=2)
        keys = pairs[..., 0].astype(np.int64) * count + pairs[..., 1]
        _, inverse = np.unique(keys.ravel(), return_inverse=True)
        self.edge_index = inverse.reshape(-1, 3)
        self.edge_normals = np.zeros((int(inverse.max()) + 1, 3))
        np.add.at(self.edge_normals, inverse, np.repeat(self.normals, 3, axis=0))

    def nearest(self, points, candidates=48):
        """Odległość i najbliższy punkt powierzchni; kandydaci = różne trójkąty spośród najbliższych próbek."""
        if not len(points):
            self.last_faces = np.zeros(0, dtype=np.int64); self.last_regions = np.zeros(0, dtype=np.int8)
            return np.zeros(0), np.zeros((0, 3))
        _, index = self.tree.query(points, k=min(candidates, len(self.samples)), workers=2)
        owners = self.owners[index.reshape(len(points), -1)]
        # Ten sam trójkąt w kolejnych kolumnach liczony raz (gęste próbki jednej ściany zasłaniały sąsiednią).
        order = np.sort(owners, axis=1)
        repeated = np.zeros_like(order, dtype=bool); repeated[:, 1:] = order[:, 1:] == order[:, :-1]
        best = np.full(len(points), np.inf); closest = np.zeros_like(points)
        nearest_face = np.zeros(len(points), dtype=np.int64); nearest_region = np.zeros(len(points), dtype=np.int8)
        for column in range(order.shape[1]):
            rows = np.nonzero(~repeated[:, column])[0]
            if not len(rows):
                continue
            face = self.faces[order[rows, column]]
            q, region = closest_points(points[rows], self.vertices[face[:, 0]], self.vertices[face[:, 1]], self.vertices[face[:, 2]], regions=True)
            distance = np.linalg.norm(points[rows] - q, axis=1)
            better = distance < best[rows]
            best[rows[better]] = distance[better]; closest[rows[better]] = q[better]
            nearest_face[rows[better]] = order[rows[better], column]; nearest_region[rows[better]] = region[better]
        self.last_faces, self.last_regions = nearest_face, nearest_region
        return best, closest

    def pseudonormals(self, faces, regions):
        result = self.normals[faces].copy()
        for code, corner in ((1, 0), (2, 1), (3, 2)):
            mask = regions == code
            result[mask] = self.vertex_normals[self.corners[faces[mask], corner]]
        for code, edge in ((4, 0), (5, 1), (6, 2)):
            mask = regions == code
            result[mask] = self.edge_normals[self.edge_index[faces[mask], edge]]
        return result

    def outward(self, points):
        """Najbliższy punkt powierzchni i jednostkowy kierunek od niego na zewnątrz bryły."""
        distance, closest = self.nearest(points)
        normal = self.pseudonormals(self.last_faces, self.last_regions)
        direction = points - closest
        length = np.linalg.norm(direction, axis=1)
        flat = length < 1e-9
        direction[flat] = normal[flat]; length[flat] = np.maximum(np.linalg.norm(normal[flat], axis=1), 1e-300)
        direction /= length[:, None]
        # Punkt po wewnętrznej stronie (np. wierzchołek „surface nets” przy krawędzi bryły) → kierunek odwrócony.
        inside = np.einsum('ij,ij->i', direction, normal) < 0
        direction[inside] *= -1
        return distance, closest, direction, inside


def surface_nets(field, origin, cell):
    """Powierzchnia izo f = 0 (f < 0 wewnątrz) — „naive surface nets”; trójkąty skierowane na zewnątrz."""
    inside = field < 0
    shape = np.array(field.shape)
    cells = shape - 1
    cell_ids, points = [], []
    edges = []
    for axis in range(3):
        lo = [slice(None)] * 3; hi = [slice(None)] * 3
        lo[axis] = slice(0, -1); hi[axis] = slice(1, None)
        s0, s1 = field[tuple(lo)], field[tuple(hi)]
        mask = inside[tuple(lo)] != inside[tuple(hi)]
        index = np.argwhere(mask)
        t = (s0[mask] / (s0[mask] - s1[mask]))
        position = index.astype(np.float64); position[:, axis] += t
        others = [a for a in range(3) if a != axis]
        # Cztery komórki wokół krawędzi: przesunięcia −1/0 w dwóch pozostałych osiach.
        for d0 in (-1, 0):
            for d1 in (-1, 0):
                c = index.copy(); c[:, others[0]] += d0; c[:, others[1]] += d1
                valid = (c >= 0).all(axis=1) & (c < cells).all(axis=1)
                cell_ids.append(np.ravel_multi_index(c[valid].T, cells)); points.append(position[valid])
        edges.append((axis, index, s0[mask] < 0))
    cell_ids = np.concatenate(cell_ids); points = np.concatenate(points)
    unique, inverse = np.unique(cell_ids, return_inverse=True)
    counts = np.bincount(inverse)
    vertices = np.stack([np.bincount(inverse, weights=points[:, i]) / counts for i in range(3)], axis=1)
    vertices = origin + vertices * cell
    quads = []
    for axis, index, low_inside in edges:
        a1, a2 = [(1, 2), (2, 0), (0, 1)][axis]  # para osi tworząca prawoskrętny układ z osią krawędzi
        corners = []
        for d1, d2 in ((-1, -1), (0, -1), (0, 0), (-1, 0)):
            c = index.copy(); c[:, a1] += d1; c[:, a2] += d2
            corners.append(c)
        valid = np.all([(c >= 0).all(axis=1) & (c < cells).all(axis=1) for c in corners], axis=0)
        ids = [np.searchsorted(unique, np.ravel_multi_index(c[valid].T, cells)) for c in corners]
        quad = np.stack(ids, axis=1)
        # Wnętrze na dolnym końcu krawędzi → normalna w +oś: kolejność przeciwna do zegara patrząc od +osi.
        quad[~low_inside[valid]] = quad[~low_inside[valid]][:, ::-1]
        quads.append(quad)
    quads = np.concatenate(quads)
    faces = np.concatenate([quads[:, [0, 1, 2]], quads[:, [0, 2, 3]]])
    return vertices, faces


def probe_deficits(result, triangles, distance, offset):
    """Niedobór odległości (offset − d) w środku ściany i środkach jej trzech krawędzi; maksimum na trójkąt."""
    a, b, c = result[triangles[:, 0]], result[triangles[:, 1]], result[triangles[:, 2]]
    probes = np.concatenate([(a + b + c) / 3, (a + b) / 2, (b + c) / 2, (c + a) / 2])
    d, _ = distance.nearest(probes)
    return (offset - d).reshape(4, -1).max(axis=0)


def refine(result, triangles, distance, offset, rounds=6, tolerance=.02):
    """Podział trójkątów zbyt bliskich bryle (podział krawędzi 1→2/3/4, bez T-połączeń), do `rounds` razy."""
    limit = max(4 * len(triangles), 50_000)
    for _ in range(rounds):
        bad = probe_deficits(result, triangles, distance, offset) > tolerance * offset
        if not bad.any() or len(triangles) > limit:
            break
        edges = np.sort(np.stack([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]], axis=1), axis=2)
        keys = edges[..., 0].astype(np.int64) * len(result) + edges[..., 1]
        unique, inverse = np.unique(keys.ravel(), return_inverse=True)
        inverse = inverse.reshape(-1, 3)
        split = np.zeros(len(unique), dtype=bool); split[inverse[bad].ravel()] = True
        first, second = unique[split] // len(result), unique[split] % len(result)
        _, closest, direction, _ = distance.outward(.5 * (result[first] + result[second]))
        middle = np.full(len(unique), -1, dtype=np.int64)
        middle[split] = len(result) + np.arange(int(split.sum()))
        result = np.concatenate([result, closest + direction * offset])
        m = middle[inverse]                                     # punkt środkowy krawędzi k (v_k → v_k+1) albo −1
        count = (m >= 0).sum(axis=1)
        pieces = [triangles[count == 0]]
        rows = np.nonzero(count == 3)[0]
        t, mm = triangles[rows], m[rows]
        pieces += [np.stack([t[:, 0], mm[:, 0], mm[:, 2]], 1), np.stack([mm[:, 0], t[:, 1], mm[:, 1]], 1),
                   np.stack([mm[:, 2], mm[:, 1], t[:, 2]], 1), np.stack([mm[:, 0], mm[:, 1], mm[:, 2]], 1)]
        for k in range(3):
            one = np.nonzero((count == 1) & (m[:, k] >= 0))[0]           # podzielona krawędź k
            a, b, c, mid = triangles[one, k], triangles[one, (k + 1) % 3], triangles[one, (k + 2) % 3], m[one, k]
            pieces += [np.stack([a, mid, c], 1), np.stack([mid, b, c], 1)]
            two = np.nonzero((count == 2) & (m[:, k] < 0))[0]            # niepodzielona krawędź k: (c, a) po obrocie
            a, b, c = triangles[two, (k + 1) % 3], triangles[two, (k + 2) % 3], triangles[two, k]
            m1, m2 = m[two, (k + 1) % 3], m[two, (k + 2) % 3]
            pieces += [np.stack([m1, b, m2], 1), np.stack([a, m1, m2], 1), np.stack([a, m2, c], 1)]
        triangles = np.concatenate(pieces)
    return result, triangles


def push_out(result, triangles, distance, offset, rounds=6, limit=.5, tolerance=.005):
    """Pozostały niedobór (środki ścian/krawędzi < offset) → wierzchołki na zewnątrz, łącznie ≤ limit·offset."""
    pushed = np.zeros(len(result))
    for _ in range(rounds):
        a, b, c = result[triangles[:, 0]], result[triangles[:, 1]], result[triangles[:, 2]]
        probes = np.concatenate([(a + b + c) / 3, (a + b) / 2, (b + c) / 2, (c + a) / 2])
        owner = np.tile(np.arange(len(triangles)), 4)
        d, _ = distance.nearest(probes)
        deficit = offset - d
        bad = deficit > tolerance * offset
        if not bad.any():
            break
        push = np.zeros(len(result))
        np.maximum.at(push, triangles[owner[bad]].ravel(), np.repeat(deficit[bad], 3))
        push = np.minimum(push, limit * offset - pushed)
        _, _, direction, _ = distance.outward(result)
        result = result + direction * push[:, None]
        pushed += push
    return result


def safety_mesh(vertices, faces, offset=.002, max_grid=3_000_000, target=None):
    """Siatka bezpieczeństwa: powierzchnia ``offset`` [m] na zewnątrz bryły, rzadsza niż siatka wejściowa."""
    from scipy import ndimage
    started = time.monotonic()
    original_faces = len(faces)
    # Gęsty eksport CAD (np. walce z tysiącami wąskich trójkątów): próbkowanie rośnie z długością krawędzi. Przy
    # przekroczeniu budżetu — klasteryzacja wierzchołków i odsunięcie powiększone o jej błąd (względem oryginału
    # zostaje ≥ offset); statystyki raportowane konserwatywnie (min − błąd, max + błąd).
    simplified, error = 0., 0.
    resolution = 5e-5
    while sample_count(vertices, faces, .0004) > SAMPLE_BUDGET and resolution < .0006:
        candidate_vertices, candidate_faces = cluster_simplify(vertices, faces, resolution)
        if sample_count(candidate_vertices, candidate_faces, .0004) <= SAMPLE_BUDGET or resolution * 1.6 >= .0006:
            vertices, faces = candidate_vertices, candidate_faces
            simplified, error = resolution, resolution * math.sqrt(3)
            break
        resolution *= 1.6
    nominal, offset = offset, offset + error
    area = surface_area(vertices, faces)
    target = target or int(min(25_000, max(1_500, .25 * len(faces))))
    # Odcinek między sąsiednimi węzłami, który przecina powierzchnię, ma koniec w odległości ≤ pół komórki — próg
    # „daleko” ≥ pół komórki (+ zapas na zawyżenie odległości z próbek) wyklucza przeciek wypełniania zewnętrza.
    # Próg musi zostać ≤ offset (węzły bliżej niż offset mają ujemne pole niezależnie od strony), stąd komórka ≤ 1,6·offset.
    distance = SurfaceDistance(vertices, faces, .0004)
    cell = min(1.6 * offset, 2 * (offset - distance.spacing) * .98, max(.001, math.sqrt(2 * area / target)))
    if cell < .0005:
        raise MeshError('Siatka zbyt duża dla dokładnego pola odległości — podziel człon na mniejsze części')
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    margin = offset + 3 * cell
    origin = low - margin
    shape = np.ceil((high + 2 * margin - low) / cell).astype(int) + 1
    if shape.prod() > max_grid:
        raise MeshError(f'Bryła za duża dla siatki bezpieczeństwa ({int(shape.prod())} węzłów przy komórce '
                        f'{cell * 1000:.1f} mm; limit {max_grid}) — podziel człon na mniejsze części')
    band = offset + 2 * cell
    unsigned = np.empty(tuple(shape))
    axes = [np.arange(n) * cell + origin[i] for i, n in enumerate(shape)]
    step = max(1, 1_000_000 // int(shape[1] * shape[2]))
    for start in range(0, shape[0], step):  # porcjami: pamięć ∝ jednej warstwie siatki
        block = np.stack(np.meshgrid(axes[0][start:start + step], axes[1], axes[2], indexing='ij'), axis=-1)
        values, _ = distance.tree.query(block.reshape(-1, 3), k=1, distance_upper_bound=band, workers=2)
        unsigned[start:start + step] = values.reshape(block.shape[:3])
    # Odległość z próbek zawyża prawdziwą o ≤ krok próbek — próg powiększony o ten zapas.
    threshold = cell / 2 + distance.spacing
    if threshold > offset:
        raise MeshError('Siatka zbyt duża dla dokładnego pola odległości — podziel człon na mniejsze części')
    far = unsigned > threshold
    labels, _ = ndimage.label(far)
    border = np.unique(np.concatenate([labels[0].ravel(), labels[-1].ravel(), labels[:, 0].ravel(), labels[:, -1].ravel(),
                                       labels[:, :, 0].ravel(), labels[:, :, -1].ravel()]))
    outside = np.isin(labels, border[border > 0])
    field = np.where(outside, np.minimum(unsigned, band) - offset, np.where(far, -cell, unsigned - offset))
    if not (field < 0).any() or not (field > 0).any():
        raise MeshError('Nie wyznaczono powierzchni bezpieczeństwa (brak zamkniętej bryły?)')
    result, triangles = surface_nets(field, origin, cell)
    # Rzutowanie wierzchołków na dokładną odległość offset od najbliższego trójkąta oryginału, po zewnętrznej stronie
    # (przy krawędziach bryły uśrednienie przecięć z dwóch ścian potrafi dać wierzchołek wewnątrz bryły).
    _, closest, direction, _ = distance.outward(result)
    result = closest + direction * offset
    # Środki ścian i krawędzi bliżej niż offset (cięciwy łuków przy ostrych krawędziach bryły) → lokalne zagęszczenie:
    # podział krawędzi takich trójkątów (spójny, bez szczelin), nowe wierzchołki rzutowane na powierzchnię offset.
    result, triangles = refine(result, triangles, distance, offset, rounds=2, tolerance=.1)
    result = push_out(result, triangles, distance, offset)
    checks = np.concatenate([result, result[triangles].mean(axis=1), .5 * (result[triangles[:, 0]] + result[triangles[:, 1]])])
    measured, _ = distance.nearest(checks)
    return result, triangles, {
        'offset_mm': nominal * 1000, 'cell_mm': round(cell * 1000, 3), 'triangles': int(len(triangles)),
        'surface_triangles': int(original_faces), 'coarser_than_surface': bool(len(triangles) < original_faces),
        'simplified_mm': round(simplified * 1000, 3), 'error_mm': round(error * 1000, 3),
        # Przy uproszczeniu: min/max to ograniczenia względem oryginału (± błąd), średnia — względem siatki uproszczonej.
        'min_mm': round(float(measured.min() - error) * 1000, 3), 'mean_mm': round(float(measured.mean()) * 1000, 3),
        'max_mm': round(float(measured.max() + error) * 1000, 3), 'seconds': round(time.monotonic() - started, 2),
        'closed': bool(signed_volume(result, triangles) > 0)}


def update_meta(path, **values):
    meta = json.loads(Path(path).read_text(encoding='utf-8'))
    meta.update(values)
    temporary = Path(str(path) + '.tmp')
    temporary.write_text(json.dumps(meta, ensure_ascii=False, allow_nan=False, indent=2) + '\n', encoding='utf-8')
    os.replace(temporary, path)
    return meta


def run_job(job_path):
    """Zadanie importu: surowy plik → <id>.stl (metry) → <id>.safety.stl; stan w <id>.json."""
    job = json.loads(Path(job_path).read_text(encoding='utf-8'))
    directory, identifier = Path(job['directory']), job['id']
    meta_path, raw = directory / f'{identifier}.json', Path(job['raw'])
    try:
        update_meta(meta_path, state='processing', message='Odczyt siatki')
        vertices, faces = load_mesh(raw, job['units'])
        write_stl(directory / f'{identifier}.stl', vertices, faces)
        low, high = vertices[faces.ravel()].min(axis=0), vertices[faces.ravel()].max(axis=0)
        update_meta(meta_path, state='ready', message='Siatka wczytana', triangles=int(len(faces)),
                    bbox_min=[round(float(v), 6) for v in low], bbox_max=[round(float(v), 6) for v in high],
                    area_m2=round(surface_area(vertices, faces), 6),
                    safety={'state': 'processing', 'message': 'Generowanie siatki bezpieczeństwa'})
    except MemoryError:
        update_meta(meta_path, state='failed', message='Za mało pamięci na odczyt siatki (limit procesu 2 GiB) — uprość eksport', safety={'state': 'failed', 'message': 'Brak siatki'})
        return
    except Exception as error:  # noqa: BLE001 — każdy błąd odczytu trafia do stanu siatki
        update_meta(meta_path, state='failed', message=str(error)[:300], safety={'state': 'failed', 'message': 'Brak siatki'})
        return
    finally:
        raw.unlink(missing_ok=True); Path(job_path).unlink(missing_ok=True)
    try:
        result, triangles, stats = safety_mesh(vertices, faces, offset=job.get('offset', .002))
        write_stl(directory / f'{identifier}.safety.stl', result, triangles)
        update_meta(meta_path, safety={'state': 'ready', 'message': 'Siatka bezpieczeństwa gotowa', **stats})
    except MemoryError:
        update_meta(meta_path, safety={'state': 'failed', 'message': 'Za mało pamięci (limit procesu 2 GiB) — podziel człon na mniejsze części'})
    except Exception as error:  # noqa: BLE001
        update_meta(meta_path, safety={'state': 'failed', 'message': str(error)[:300]})


# Twardy limit pamięci procesu importu: komputer steruje robotem, brak pamięci nie może dotknąć procesów sterowania.
MEMORY_LIMIT = 2 * 1024 ** 3


def limit_resources():
    try:
        os.nice(10)
    except OSError:
        pass
    try:
        import resource
        resource.setrlimit(resource.RLIMIT_AS, (MEMORY_LIMIT, MEMORY_LIMIT))
    except (ImportError, ValueError, OSError):
        pass


if __name__ == '__main__':
    limit_resources()
    run_job(sys.argv[1])
