import { useCallback, useEffect, useMemo, useRef, useState, type ReactElement } from "react";
import * as THREE from "three";
import type { Notice } from "./eventLog";
import { fitCylinder, fitPlane } from "./axisFit";
import { axisFromFit, meshGeometry, reverseAxis, saveElement, uploadMesh, type ElementAxis, type ElementRole, type MeshInfo, type RobotModel } from "./robotModel";
import { bestAngle, contactOffset, defaultFlip, directionLabel, emptyChain, insertElement, jointRows, mateMatrix, matrixPlacement, newId, placementMatrix,
    planeTiltDeg, removeElement, saveChain, withMate, worldMatrices, zeroPlacement, type ChainElement, type Mate, type ModelChain } from "./modelChain";
import type { ChainAxis, ChainMarker, ChainPick, ChainSceneItem, ChainVirtual, createChainScene } from "./chainScene";
import { chainFrames, motionLabel } from "./modelChain";
import { applyChainFrames, continueFrames } from "./frameEdit";
import { jointAxes, originKeys, tcpKeys, type Values } from "./configurationData";

// Budowa modelu robota z elementów (tryb modułowy, wariant v2 z temp/model_3d_kreator): łańcuch kart od podstawy,
// ostatnie pole pyta „Co jest za…?” (element konstrukcyjny / miejsce łożyskowania osi), „+” między kartami wstawia
// element. Spasowanie: walec na poprzedniku ↔ walec na elemencie, opcjonalny styk czół, obrót i przesunięcie.
// Zmiany łańcucha są szkicem (Cofnij/Ponów) do „Zapisz model 3D”; status i oś pliku zapisują się od razu (plik).
type Kind = "structural" | "axis";
type Step = "file" | "parentAxis" | "elementAxis" | "contact" | "parentPlane" | "elementPlane" | "jointAxis" | "review";
type Session = { mode: "base" | "add" | "remate" | "reparent" | "axis"; at: number; kind: Kind | null; mesh: string | null; id: string;
    mate: Partial<Mate>; contact: boolean; step: Step; jointAxis: ElementAxis | null; radii: {parent?: number; element?: number} };
type Snapshot = { up: "z" | "y"; elements: ChainElement[] };
const fixed = (value: number, digits = 1) => value.toLocaleString("pl-PL", {maximumFractionDigits: digits, minimumFractionDigits: 0});
const vectorText = (value: THREE.Vector3) => (["X", "Y", "Z"] as const).map((axis, i) => Math.abs(value.getComponent(i)) >= .05 ? `${axis} ${value.getComponent(i) > 0 ? "+" : "−"}${fixed(Math.abs(value.getComponent(i)))}` : "").filter(Boolean).join(" · ") || "0";
const mm = (vector: THREE.Vector3) => vector.toArray().map(value => value * 1000);
const icons = {
    base: <svg viewBox="0 0 42 42" aria-hidden="true"><rect x="4" y="22" width="34" height="14" rx="2" fill="#2b3138" stroke="#8a97a3" strokeWidth="2"/></svg>,
    structural: <svg viewBox="0 0 42 42" aria-hidden="true"><rect x="15" y="4" width="12" height="34" rx="2" fill="#1e252c" stroke="#8a97a3" strokeWidth="2"/></svg>,
    axis: <svg viewBox="0 0 42 42" aria-hidden="true"><rect x="6" y="9" width="30" height="24" rx="6" fill="#132a3d" stroke="#2fa8ff" strokeWidth="2"/><circle cx="21" cy="21" r="7" fill="none" stroke="#ffd166" strokeWidth="2"/><circle cx="21" cy="21" r="2" fill="#ffd166"/></svg>,
    unset: <svg viewBox="0 0 42 42" aria-hidden="true"><rect x="8" y="8" width="26" height="26" rx="3" fill="none" stroke="#6e7983" strokeWidth="2" strokeDasharray="4 3"/></svg>,
};
const stepHints: Record<Step, string> = {file: "Wybierz plik elementu (lub wgraj nowy)", parentAxis: "Kliknij walec mocowania na poprzedniku (kołnierz, czop, otwór) — podświetlony",
    elementAxis: "Kliknij pasujący walec na nowym elemencie (zielony, obok łańcucha)", contact: "Dodaj styk czół albo pomiń — przesunięcie ustawisz polem",
    parentPlane: "Kliknij płaskie czoło styku na poprzedniku", elementPlane: "Kliknij płaskie czoło styku na elemencie", jointAxis: "Kliknij walec łożyskowania osi na elemencie",
    review: "Sprawdź położenie — obrót, przesunięcie, zwrot — i zatwierdź"};
function readGuide() { try { return localStorage.getItem("robot-model-guide") !== "0"; } catch { return true; } }

export function ModelChainBuilder({model, values, onDraft, onChainDraft, onRefresh, onNotice}: { model: RobotModel; values: Values; onDraft: (updates: Record<string, string>) => void;
    onChainDraft?: (chain: Pick<ModelChain, "up" | "elements">) => void; onRefresh: () => Promise<void>; onNotice: (notice: Notice) => void }) {
    const saved = model.chain ?? emptyChain();
    const [draft, setDraft] = useState<Snapshot>({up: saved.up, elements: saved.elements});
    const [base, setBase] = useState(saved.revision);
    const [past, setPast] = useState<Snapshot[]>([]), [future, setFuture] = useState<Snapshot[]>([]);
    const [sel, setSel] = useState(-1);
    const [session, setSession] = useState<Session | null>(null);
    const [guide, setGuide] = useState(readGuide);
    const [log, setLog] = useState<string[]>([]);
    const [busy, setBusy] = useState(false);
    const [units, setUnits] = useState("mm");
    const [showVirtual, setShowVirtual] = useState(true);
    const [geometries, setGeometries] = useState<Map<string, THREE.BufferGeometry>>(new Map());
    const host = useRef<HTMLDivElement>(null), scene = useRef<ReturnType<typeof createChainScene> | null>(null), fileInput = useRef<HTMLInputElement>(null);
    const [sceneError, setSceneError] = useState("");
    const meshes = useMemo(() => new Map(model.meshes.map(mesh => [mesh.id, mesh])), [model.meshes]);
    const dirty = JSON.stringify(draft) !== JSON.stringify({up: saved.up, elements: saved.elements});
    // Nowa rewizja z serwera (zapis w innym panelu) przy czystym szkicu → przyjęcie.
    useEffect(() => { if (saved.revision !== base && !dirty) { setDraft({up: saved.up, elements: saved.elements}); setBase(saved.revision); } }, [saved.revision]);  // eslint-disable-line react-hooks/exhaustive-deps
    const note = useCallback((text: string) => setLog(current => [...current.slice(-30), `${new Date().toLocaleTimeString("pl-PL")} · ${text}`]), []);
    function commit(next: Snapshot, text: string) { setPast(current => [...current.slice(-50), draft]); setFuture([]); setDraft(next); note(text); }
    function undo() { const previous = past.at(-1); if (!previous) return; setFuture(current => [draft, ...current]); setPast(current => current.slice(0, -1)); setDraft(previous); setSession(null); note("Cofnięto zmianę"); }
    function redo() { const next = future[0]; if (!next) return; setPast(current => [...current, draft]); setFuture(current => current.slice(1)); setDraft(next); setSession(null); note("Ponowiono zmianę"); }
    const roleOf = (id: string | null) => id ? meshes.get(id)?.element?.role ?? null : null;
    const nameOf = (id: string | null) => id ? meshes.get(id)?.name ?? "?" : "—";
    // Numery osi: modele osi w kolejności łańcucha.
    const jointNumber = useMemo(() => { let n = 0; return draft.elements.map(element => roleOf(element.mesh) === "axis" ? ++n : null); }, [draft.elements, meshes]);  // eslint-disable-line react-hooks/exhaustive-deps
    const axisCount = jointNumber.filter(Boolean).length;
    const label = (index: number) => index === 0 ? "Podstawa" : jointNumber[index] ? `J${jointNumber[index]}` : roleOf(draft.elements[index].mesh) === "structural" ? "Konstrukcyjny" : "Bez statusu";

    // Geometrie: elementy łańcucha i plik sesji (pamięć podręczna meshGeometry).
    const needed = useMemo(() => [...new Set([...draft.elements.map(element => element.mesh), ...(session?.mesh ? [session.mesh] : [])])].filter(id => meshes.get(id)?.state === "ready"), [draft.elements, session?.mesh, meshes]);
    useEffect(() => {
        let closed = false;
        for (const id of needed) if (!geometries.has(id)) void meshGeometry(id).then(geometry => { if (!closed) setGeometries(current => current.has(id) ? current : new Map(current).set(id, geometry)); })
            .catch(() => onNotice({source: "robot-chain", severity: "warning", message: `Nie wczytano siatki „${nameOf(id)}”`}));
        return () => { closed = true; };
    }, [needed.join(",")]);  // eslint-disable-line react-hooks/exhaustive-deps

    // Element sesji z aktualnym spasowaniem (gdy kompletne) i łańcuch podglądu.
    const mateReady = (mate: Partial<Mate>): mate is Mate => !!(mate.parent_point && mate.element_point);
    const sessionElement = useMemo((): ChainElement | null => {
        if (!session?.mesh) return null;
        const mate = mateReady(session.mate) ? session.mate as Mate : null;
        if (session.mode === "base") return {id: session.id, mesh: session.mesh, mate: null, placement: zeroPlacement()};
        if (session.mode === "axis") return null;
        const current = session.mode === "add" ? null : draft.elements[session.at];
        return mate ? withMate({id: session.id, mesh: session.mesh, mate: null, placement: zeroPlacement()}, {...mate, needs_parent: false}) :
            current ? {...current, mesh: session.mesh} : null;
    }, [session, draft.elements]);
    const preview = useMemo(() => {
        if (!session || !sessionElement) return draft;
        if (session.mode === "base") return {...draft, elements: [sessionElement, ...draft.elements.slice(session.mode === "base" && draft.elements.length ? 1 : 0)]};
        if (session.mode === "add") return {...draft, elements: [...draft.elements.slice(0, session.at), sessionElement, ...draft.elements.slice(session.at)]};
        return {...draft, elements: draft.elements.map((element, index) => index === session.at ? sessionElement : element)};
    }, [draft, session, sessionElement]);
    const previewIndex = session && session.mode !== "axis" ? (session.mode === "base" ? 0 : session.at) : -1;
    const staged = !!session && (session.mode === "add" || session.mode === "remate") && !!session.mesh && !mateReady(session.mate);
    const worlds = useMemo(() => worldMatrices(preview), [preview]);
    // Układy przegubów z łańcucha (układ bazy robota) i ich kontynuacja z konfiguracji (osie wirtualne).
    const frames = useMemo(() => chainFrames(draft, meshes, geometries), [draft, meshes, geometries]);
    const full = useMemo(() => frames.joints.length ? continueFrames(values, frames.joints.map(joint => joint.frame)) : null, [frames, values]);
    // Dodatni obrót słownie (układ bazy: X przód, Y lewo, Z góra) — do porównania z ruchem robota przy JOG „+”.
    // Punkt odniesienia: środek siatek członu napędzanego przez oś (rzeczywisty kształt ramienia); gdy leży na osi —
    // następny przegub (z modelu albo wirtualny).
    const motion = useMemo(() => {
        if (!full || !frames.base) return [];
        const toBase = frames.base.clone().invert(), worldsNow = worldMatrices(draft);
        return frames.joints.map(joint => {
            const at = new THREE.Vector3().setFromMatrixPosition(joint.frame), box = new THREE.Box3();
            draft.elements.forEach((element, index) => {
                const geometry = geometries.get(element.mesh);
                if (frames.links[index] === `link_${joint.joint}` && geometry?.boundingBox) box.union(geometry.boundingBox.clone().applyMatrix4(toBase.clone().multiply(worldsNow[index])));
            });
            const across = (point: THREE.Vector3) => { const r = point.clone().sub(at); return r.addScaledVector(joint.direction, -r.dot(joint.direction)); };
            let probe = box.isEmpty() ? new THREE.Vector3() : across(box.getCenter(new THREE.Vector3()));
            if (probe.length() < .01) probe = across(new THREE.Vector3().setFromMatrixPosition(joint.joint < 6 ? full.joints[joint.joint] : full.tool));
            return motionLabel(joint.direction, probe);
        });
    }, [frames, full, draft, geometries]);


    // Scena.
    useEffect(() => {
        let closed = false;
        void import("./chainScene").then(engine => {
            if (closed || !host.current) return;
            try { scene.current = engine.createChainScene(host.current); setSceneTick(tick => tick + 1); }
            catch { host.current.replaceChildren(); setSceneError("Podgląd 3D niedostępny — sprawdź obsługę WebGL"); }
        }).catch(() => { if (!closed) setSceneError("Nie udało się wczytać podglądu 3D"); });
        return () => { closed = true; scene.current?.dispose(); scene.current = null; };
    }, []);
    const [sceneTick, setSceneTick] = useState(0);
    const items = useMemo(() => {
        const list: ChainSceneItem[] = [];
        const box = new THREE.Box3();
        preview.elements.forEach((element, index) => {
            const geometry = geometries.get(element.mesh);
            if (!geometry || (staged && index === previewIndex)) return;
            const role = roleOf(element.mesh);
            list.push({key: element.id, geometry, matrix: worlds[index], kind: index === previewIndex ? "draft" : index === 0 ? "base" : role === "axis" ? "axis" : role === "structural" ? "structural" : "unset",
                selected: index === sel && !session, warn: !!element.mate?.needs_parent});
            if (geometry.boundingBox) box.union(geometry.boundingBox.clone().applyMatrix4(worlds[index]));
        });
        // Nowy element przed spasowaniem: obok łańcucha (do wskazania walca na nim).
        const geometry = session?.mesh ? geometries.get(session.mesh) : undefined;
        if (staged && geometry?.boundingBox) {
            const own = geometry.boundingBox, gap = box.isEmpty() ? 0 : Math.max(.03, box.getSize(new THREE.Vector3()).length() * .15);
            const offset = box.isEmpty() ? new THREE.Vector3() : new THREE.Vector3(box.max.x + gap - own.min.x, (box.min.y + box.max.y) / 2 - (own.min.y + own.max.y) / 2, box.min.z - own.min.z);
            list.push({key: session!.id, geometry, matrix: new THREE.Matrix4().makeTranslation(offset.x, offset.y, offset.z), kind: "staging"});
        }
        return list;
    }, [preview, worlds, geometries, sel, session, staged, previewIndex, meshes]);  // eslint-disable-line react-hooks/exhaustive-deps
    const matrixOf = (key: string) => items.find(item => item.key === key)?.matrix;
    useEffect(() => { scene.current?.setItems(items); }, [items, sceneTick]);
    useEffect(() => {
        const axes: ChainAxis[] = [], markers: ChainMarker[] = [];
        preview.elements.forEach((element, index) => {
            const editing = index === previewIndex || (session?.mode === "axis" && index === session.at);
            const mesh = meshes.get(element.mesh), axis = editing && session?.jointAxis ? session.jointAxis : mesh?.element?.role === "axis" ? mesh.element.axis : null;
            const geometry = geometries.get(element.mesh), matrix = matrixOf(element.id);
            if (!axis || !matrix) return;
            const length = Math.max(.06, (geometry?.boundingSphere?.radius ?? .05) * 2.4);
            let n = 0; for (let k = 0; k <= index; k++) if (roleOf(preview.elements[k].mesh) === "axis" || (k === previewIndex && session?.kind === "axis")) n++;
            axes.push({point: new THREE.Vector3(...axis.point_mm).multiplyScalar(.001).applyMatrix4(matrix), direction: new THREE.Vector3(...axis.direction).applyMatrix3(new THREE.Matrix3().setFromMatrix4(matrix)).normalize(),
                length, label: `J${n}`, draft: editing});
        });
        if (session) {
            const parent = session.mode === "base" ? null : preview.elements[session.mode === "add" ? session.at - 1 : session.at - 1];
            const parentMatrix = parent ? matrixOf(parent.id) : undefined, elementMatrix = matrixOf(session.id);
            const rotate = (matrix: THREE.Matrix4, direction: number[]) => new THREE.Vector3(...direction).applyMatrix3(new THREE.Matrix3().setFromMatrix4(matrix)).normalize();
            const at = (matrix: THREE.Matrix4, point: number[]) => new THREE.Vector3(...point).multiplyScalar(.001).applyMatrix4(matrix);
            const m = session.mate;
            if (parentMatrix && m.parent_point && m.parent_direction) markers.push({point: at(parentMatrix, m.parent_point), direction: rotate(parentMatrix, m.parent_direction), radius: session.radii.parent ?? .01, kind: "ring"});
            if (elementMatrix && m.element_point && m.element_direction) markers.push({point: at(elementMatrix, m.element_point), direction: rotate(elementMatrix, m.element_direction), radius: session.radii.element ?? .01, kind: "ring"});
            if (parentMatrix && m.parent_plane) markers.push({point: at(parentMatrix, m.parent_plane.point), direction: rotate(parentMatrix, m.parent_plane.normal), radius: (session.radii.parent ?? .01) * 1.4, kind: "disc"});
            if (elementMatrix && m.element_plane) markers.push({point: at(elementMatrix, m.element_plane.point), direction: rotate(elementMatrix, m.element_plane.normal), radius: (session.radii.element ?? .01) * 1.4, kind: "disc"});
        }
        // Brakujące osie (za ostatnią osią z modelu): przeguby i TCP z konfiguracji jako bryły wirtualne.
        let virtual: ChainVirtual | null = null;
        if (showVirtual && !session && full && frames.base && frames.joints.length < 6) {
            const k = frames.joints.length, toChain = (m: THREE.Matrix4) => frames.base!.clone().multiply(m);
            const position = (m: THREE.Matrix4) => new THREE.Vector3().setFromMatrixPosition(toChain(m));
            const length = axes.length ? axes.reduce((sum, axis) => sum + axis.length, 0) / axes.length : .12;
            virtual = {joints: [], segments: [], tcp: position(full.tool)};
            for (let n = k + 1; n <= 6; n++) {
                const matrix = toChain(full.joints[n - 1]), point = position(full.joints[n - 1]);
                const direction = new THREE.Vector3(...jointAxes[n - 1]).transformDirection(matrix);
                virtual.joints.push({point, direction}); virtual.segments.push([position(full.joints[n - 2]), point]);
                axes.push({point, direction, length: length * .6, label: `J${n} (wirt.)`, virtual: true});
            }
            virtual.segments.push([position(full.joints[5]), virtual.tcp!]);
        }
        scene.current?.setOverlay(axes, markers, virtual);
    }, [items, session, preview, meshes, sceneTick, full, showVirtual]);  // eslint-disable-line react-hooks/exhaustive-deps

    // Wskazania na podglądzie.
    const parentKey = session && session.mode !== "base" ? preview.elements[session.at - 1]?.id : undefined;
    const pickKeys = !session ? null : session.step === "parentAxis" || session.step === "parentPlane" ? (parentKey ? [parentKey] : null) :
        session.step === "elementAxis" || session.step === "elementPlane" || session.step === "jointAxis" ? [session.mode === "axis" ? draft.elements[session.at]?.id : session.id] : null;
    const handlePick = useCallback((pick: ChainPick) => {
        if (!session) return;
        try {
            const next: Session = {...session, mate: {...session.mate}, radii: {...session.radii}};
            const parentOnly = session.mode === "reparent";
            if (session.step === "parentAxis") {
                const fit = fitCylinder(pick.geometry, pick.face);
                next.mate.parent_point = mm(fit.point); next.mate.parent_direction = fit.direction.toArray(); next.radii.parent = fit.radius;
                if (parentOnly && next.mate.element_point) { next.mate.angle_deg = bestAngle({...defaults(next.mate), flip: !!next.mate.flip}); next.mate.parent_plane = null; }
                next.step = parentOnly ? (next.mate.element_plane ? "parentPlane" : "review") : "elementAxis";
            } else if (session.step === "elementAxis") {
                const fit = fitCylinder(pick.geometry, pick.face);
                next.mate.element_point = mm(fit.point); next.mate.element_direction = fit.direction.toArray(); next.radii.element = fit.radius;
                const parentMesh = preview.elements[session.at - 1]?.mesh, parentBox = parentMesh ? geometries.get(parentMesh)?.boundingBox : null, own = pick.geometry.boundingBox;
                const flip = parentBox && own ? defaultFlip(next.mate as Mate, parentBox.getCenter(new THREE.Vector3()), own.getCenter(new THREE.Vector3())) : false;
                const mate = {...defaults(next.mate), flip};
                next.mate = {...mate, angle_deg: bestAngle(mate)}; next.step = "contact";
            } else if (session.step === "parentPlane") {
                const plane = fitPlane(pick.geometry, pick.face);
                next.mate.parent_plane = {point: mm(plane.point), normal: plane.normal.toArray()};
                next.step = parentOnly ? "review" : "elementPlane";
            } else if (session.step === "elementPlane") {
                const plane = fitPlane(pick.geometry, pick.face);
                next.mate.element_plane = {point: mm(plane.point), normal: plane.normal.toArray()}; next.mate.offset_mm = 0;
                next.step = afterContact(next);
            } else if (session.step === "jointAxis") {
                next.jointAxis = axisFromFit(fitCylinder(pick.geometry, pick.face)); next.step = "review";
            }
            setSession(next);
        } catch (failure) { onNotice({source: "robot-chain", severity: "warning", message: failure instanceof Error ? failure.message : "Nie dopasowano powierzchni"}); }
    }, [session, preview, geometries, onNotice]);  // eslint-disable-line react-hooks/exhaustive-deps
    useEffect(() => {
        scene.current?.setPick(pickKeys, pickKeys ? handlePick : null);
        scene.current?.onSelect(session ? null : key => { const index = draft.elements.findIndex(element => element.id === key); setSel(index); });
    }, [pickKeys?.join(","), handlePick, session, draft.elements, sceneTick]);  // eslint-disable-line react-hooks/exhaustive-deps
    function defaults(mate: Partial<Mate>): Mate {
        return {parent_point: mate.parent_point!, parent_direction: mate.parent_direction!, element_point: mate.element_point!, element_direction: mate.element_direction!,
            flip: mate.flip ?? false, angle_deg: mate.angle_deg ?? 0, offset_mm: mate.offset_mm ?? 0, parent_plane: mate.parent_plane ?? null, element_plane: mate.element_plane ?? null};
    }
    function afterContact(s: Session): Step {
        const mesh = s.mesh ? meshes.get(s.mesh) : undefined;
        return s.kind === "axis" && !(mesh?.element?.role === "axis" && mesh.element.axis) ? "jointAxis" : "review";
    }

    // Sesje.
    function start(mode: Session["mode"], at: number, kind: Kind | null, mesh: string | null = null) {
        const current = draft.elements[at];
        const mate: Partial<Mate> = mode === "reparent" && current?.mate ? {...current.mate, parent_point: undefined, parent_direction: undefined, parent_plane: null} : {};
        const step: Step = mode === "base" ? (mesh ? "review" : "file") : mode === "reparent" ? "parentAxis" : mode === "axis" ? "jointAxis" : mesh ? "parentAxis" : "file";
        setSession({mode, at, kind, mesh: mesh ?? (mode === "reparent" || mode === "axis" ? current?.mesh ?? null : null), id: mode === "add" || mode === "base" ? newId() : current?.id ?? newId(),
            mate, contact: false, step, jointAxis: null, radii: {}});
        setSel(-1);
    }
    function chooseFile(id: string) {
        if (!session) return;
        setSession({...session, mesh: id, mate: session.mode === "remate" ? {} : session.mate, step: session.mode === "base" ? "review" : "parentAxis", jointAxis: null, radii: {}});
    }
    async function confirm() {
        if (!session?.mesh) return;
        const s = {...session, mesh: session.mesh}, mesh = meshes.get(s.mesh);
        setBusy(true);
        try {
            // Status i oś pliku (definicja elementu) — zapis od razu, przed zmianą szkicu łańcucha.
            const wantedRole: ElementRole | null = s.mode === "base" ? mesh?.element?.role ?? null : s.kind ?? mesh?.element?.role ?? null;
            const axis = s.jointAxis ?? (mesh?.element?.role === "axis" ? mesh.element.axis : null);
            if (mesh && (wantedRole !== (mesh.element?.role ?? null) || s.jointAxis)) {
                await saveElement(mesh.id, {role: wantedRole, axis: wantedRole === "axis" ? axis : null}, mesh.element?.revision ?? "initial");
                await onRefresh();
            }
            if (s.mode === "axis") { note(`Wskazano oś „${nameOf(s.mesh)}”`); setSession(null); setSel(s.at); return; }
            let elements = draft.elements, text = "", follow: Session | null = null;
            if (s.mode === "base") {
                const element: ChainElement = {id: s.id, mesh: s.mesh, mate: null, placement: zeroPlacement()};
                const changed = elements[0]?.mesh !== s.mesh;
                elements = [element, ...elements.slice(1)];
                if (changed && elements.length > 1) elements[1] = {...elements[1], mate: elements[1].mate && {...elements[1].mate, needs_parent: true}};
                text = `Podstawa: ${nameOf(s.mesh)}`;
            } else {
                const mate = {...defaults(s.mate), needs_parent: false};
                if (s.mode === "add") {
                    elements = insertElement(elements, s.at, withMate({id: s.id, mesh: s.mesh, mate: null, placement: zeroPlacement()}, mate));
                    text = `Dodano ${s.kind === "axis" ? "oś" : "element konstrukcyjny"} „${nameOf(s.mesh)}”${s.at < draft.elements.length ? ` (wstawiony na pozycję ${s.at + 1})` : ""}`;
                    if (s.at < draft.elements.length && guide && elements[s.at + 1].mate) follow = {mode: "reparent", at: s.at + 1, kind: null, mesh: elements[s.at + 1].mesh, id: elements[s.at + 1].id,
                        mate: {...elements[s.at + 1].mate!, parent_point: undefined, parent_direction: undefined, parent_plane: null}, contact: false, step: "parentAxis", jointAxis: null, radii: {}};
                } else {
                    const old = elements[s.at], updated = withMate({...old, mesh: s.mesh}, mate);
                    elements = elements.map((element, index) => index === s.at ? updated : element);
                    const next = elements[s.at + 1];
                    if (next && old.mesh !== s.mesh) elements[s.at + 1] = {...next, mate: next.mate && {...next.mate, needs_parent: true},
                        placement: matrixPlacement(placementMatrix(updated.placement).invert().multiply(placementMatrix(old.placement)).multiply(placementMatrix(next.placement)))};
                    text = s.mode === "reparent" ? `Wskazano mocowanie „${nameOf(s.mesh)}” na nowym poprzedniku` : `Spasowano ponownie „${nameOf(s.mesh)}”`;
                }
            }
            commit({...draft, elements}, text);
            setSession(follow); setSel(follow || guide ? -1 : s.mode === "add" ? s.at : s.at);
        } catch (failure) { onNotice({source: "robot-chain", severity: "error", message: failure instanceof Error ? failure.message : "Nie zapisano statusu pliku"}); }
        setBusy(false);
    }
    function editMate(index: number, change: Partial<Mate>, text: string) {
        const element = draft.elements[index];
        if (!element.mate) return;
        commit({...draft, elements: draft.elements.map((item, i) => i === index ? withMate(item, {...element.mate!, ...change}) : item)}, text);
    }
    function remove(index: number) {
        commit({...draft, elements: removeElement(draft.elements, index)}, `Usunięto „${nameOf(draft.elements[index].mesh)}”${index < draft.elements.length - 1 ? " — następny czeka na wskazanie mocowania" : ""}`);
        setSel(-1);
    }
    async function setRole(index: number, role: ElementRole) {
        const mesh = meshes.get(draft.elements[index].mesh);
        if (!mesh) return;
        try {
            await saveElement(mesh.id, {role, axis: role === "axis" ? mesh.element?.axis ?? null : null}, mesh.element?.revision ?? "initial");
            await onRefresh(); note(`Plik „${mesh.name}”: ${role === "axis" ? "miejsce łożyskowania osi" : "element konstrukcyjny"}`);
            if (role === "axis" && !mesh.element?.axis) start("axis", index, "axis");
        } catch (failure) { onNotice({source: "robot-chain", severity: "error", message: failure instanceof Error ? failure.message : "Nie zapisano statusu"}); }
    }
    // Zwrot osi pliku (wszystkich wystąpień) — zapis od razu; zmienia znak dodatniego kąta przegubu.
    async function flipAxis(index: number) {
        const mesh = meshes.get(draft.elements[index].mesh), axis = mesh?.element?.axis;
        if (!mesh || !axis) return;
        setBusy(true);
        try {
            const next = reverseAxis(axis);
            await saveElement(mesh.id, {role: "axis", axis: next}, mesh.element!.revision);
            await onRefresh(); note(`Zwrot osi „${mesh.name}”: ${next.reversed ? "odwrócony" : "jak wskazany walec"}`);
        } catch (failure) { onNotice({source: "robot-chain", severity: "error", message: failure instanceof Error ? failure.message : "Nie zapisano zwrotu osi"}); }
        setBusy(false);
    }
    async function save() {
        setBusy(true);
        try {
            const result: ModelChain = await saveChain({up: draft.up, elements: draft.elements}, base);
            setBase(result.revision); setDraft({up: result.up, elements: result.elements}); note("Zapisano model 3D");
            await onRefresh();
            onNotice({source: "robot-chain", severity: "info", message: "Łańcuch modelu zapisany. Podgląd na stronie sterowania pokaże go po jej ponownym otwarciu."});
        } catch (failure) { onNotice({source: "robot-chain", severity: "error", message: failure instanceof Error ? failure.message : "Nie zapisano łańcucha"}); }
        setBusy(false);
    }
    async function upload(files: FileList | null) {
        if (!files?.length) return;
        for (const file of Array.from(files)) {
            try { await uploadMesh(file, units); note(`Wgrano „${file.name}”`); }
            catch (failure) { onNotice({source: "robot-chain", severity: "error", message: `„${file.name}”: ${failure instanceof Error ? failure.message : "błąd wgrywania"}`}); }
        }
        await onRefresh();
    }
    const kin = useMemo(() => jointRows(draft, meshes), [draft, meshes]);
    useEffect(() => { onChainDraft?.(draft); }, [draft, onChainDraft]);
    // Układy przegubów z łańcucha → pola konfiguracji J1…Jk; porównanie z bieżącym szkicem.
    const fromChain = useMemo(() => frames.joints.length ? applyChainFrames(values, frames.joints.map(joint => joint.frame)) : null, [frames, values]);
    const baseDepth = frames.base && frames.joints[0] ? new THREE.Vector3().setFromMatrixPosition(frames.joints[0].frame).length() * 1000 : null;
    function transfer() {
        if (!fromChain) return;
        const k = frames.joints.length;
        onDraft(fromChain); note(`Przeniesiono układy J1–J${k} do szkicu konfiguracji`);
        onNotice({source: "robot-chain", severity: "info", message: `Układy J1–J${k} z łańcucha są w szkicu konfiguracji${k < 6 ? `; J${k + 1}–J6 i TCP zachowują wartości z konfiguracji względem poprzedniego przegubu (osie wirtualne)` : ""}. Zastosowanie w sterowaniu: „Zapisz do ROS”.${dirty ? " Zapisz też model 3D — podgląd sterowania korzysta z zapisanego łańcucha." : ""}`});
    }
    const needsParent = draft.elements.some(element => element.mate?.needs_parent);
    const missingAxis = draft.elements.filter(element => roleOf(element.mesh) === "axis" && !meshes.get(element.mesh)?.element?.axis).length;

    // ── Widok ──
    const hint = session ? stepHints[session.step] : guide && !draft.elements.length ? "Zacznij od podstawy — pole w łańcuchu poniżej" :
        guide && sel < 0 ? `Co jest za „${draft.elements.length ? nameOf(draft.elements.at(-1)!.mesh) : ""}”? — pole na końcu łańcucha` : "";
    return <section className="chain-builder" aria-label="Budowa modelu z elementów">
        <div className="chain-toolbar">
            <label className="chain-switch" title="Po każdym elemencie pyta o następny i podpowiada kliknięcia"><input type="checkbox" checked={guide}
                onChange={event => { setGuide(event.target.checked); try { localStorage.setItem("robot-model-guide", event.target.checked ? "1" : "0"); } catch { /* bez pamięci przeglądarki */ } }}/>Prowadź krok po kroku</label>
            <label className="chain-check" title="Przeguby za ostatnią osią z modelu i TCP z konfiguracji"><input type="checkbox" checked={showVirtual} onChange={event => setShowVirtual(event.target.checked)}/>Brakujące osie jako bryły wirtualne</label>
            <span className="chain-spacer"/>
            <button type="button" onClick={undo} disabled={!past.length || busy}>↶ Cofnij</button>
            <button type="button" onClick={redo} disabled={!future.length || busy}>↷ Ponów</button>
            <span className={`badge ${dirty ? "warn" : "ok"}`}>{dirty ? "Szkic · niezapisany" : "Zapisany"}</span>
            <button type="button" className="configuration-apply" onClick={() => void save()} disabled={!dirty || busy || !!session}>Zapisz model 3D</button>
            <button type="button" className="configuration-apply" onClick={transfer} disabled={!fromChain || needsParent || !!session || busy}
                title="Układy przegubów z łańcucha do szkicu konfiguracji; zastosowanie w sterowaniu — „Zapisz do ROS”">{frames.joints.length ? `Przenieś J1–J${frames.joints.length} do szkicu konfiguracji` : "Przenieś do szkicu konfiguracji"}</button>
        </div>
        <div className="chain-top">
            <div className="chain-viewport preview-viewport"><div className="preview-canvas" ref={host}/>
                {sceneError && <p className="preview-overlay" role="status">{sceneError}</p>}
                {hint && <span className="configuration-pick-hint" role="status">{hint}</span>}
                <button className="preview-home" aria-label="Dopasuj widok" title="Dopasuj widok" onClick={() => scene.current?.fit()}><svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.6" aria-hidden="true"><path d="M3 11 12 3l9 8M5 10v11h5v-7h4v7h5V10"/></svg></button>
                <span className="configuration-legend">Pozycja zerowa · żółte: osie (strzałka = dodatni obrót) · zielone: nowy element · pomarańczowe: wskazania spasowania</span>
            </div>
            <div className="chain-inspector">{session ? sessionPanel() : selectionPanel()}</div>
        </div>
        <div className="chain-strip" role="group" aria-label="Łańcuch elementów od podstawy">{strip()}</div>
        <div className="chain-bottom">
            <section className="chain-box"><h3>Kinematyka z łańcucha</h3>
                {frames.joints.length && fromChain ? <div className="robot-model-table-scroll"><table className="robot-model-table chain-kin"><thead><tr><th>Oś</th><th>Kierunek (baza)</th><th>Dodatni obrót</th><th>Położenie względem poprzedniego [mm]</th><th>Obrót r/p/y [°]</th><th>Szkic konfiguracji teraz</th><th>Przez elementy</th></tr></thead>
                    <tbody>{frames.joints.map(joint => {
                        const keys = originKeys.map(key => `joint.${joint.joint}.${key}`), next = keys.map(key => Number(fromChain[key])), now = keys.map(key => values[key]);
                        const differs = keys.some((_, i) => typeof now[i] !== "number" || Math.abs((now[i] as number) - next[i]) > (i < 3 ? .05 : .01));
                        const row = kin.rows.find(item => item.index === joint.index);
                        return <tr key={joint.joint} data-differs={differs}><td><span className="chain-chip">J{joint.joint}</span></td><td>{directionLabel(joint.direction)}{joint.snapped ? <small>przyciągnięta (&lt; 1°)</small> : null}</td>
                            <td>{motion[frames.joints.indexOf(joint)] ? `człon ${motion[frames.joints.indexOf(joint)]}` : "—"}{joint.flipped ? <small>odwrócony względem konwencji</small> : null}</td>
                            <td>{next.slice(0, 3).map(value => fixed(value, 2)).join(" / ")}</td><td>{next.slice(3).map(value => fixed(value, 3)).join(" / ")}</td>
                            <td>{differs ? <small>{now.slice(0, 3).map(value => typeof value === "number" ? fixed(value, 2) : "—").join(" / ")} mm · {now.slice(3).map(value => typeof value === "number" ? fixed(value, 2) : "—").join(" / ")}°</small> : <small className="chain-same">zgodny</small>}</td>
                            <td><small>{row?.via.join(" + ") || "—"}</small></td></tr>;
                    })}
                    {frames.joints.length < 6 && [...Array.from({length: 6 - frames.joints.length}, (_, i) => frames.joints.length + i + 1).map(n => ({label: `J${n}`, keys: originKeys.map(key => `joint.${n}.${key}`)})),
                        {label: "TCP", keys: tcpKeys.map(key => `tool.${key}`)}].map(row => {
                        const now = row.keys.map(key => values[key]);
                        return <tr key={row.label} data-virtual="true"><td><span className="chain-chip virtual">{row.label}</span></td><td><small>wirtualna</small></td><td>—</td>
                            <td>{now.slice(0, 3).map(value => typeof value === "number" ? fixed(value, 2) : "—").join(" / ")}</td><td>{now.slice(3).map(value => typeof value === "number" ? fixed(value, 3) : "—").join(" / ")}</td>
                            <td><small>z konfiguracji (bez zmian)</small></td><td><small>—</small></td></tr>;
                    })}</tbody></table></div>
                    : <p className="chain-muted">{frames.missingAxis ? `J${frames.missingAxis}: model osi bez wskazanego walca łożyskowania.` : "Brak osi ze wskazanym walcem łożyskowania."}</p>}
                <p className="chain-muted">Zero robota: na osi J1, {baseDepth === null ? "—" : `${fixed(baseDepth, 2)} mm pod punktem osi J1`} ({frames.baseFromMesh ? "najniższy punkt podstawy" : "brak siatek podstawy"}). Układ Jn: J1 w punkcie wskazanego walca, kolejne — rzut początku poprzedniego układu na oś (osie równoległe → czyste przesunięcie). Element z osią obraca się z nią i zaczyna człon; element konstrukcyjny liczy się do kinematyki, gdy za nim jest oś{kin.after.length ? `; za ostatnią osią: ${kin.after.join(" + ")} (przesuwa TCP)` : ""}.{frames.joints.length && frames.joints.length < 6 ? ` Brakujące osie J${frames.joints.length + 1}–J6 i TCP — wirtualne (fioletowe bryły), z wartości konfiguracji względem poprzedniego przegubu; przeniesienie zmienia tylko J1–J${frames.joints.length}.` : ""}</p>
                {(axisCount > 6 || needsParent || missingAxis > 0) && <p className="chain-warning">{[axisCount > 6 ? `Łańcuch ma ${axisCount} osi — robot ma 6.` : "", needsParent ? "Są elementy czekające na wskazanie mocowania (⚠) — przeniesienie do szkicu zablokowane." : "", missingAxis ? `Modele osi bez wskazanej osi: ${missingAxis}.` : ""].filter(Boolean).join(" ")}</p>}
            </section>
            <section className="chain-box"><h3>Historia zmian</h3>
                {log.length ? <ol className="chain-log">{log.slice(-8).map((item, i) => <li key={i}>{item}</li>)}</ol> : <p className="chain-muted">Brak zmian w tej sesji. Cofnij/Ponów obejmuje zmiany łańcucha (status i oś pliku zapisują się od razu).</p>}
            </section>
        </div>
    </section>;

    function strip() {
        const cards: ReactElement[] = [];
        draft.elements.forEach((element, index) => {
            if (index > 0) cards.push(<div className="chain-ins" key={`ins-${index}`}><button type="button" title="Wstaw element tutaj" disabled={!!session} onClick={() => start("add", index, "structural")}>+</button></div>);
            const role = roleOf(element.mesh), kind = index === 0 ? "base" : role ?? "unset";
            const axis = meshes.get(element.mesh)?.element?.axis;
            let detail = "";
            if (index === 0) detail = `oś „w górę”: ${draft.up.toUpperCase()}`;
            else if (role === "axis") detail = axis ? "oś wskazana" : "⚠ oś nie wskazana";
            else if (role === "structural") { const next = jointNumber.findIndex((n, k) => k > index && n); detail = next >= 0 ? `do kinematyki (do J${jointNumber[next]})` : "za ostatnią osią — przesuwa TCP"; }
            else detail = "plik bez statusu";
            const link = frames.links[index];
            if (index > 0) detail = `${link === "base" ? "podstawa" : `człon ${link.slice(5)}`} · ${detail}`;
            cards.push(<button type="button" key={element.id} className={`chain-node ${kind}${element.mate?.needs_parent ? " warn" : ""}`} aria-current={index === sel && !session} disabled={!!session}
                onClick={() => setSel(index === sel ? -1 : index)}>
                <span className="chain-node-head">{icons[kind as keyof typeof icons]}<strong>{label(index)}</strong></span>
                <span className="chain-node-file" title={nameOf(element.mesh)}>{nameOf(element.mesh)}</span>
                <small>{element.mate?.needs_parent ? "⚠ wskaż mocowanie na poprzedniku" : detail}</small></button>);
        });
        if (session?.mode === "add" || session?.mode === "base") {
            const draftCard = <div key="draft" className="chain-node draft"><span className="chain-node-head">{icons[session.mode === "base" ? "base" : session.kind ?? "unset"]}<strong>{session.mode === "base" ? "Podstawa" : session.kind === "axis" ? `J${jointNumber.slice(0, session.at).filter(Boolean).length + 1}` : "Nowy element"}</strong></span>
                <span className="chain-node-file">{session.mesh ? nameOf(session.mesh) : "wybierz plik…"}</span><small>w trakcie dodawania…</small></div>;
            const position = session.mode === "base" ? 0 : session.at * 2;
            cards.splice(position, 0, draftCard);
        } else if (!session) {
            if (draft.elements.length) cards.push(<div className="chain-ins" key="ins-end"/>);
            const n = axisCount + 1, last = draft.elements.at(-1);
            cards.push(!draft.elements.length ? <div key="next" className="chain-next"><span className="chain-q">Od czego zaczynamy?</span>
                    <div className="chain-opts"><button type="button" onClick={() => start("base", 0, null)}>{icons.base}Podstawa</button></div>
                    <small>Podstawa jest początkiem łańcucha; jej plik wyznacza układ podglądu.</small></div>
                : guide ? <div key="next" className="chain-next"><span className="chain-q">Co jest za „{nameOf(last!.mesh)}”?</span>
                    <div className="chain-opts"><button type="button" onClick={() => start("add", draft.elements.length, "structural")}>{icons.structural}Konstrukcyjny</button>
                        <button type="button" onClick={() => start("add", draft.elements.length, "axis")}>{icons.axis}Oś J{n}</button></div>
                    <small>{n > 6 ? "Robot ma 6 osi — dalej tylko narzędzie (konstrukcyjny, przesuwa TCP)." : `Osie: ${n - 1}/6. Element konstrukcyjny liczy się do kinematyki, gdy dodasz za nim oś.`}</small></div>
                : <button type="button" key="next" className="chain-next idle" onClick={() => setGuide(true)}><span>+</span>Dodaj element</button>);
        }
        return cards;
    }

    function sessionPanel() {
        const s = session!, parent = s.mode === "base" ? null : preview.elements[s.at - 1];
        const title = {base: "Podstawa", add: s.kind === "axis" ? "Nowe miejsce łożyskowania osi" : "Nowy element konstrukcyjny", remate: "Spasowanie od nowa", reparent: "Mocowanie na nowym poprzedniku", axis: "Oś przegubu"}[s.mode];
        const candidates = model.meshes.filter(mesh => s.mode === "base" || s.kind === null || (mesh.element?.role ?? null) !== (s.kind === "axis" ? "structural" : "axis"));
        const others = model.meshes.filter(mesh => !candidates.includes(mesh));
        const mesh = s.mesh ? meshes.get(s.mesh) : undefined, mate = mateReady(s.mate) ? defaults(s.mate) : null;
        const contact = mate ? contactOffset(mate) : null;
        const stepsDone = (step: Step) => {
            const order: Step[] = ["file", "parentAxis", "elementAxis", "contact", "parentPlane", "elementPlane", "jointAxis", "review"];
            return order.indexOf(s.step) > order.indexOf(step);
        };
        const pickRow = (step: Step, text: string, done: string | null) => <div className={`pick ${done ? "done" : ""}${s.step === step ? " current" : ""}`}><span>{done ?? text}</span>
            {(done || s.step === step) && <button type="button" onClick={() => setSession({...s, step})}>{done ? "Wskaż ponownie" : "Czekam na klik…"}</button>}</div>;
        const roleChange = mesh && s.kind && s.mode !== "base" && (mesh.element?.role ?? null) !== s.kind;
        return <>
            <header><span className="eyebrow">{s.mode === "add" ? `Element ${s.at + 1}` : s.mode === "base" ? "Początek łańcucha" : nameOf(s.mesh)}</span><strong>{title}</strong></header>
            <div className="chain-inspector-body">
                {(s.mode === "base" || s.mode === "add" || s.mode === "remate") && <div className="chain-file">
                    <label>Plik<select value={s.mesh ?? ""} onChange={event => chooseFile(event.target.value)}>
                        <option value="" disabled>— wybierz plik —</option>
                        {candidates.map(item => <option key={item.id} value={item.id} disabled={item.state !== "ready"}>{item.name}{item.state !== "ready" ? " (przetwarzanie…)" : item.element?.role === "axis" ? " · model osi" : item.element?.role === "structural" ? " · konstrukcyjny" : ""}</option>)}
                        {others.length > 0 && <optgroup label="Inny status (zmieni się)">{others.map(item => <option key={item.id} value={item.id} disabled={item.state !== "ready"}>{item.name}</option>)}</optgroup>}</select></label>
                    <div className="row"><input ref={fileInput} hidden type="file" multiple accept=".stl,.obj" onChange={event => { void upload(event.target.files); event.target.value = ""; }}/>
                        <button type="button" onClick={() => fileInput.current?.click()}>⬆ Wgraj STL/OBJ…</button>
                        <select aria-label="Jednostki pliku" value={units} onChange={event => setUnits(event.target.value)}><option value="mm">mm</option><option value="cm">cm</option><option value="m">m</option><option value="in">cale</option></select></div>
                    {roleChange && <p className="chain-note">Status pliku „{mesh!.name}” zmieni się na „{s.kind === "axis" ? "model osi" : "element konstrukcyjny"}” po zatwierdzeniu.</p>}
                </div>}
                {s.mode === "base" && <label className="chain-field">Oś „w górę” w pliku podstawy<select value={draft.up} onChange={event => commit({...draft, up: event.target.value as "z" | "y"}, `Oś „w górę”: ${event.target.value.toUpperCase()}`)}>
                    <option value="z">Z (jak robot)</option><option value="y">Y (Fusion „Y up”)</option></select></label>}
                {s.mesh && s.mode !== "base" && s.mode !== "axis" && <>
                    <p className="chain-muted">Dołączenie do „{nameOf(parent?.mesh ?? null)}”</p>
                    {pickRow("parentAxis", "1. Walec mocowania na poprzedniku", s.mate.parent_point ? `poprzednik: walec R ${fixed((s.radii.parent ?? 0) * 1000)} mm` : null)}
                    {s.mode !== "reparent" && pickRow("elementAxis", "2. Pasujący walec na elemencie", s.mate.element_point && stepsDone("elementAxis") ? `element: walec R ${fixed((s.radii.element ?? 0) * 1000)} mm` : null)}
                    {s.step === "contact" && <div className="pick current"><span>3. Styk czół (dosunięcie wzdłuż osi)</span><span className="row">
                        <button type="button" onClick={() => setSession({...s, step: "parentPlane"})}>Wskaż czoła</button>
                        <button type="button" onClick={() => setSession({...s, mate: {...s.mate, parent_plane: null, element_plane: null}, step: afterContact(s)})}>Pomiń</button></span></div>}
                    {(s.step === "parentPlane" || s.mate.parent_plane) && pickRow("parentPlane", "Czoło na poprzedniku", s.mate.parent_plane ? "czoło na poprzedniku" : null)}
                    {s.mode !== "reparent" && (s.step === "elementPlane" || s.mate.element_plane) && pickRow("elementPlane", "Czoło na elemencie", s.mate.element_plane && s.mate.parent_plane ? "czoło na elemencie" : null)}
                </>}
                {s.kind === "axis" && s.mesh && (s.step === "jointAxis" || s.jointAxis || (mesh?.element?.axis && s.mode !== "axis")) &&
                    pickRow("jointAxis", "Walec łożyskowania osi na elemencie", s.jointAxis ? `oś: walec R ${fixed(s.jointAxis.radius_mm)} mm, zasięg ${fixed(s.jointAxis.coverage_deg, 0)}°` : mesh?.element?.axis ? `oś z pliku: R ${fixed(mesh.element.axis.radius_mm)} mm` : null)}
                {mate && <div className="chain-adjust">
                    <label className="chain-field">Obrót wokół osi [°]<span className="row"><input type="number" step="1" key={`sa-${mate.angle_deg}`} defaultValue={Math.round(mate.angle_deg * 100) / 100} onBlur={event => { const value = Number(event.target.value); if (event.target.value !== "" && Number.isFinite(value)) setSession({...s, mate: {...s.mate, angle_deg: value}}); }}/>
                        <button type="button" onClick={() => setSession({...s, mate: {...s.mate, angle_deg: ((mate.angle_deg + 90 + 180) % 360) - 180}})}>+90°</button></span></label>
                    <label className="chain-field">Przesunięcie wzdłuż osi [mm]<input type="number" step="0.1" key={`so-${mate.offset_mm}`} defaultValue={mate.offset_mm} onBlur={event => { const value = Number(event.target.value); if (event.target.value !== "" && Number.isFinite(value)) setSession({...s, mate: {...s.mate, offset_mm: value}}); }}/></label>
                    <div className="row"><button type="button" onClick={() => { const flipped = {...mate, flip: !mate.flip}; setSession({...s, mate: {...s.mate, flip: !mate.flip, angle_deg: bestAngle(flipped)}}); }}>Odwróć element</button>
</div>
                    {s.jointAxis && <label className="chain-check"><input type="checkbox" checked={!!s.jointAxis.reversed} onChange={() => setSession({...s, jointAxis: reverseAxis(s.jointAxis!)})}/>Odwrócony zwrot osi (strzałka na podglądzie = dodatni obrót)</label>}
                    {contact !== null && <small className="chain-muted">Styk czół: {fixed(contact, 2)} mm wzdłuż osi{mate.parent_plane && planeTiltDeg(mate.parent_plane, mate.parent_direction) > 5 ? " · ⚠ czoło poprzednika nie jest prostopadłe do osi" : ""}{mate.element_plane && planeTiltDeg(mate.element_plane, mate.element_direction) > 5 ? " · ⚠ czoło elementu nie jest prostopadłe do osi" : ""}</small>}
                </div>}
                {s.mode === "axis" && <p className="chain-muted">Oś zapisze się w pliku „{nameOf(s.mesh)}” — każde wystąpienie tego pliku ma ją od razu.</p>}
            </div>
            <footer><button type="button" onClick={() => setSession(null)} disabled={busy}>Anuluj</button><span className="chain-spacer"/>
                <button type="button" className="configuration-apply" onClick={() => void confirm()} disabled={busy || s.step !== "review" || !s.mesh}>{s.mode === "add" ? "Dodaj do łańcucha" : s.mode === "base" ? "Ustaw podstawę" : "Zatwierdź"}</button></footer>
        </>;
    }

    function selectionPanel() {
        if (sel < 0 || !draft.elements[sel]) return <>
            <header><strong>Element</strong></header>
            <div className="chain-inspector-body chain-muted">Wybierz kartę w łańcuchu albo element na podglądzie.<br/><br/>Nowy element: pole „Co jest za…?” na końcu łańcucha albo „+” między kartami (wstawienie).</div>
        </>;
        const element = draft.elements[sel], mesh = meshes.get(element.mesh), role = roleOf(element.mesh), mate = element.mate;
        return <>
            <header>{jointNumber[sel] ? <span className="chain-chip">J{jointNumber[sel]}</span> : null}<strong title={mesh?.name}>{sel === 0 ? "Podstawa" : mesh?.name}</strong>
                <span className="chain-spacer"/><span className="badge">{sel === 0 ? "podstawa" : role === "axis" ? "Miejsce łożyskowania osi" : role === "structural" ? "Element konstrukcyjny" : "bez statusu"}</span></header>
            <div className="chain-inspector-body">
                <dl className="chain-props"><dt>Plik</dt><dd>{mesh?.name ?? "?"} · {mesh?.triangles?.toLocaleString("pl-PL")} trójk.</dd>
                    {sel > 0 && <><dt>Dołączony do</dt><dd>{nameOf(draft.elements[sel - 1].mesh)}{mate?.parent_plane ? " · styk czół" : ""}</dd></>}
                    {role === "axis" && <><dt>Oś</dt><dd>{mesh?.element?.axis ? `walec R ${fixed(mesh.element.axis.radius_mm)} mm · zasięg ${fixed(mesh.element.axis.coverage_deg, 0)}°` : "nie wskazana"}</dd></>}
                    {sel > 0 && <><dt>Położenie</dt><dd>{(["x", "y", "z"] as const).map(key => fixed(element.placement[key])).join(" / ")} mm · {(["roll", "pitch", "yaw"] as const).map(key => fixed(element.placement[key])).join(" / ")}°</dd></>}</dl>
                {role === "axis" && mesh?.element?.axis && (() => {
                    const joint = frames.joints.find(item => item.index === sel), k = joint ? frames.joints.indexOf(joint) : -1;
                    return <div className="chain-direction">
                        <label className="chain-check"><input type="checkbox" checked={joint ? joint.flipped : !!mesh.element.axis.reversed} disabled={busy} onChange={() => void flipAxis(sel)}/>
                            Odwrócony kierunek dodatniego obrotu{jointNumber[sel] ? ` J${jointNumber[sel]}` : ""}</label>
                        {joint && <small>Dodatni obrót: <strong>{motion[k] ? `człon ${motion[k]}` : "—"}</strong> (strzałka na podglądzie). Konwencja bez odwrócenia: J1 — przód w lewo, J2/J3/J5 — do przodu / w dół. Porównaj z ruchem robota przy JOG „›”; niezgodność usuwa to pole albo kierunek osi w kalibracji.</small>}
                    </div>;
                })()}
                {mate?.needs_parent && <div className="chain-warning">Poprzednik się zmienił — element trzyma dawne miejsce w przestrzeni. <button type="button" onClick={() => start("reparent", sel, null)}>Wskaż mocowanie na poprzedniku</button></div>}
                {sel === 0 && <label className="chain-field">Oś „w górę” w pliku podstawy<select value={draft.up} onChange={event => commit({...draft, up: event.target.value as "z" | "y"}, `Oś „w górę”: ${event.target.value.toUpperCase()}`)}>
                    <option value="z">Z (jak robot)</option><option value="y">Y (Fusion „Y up”)</option></select></label>}
                {mate && !mate.needs_parent && <div className="chain-adjust">
                    <label className="chain-field">Obrót wokół osi [°]<span className="row"><input type="number" step="1" key={`a-${element.id}-${mate.angle_deg}`} defaultValue={mate.angle_deg}
                        onBlur={event => { const value = Number(event.target.value); if (event.target.value !== "" && Number.isFinite(value) && value !== mate.angle_deg) editMate(sel, {angle_deg: value}, `Obrót „${mesh?.name}”: ${value}°`); }}/>
                        <button type="button" onClick={() => editMate(sel, {angle_deg: ((mate.angle_deg + 90 + 180) % 360) - 180}, `Obrót „${mesh?.name}” +90°`)}>+90°</button></span></label>
                    <label className="chain-field">Przesunięcie wzdłuż osi [mm]<input type="number" step="0.1" key={`o-${element.id}-${mate.offset_mm}`} defaultValue={mate.offset_mm}
                        onBlur={event => { const value = Number(event.target.value); if (event.target.value !== "" && Number.isFinite(value) && value !== mate.offset_mm) editMate(sel, {offset_mm: value}, `Przesunięcie „${mesh?.name}”: ${value} mm`); }}/></label>
                    <div className="row"><button type="button" onClick={() => editMate(sel, {flip: !mate.flip, angle_deg: bestAngle({...mate, flip: !mate.flip})}, `Odwrócono „${mesh?.name}”`)}>Odwróć element</button>
                        <button type="button" onClick={() => start("remate", sel, role === "axis" || role === "structural" ? role : null, element.mesh)}>Spasuj ponownie</button></div>
                </div>}
                <div className="row">
                    {sel > 0 && role !== "axis" && <button type="button" onClick={() => void setRole(sel, "axis")}>→ Miejsce osi</button>}
                    {sel > 0 && role !== "structural" && <button type="button" onClick={() => void setRole(sel, "structural")}>→ Konstrukcyjny</button>}
                    {role === "axis" && <button type="button" onClick={() => start("axis", sel, "axis")}>{mesh?.element?.axis ? "Wskaż oś ponownie" : "Wskaż oś"}</button>}
                    {sel === 0 ? <button type="button" onClick={() => start("base", 0, null)}>Zamień plik podstawy…</button> : <button type="button" onClick={() => start("remate", sel, role === "axis" || role === "structural" ? role : null)}>Zamień plik…</button>}
                </div>
                <p className="chain-muted">Status i oś dotyczą pliku (wszystkich jego wystąpień) i zapisują się od razu; położenie — szkic do „Zapisz model 3D”.</p>
            </div>
            <footer><button type="button" className="chain-danger" onClick={() => remove(sel)}>Usuń</button><span className="chain-spacer"/><span className="chain-muted">zmiany od razu w szkicu</span></footer>
        </>;
    }
}
