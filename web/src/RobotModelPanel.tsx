import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { Notice } from "./eventLog";
import { moduleNotice, useCondition } from "./messageCenter";
import { assemblyMatrix, deleteMesh, emptyAssembly, fetchRobotModel, loadAssemblyMeshes, modelLinkLabels, modelLinks, saveAssembly, uploadMesh,
    type Assembly, type AssemblyTransform, type MeshInfo, type ModelLink, type RobotModel } from "./robotModel";
import { fitCylinder, type CylinderFit } from "./axisFit";
import { applyJointAxis, applyTcpPoint, geometryChecks, lowestAlong, rootUnderBase } from "./frameEdit";
import type { ModelPick, PickOverlay } from "./configurationScene";
import type { PreviewModel } from "./ConfigurationPreview";
import type { Values } from "./configurationData";

export type PickTarget = { kind: "joint"; joint: number } | { kind: "tcp-point" } | { kind: "tcp-center" };
type Pending = { target: PickTarget; fit?: CylinderFit; point: import("three").Vector3; mesh: string; reverse: boolean };
const transformFields: [keyof AssemblyTransform, string, string][] = [["x", "X", "mm"], ["y", "Y", "mm"], ["z", "Z", "mm"], ["roll", "Roll", "°"], ["pitch", "Pitch", "°"], ["yaw", "Yaw", "°"]];
const stateLabels: Record<MeshInfo["state"], string> = {queued: "w kolejce", processing: "przetwarzanie…", ready: "wczytana", failed: "błąd"};
const assignmentOf = (assembly: Assembly, id: string) => modelLinks.find(link => assembly.links[link].includes(id)) ?? "";
const comparable = (assembly: Assembly) => JSON.stringify({up: assembly.up, transform: assembly.transform, links: assembly.links});
const fixed = (value: number | undefined, digits = 2) => value === undefined ? "—" : value.toLocaleString("pl-PL", {maximumFractionDigits: digits});

// Model 3D z CAD (Fusion 360: człony jako komponenty, pozycja zerowa, wspólny układ złożenia) i konfiguracja układów
// przegubów przez wskazanie powierzchni walcowych na siatkach. Wynik wskazania trafia do szkicu konfiguracji;
// zastosowanie w sterowaniu — przez „Zapisz do ROS” (jak każda zmiana geometrii).
export function RobotModelPanel({values, onDraft, onNotice, onPreview, onPicking}: {
    values: Values; onDraft: (updates: Record<string, string>) => void; onNotice: (notice: Notice) => void;
    onPreview: (preview: {model: PreviewModel | null; display: {safety: boolean; primitives: boolean}; overlay: PickOverlay; onPick: ((pick: ModelPick) => void) | null; hint: string}) => void;
    onPicking?: (picking: boolean) => void;
}) {
    const [model, setModel] = useState<RobotModel | null>(null);
    const [assembly, setAssembly] = useState<Assembly>(emptyAssembly);
    const [error, setError] = useState("");
    const [units, setUnits] = useState("mm");
    const [busy, setBusy] = useState(false);
    const [display, setDisplay] = useState({safety: false, primitives: false});
    const [target, setTarget] = useState<PickTarget | null>(null);
    const [pending, setPending] = useState<Pending | null>(null);
    const [preview, setPreviewModel] = useState<PreviewModel | null>(null);
    const fileInput = useRef<HTMLInputElement>(null);
    const dirty = !!model && comparable(assembly) !== comparable(model.assembly);
    const refresh = useCallback(async (adopt = false) => {
        try {
            const next = await fetchRobotModel(); setModel(next); setError("");
            setAssembly(current => adopt || current.revision === "initial" && comparable(current) === comparable(emptyAssembly()) ? next.assembly : current);
        } catch (failure) { setError(failure instanceof Error ? failure.message : "Model 3D niedostępny"); }
    }, []);
    useEffect(() => { void refresh(true); }, [refresh]);
    useCondition("config-model", error ? moduleNotice({ source: "config-model", severity: "warning", title: "Model 3D niedostępny", message: error }, "Konfiguracja · model 3D", "#config") : null);
    const processing = !!model?.meshes.some(mesh => mesh.state === "queued" || mesh.state === "processing" || mesh.safety.state === "queued" || mesh.safety.state === "processing");
    useEffect(() => {
        if (!processing) return;
        const timer = setInterval(() => void refresh(), 1500);
        return () => clearInterval(timer);
    }, [processing, refresh]);
    // Podgląd: przypisania i przekształcenie ze szkicu modelu (bez zapisu), geometrie pobierane raz.
    const loadKey = model ? JSON.stringify([assembly.links, display.safety, model.meshes.map(mesh => [mesh.id, mesh.state, mesh.safety.state])]) : "";
    useEffect(() => {
        if (!model) return;
        let closed = false;
        void loadAssemblyMeshes({assembly, meshes: model.meshes}, display.safety).then(items => {
            if (!closed) setPreviewModel({items, matrix: assemblyMatrix(assembly)});
        }).catch(failure => onNotice({source: "robot-model", severity: "warning", message: `Nie wczytano siatek modelu 3D: ${failure instanceof Error ? failure.message : "błąd"}`}));
        return () => { closed = true; };
    }, [loadKey]);  // eslint-disable-line react-hooks/exhaustive-deps
    const matrixKey = JSON.stringify([assembly.up, assembly.transform]);
    useEffect(() => { setPreviewModel(current => current && {items: current.items, matrix: assemblyMatrix(assembly)}); }, [matrixKey]);  // eslint-disable-line react-hooks/exhaustive-deps

    const handlePick = useCallback((pick: ModelPick) => {
        if (!target) return;
        try {
            if (target.kind === "tcp-point") { setPending({target, point: pick.point, mesh: pick.name, reverse: false}); return; }
            const fit = fitCylinder(pick.geometry, pick.face, pick.matrix);
            setPending({target, fit, point: fit.point, mesh: pick.name, reverse: false});
        } catch (failure) {
            onNotice({source: "robot-model-pick", severity: "warning", message: failure instanceof Error ? failure.message : "Nie dopasowano osi"});
        }
    }, [target, onNotice]);
    const edit = useMemo(() => pending?.target.kind === "joint" && pending.fit ? applyJointAxis(values, pending.target.joint, pending.fit.point, pending.fit.direction, pending.reverse) : null, [pending, values]);
    const overlay: PickOverlay = useMemo(() => pending ? {axis: pending.fit && pending.target.kind === "joint" ? {point: pending.fit.point, direction: edit?.direction ?? pending.fit.direction, radius: pending.fit.radius} : null,
        point: pending.target.kind !== "joint" ? pending.point : null} : {}, [pending, edit]);
    const pickLabel = target ? target.kind === "joint" ? `Kliknij powierzchnię walcową osi J${target.joint} (czop, otwór, gniazdo łożyska)` :
        target.kind === "tcp-point" ? "Kliknij punkt TCP na narzędziu lub kołnierzu" : "Kliknij powierzchnię walcową — TCP w jej środku na osi" : "";
    useEffect(() => {
        onPreview({model: preview, display: {safety: display.safety, primitives: display.primitives || !preview?.items.length}, overlay,
            onPick: target ? handlePick : null, hint: pending ? "" : pickLabel});
    }, [preview, display, overlay, target, handlePick, pending, pickLabel, onPreview]);
    useEffect(() => { onPicking?.(!!target); }, [target, onPicking]);

    async function upload(files: FileList | null) {
        if (!files?.length) return;
        setBusy(true);
        for (const file of Array.from(files)) {
            try { await uploadMesh(file, units); onNotice({source: "robot-model", severity: "info", message: `Wgrano „${file.name}” — trwa odczyt i generowanie siatki bezpieczeństwa.`}); }
            catch (failure) { onNotice({source: "robot-model", severity: "error", message: `„${file.name}”: ${failure instanceof Error ? failure.message : "błąd wgrywania"}`}); }
        }
        setBusy(false); await refresh();
    }
    async function save() {
        if (!model) return;
        setBusy(true);
        try {
            const saved = await saveAssembly({up: assembly.up, transform: assembly.transform, links: assembly.links}, model.assembly.revision);
            setAssembly(saved); await refresh(true);
            onNotice({source: "robot-model", severity: "info", message: "Model 3D zapisany. Podgląd robota pokaże go po ponownym otwarciu strony sterowania."});
        } catch (failure) { onNotice({source: "robot-model", severity: "error", message: failure instanceof Error ? failure.message : "Nie zapisano modelu 3D"}); }
        setBusy(false);
    }
    async function remove(mesh: MeshInfo) {
        try { await deleteMesh(mesh.id); await refresh(); }
        catch (failure) { onNotice({source: "robot-model", severity: "warning", message: failure instanceof Error ? failure.message : "Nie usunięto siatki"}); }
    }
    function assign(id: string, link: ModelLink | "") {
        setAssembly(current => ({...current, links: Object.fromEntries(modelLinks.map(item => [item, [...current.links[item].filter(value => value !== id), ...(item === link ? [id] : [])]])) as Record<ModelLink, string[]>}));
    }
    // Zero robota zawsze pod podstawą: najniższy punkt siatek podstawy wzdłuż osi J1 (bez siatki podstawy — na osi J1).
    const lowest = useMemo(() => {
        const sources = preview ? preview.items.filter(item => item.link === "base").map(item => ({geometry: item.geometry, matrix: preview.matrix})) : [];
        return sources.length ? (point: import("three").Vector3, axis: import("three").Vector3) => lowestAlong(sources, point, axis) : null;
    }, [preview]);
    const root = useMemo(() => preview?.items.length ? rootUnderBase(values, assembly.transform, lowest) : null, [values, assembly.transform, lowest, preview]);
    function moveRoot() {
        if (!root?.needed) return;
        setAssembly(current => ({...current, transform: {...current.transform, ...root.transform}}));
        onDraft(root.draft);
        onNotice({source: "robot-model", severity: "info", message: `Zero robota przeniesione pod podstawę (przesunięcie ${root.root.toArray().map(v => fixed(v * 1000, 1)).join(" / ")} mm). Zapisz model 3D i konfigurację („Zapisz do ROS”).`});
    }
    function confirm() {
        if (!pending) return;
        if (pending.target.kind === "joint" && edit) {
            let draft = edit.draft, moved = false;
            if (pending.target.joint === 1) {
                // Nowa oś J1 → zero robota od razu na tej osi pod podstawą.
                const merged = {...values, ...Object.fromEntries(Object.entries(edit.draft).map(([key, value]) => [key, Number(value)]))};
                const shift = rootUnderBase(merged, assembly.transform, lowest);
                if (shift.needed) { draft = {...draft, ...shift.draft}; moved = true; setAssembly(current => ({...current, transform: {...current.transform, ...shift.transform}})); }
            }
            onDraft(draft);
            onNotice({source: "robot-model-pick", severity: "info", message: `Układ J${pending.target.joint} ustawiony w szkicu${edit.snapped ? " (oś przyciągnięta do osi układu)" : ""}; zmieniono pola J${pending.target.joint}${pending.target.joint < 6 ? ` i J${pending.target.joint + 1}` : " i TCP"}.${moved ? " Zero robota przeniesione pod podstawę — zapisz też model 3D." : ""} Zastosowanie w sterowaniu: „Zapisz do ROS”.`});
        } else onDraft(applyTcpPoint(values, pending.point));
        setPending(null); setTarget(null);
    }
    const checks = useMemo(() => geometryChecks(values), [values]);
    const meshes = model?.meshes ?? [];
    return <section className="robot-model-panel" aria-label="Model 3D i układy przegubów">
        <header><h2>Model 3D i układy przegubów</h2>
            <span className={`badge ${error ? "error" : dirty ? "warn" : "ok"}`}>{error ? "Niedostępny" : dirty ? "Niezapisane zmiany modelu" : `${meshes.length} siatek`}</span></header>
        <p className="robot-model-help">Eksport z Fusion 360: każdy człon jako osobny komponent (STL lub OBJ), robot w pozycji zerowej, wszystkie pliki z tego samego złożenia — wtedy mają wspólny układ współrzędnych. Po wgraniu powstaje siatka bezpieczeństwa: odsunięta o 2 mm od bryły i rzadsza od powierzchni.</p>
        {error && <p className="robot-model-error" role="status">{error}</p>}
        <div className="robot-model-row">
            <label>Jednostki pliku<select value={units} onChange={event => setUnits(event.target.value)} disabled={busy}>
                <option value="mm">mm (Fusion domyślnie)</option><option value="cm">cm</option><option value="m">m</option><option value="in">cale</option></select></label>
            <input ref={fileInput} hidden type="file" multiple accept=".stl,.obj" onChange={event => { void upload(event.target.files); event.target.value = ""; }}/>
            <button type="button" onClick={() => fileInput.current?.click()} disabled={busy || !!error}>Wgraj siatki STL/OBJ…</button>
            <label className="robot-model-toggle"><input type="checkbox" checked={display.safety} onChange={event => setDisplay(current => ({...current, safety: event.target.checked}))}/>Siatki bezpieczeństwa</label>
            <label className="robot-model-toggle"><input type="checkbox" checked={display.primitives} onChange={event => setDisplay(current => ({...current, primitives: event.target.checked}))}/>Bryły zastępcze</label>
        </div>
        {meshes.length > 0 && <div className="robot-model-table-scroll"><table className="robot-model-table">
            <thead><tr><th>Siatka</th><th>Trójkąty</th><th>Siatka bezpieczeństwa</th><th>Człon</th><th/></tr></thead>
            <tbody>{meshes.map(mesh => <tr key={mesh.id} data-state={mesh.state}>
                <td><strong title={mesh.name}>{mesh.name}</strong><small>{stateLabels[mesh.state]}{mesh.state === "failed" && mesh.message ? ` — ${mesh.message}` : ""}</small></td>
                <td>{fixed(mesh.triangles, 0)}</td>
                <td title={mesh.safety.message}>{mesh.safety.state === "ready" ? <>{fixed(mesh.safety.triangles, 0)} trójk. · {fixed(mesh.safety.offset_mm)} mm<small>odległość {fixed(mesh.safety.min_mm)}–{fixed(mesh.safety.max_mm)} mm (śr. {fixed(mesh.safety.mean_mm)}) · komórka {fixed(mesh.safety.cell_mm)} mm{mesh.safety.coarser_than_surface === false ? " · gęstsza od powierzchni" : ""}{mesh.safety.simplified_mm ? ` · gęsty eksport uproszczony (${fixed(mesh.safety.simplified_mm)} mm), odsunięcie zachowawcze` : ""}</small></> :
                    <small>{stateLabels[mesh.safety.state]}{mesh.safety.state === "failed" && mesh.safety.message ? ` — ${mesh.safety.message}` : ""}</small>}</td>
                <td><select aria-label={`Człon dla ${mesh.name}`} value={assignmentOf(assembly, mesh.id)} disabled={mesh.state !== "ready"} onChange={event => assign(mesh.id, event.target.value as ModelLink | "")}>
                    <option value="">— nieprzypisana —</option>{modelLinks.map(link => <option key={link} value={link}>{modelLinkLabels[link]}</option>)}</select></td>
                <td><button type="button" className="robot-model-delete" aria-label={`Usuń ${mesh.name}`} title="Usuń siatkę (tylko nieprzypisaną w zapisanym modelu)" onClick={() => void remove(mesh)} disabled={mesh.state === "queued" || mesh.state === "processing"}>×</button></td>
            </tr>)}</tbody></table></div>}
        <fieldset className="robot-model-transform"><legend>Układ złożenia → baza robota</legend>
            <label>Oś „w górę” w CAD<select value={assembly.up} onChange={event => setAssembly(current => ({...current, up: event.target.value as "z" | "y"}))}>
                <option value="z">Z (jak robot)</option><option value="y">Y (Fusion „Y up”)</option></select></label>
            {transformFields.map(([key, label, unit]) => <label key={key}>{label}<span><input type="number" step="any" value={assembly.transform[key]}
                onChange={event => { const value = Number(event.target.value); if (Number.isFinite(value)) setAssembly(current => ({...current, transform: {...current.transform, [key]: value}})); }}/><small>{unit}</small></span></label>)}
            <p>Przesunięcie i obrót początku złożenia względem układu bazy (po zamianie osi „w górę”). Zmiana po wskazaniu osi wymaga ponownego wskazania.</p>
            {root && <p className={`robot-model-root ${root.needed ? "warn" : "ok"}`} role="status">{root.needed
                ? <>Zero robota nie leży pod podstawą — przesunięcie do osi J1 pod spodem podstawy: {root.root.toArray().map(v => fixed(v * 1000, 1)).join(" / ")} mm.
                    <button type="button" className="configuration-apply" onClick={moveRoot} disabled={busy}>Przenieś zero pod podstawę</button></>
                : <>Zero robota: na osi J1 pod podstawą{lowest ? "" : " (brak siatki podstawy — wysokość bez zmian)"}.</>}</p>}
        </fieldset>
        <div className="robot-model-row"><button type="button" className="configuration-apply" onClick={() => void save()} disabled={!dirty || busy}>Zapisz model 3D</button>
            {dirty && <button type="button" onClick={() => model && setAssembly(model.assembly)} disabled={busy}>Cofnij zmiany modelu</button>}
            <small>Model 3D jest zapisywany osobno od konfiguracji — nie zmienia kinematyki ani przygotowanych programów.</small></div>

        <fieldset className="robot-model-frames"><legend>Układy przegubów i TCP ze wskazania na modelu</legend>
            <p>Wskaż przycisk, potem kliknij na podglądzie powierzchnię walcową przegubu. Oś zostanie dopasowana do walca, a układ przegubu umieszczony na osi w środku wskazanego fragmentu. Po zatwierdzeniu wartości trafiają do szkicu (pola przegubu i następnego); w sterowaniu po „Zapisz do ROS”.</p>
            <div className="robot-model-pick-buttons" role="group" aria-label="Wskazanie układu">
                {[1, 2, 3, 4, 5, 6].map(joint => <button key={joint} type="button" aria-pressed={target?.kind === "joint" && target.joint === joint} disabled={!preview?.items.length}
                    onClick={() => { setPending(null); setTarget(current => current?.kind === "joint" && current.joint === joint ? null : {kind: "joint", joint}); }}>Oś J{joint}</button>)}
                <button type="button" aria-pressed={target?.kind === "tcp-point"} disabled={!preview?.items.length} onClick={() => { setPending(null); setTarget(current => current?.kind === "tcp-point" ? null : {kind: "tcp-point"}); }}>TCP: punkt</button>
                <button type="button" aria-pressed={target?.kind === "tcp-center"} disabled={!preview?.items.length} onClick={() => { setPending(null); setTarget(current => current?.kind === "tcp-center" ? null : {kind: "tcp-center"}); }}>TCP: środek walca</button>
            </div>
            {target && !pending && <p className="robot-model-pick-status" role="status">{pickLabel}. <button type="button" onClick={() => setTarget(null)}>Anuluj</button></p>}
            {pending && <div className="robot-model-pick-result" role="status">
                {pending.fit ? <span>Dopasowano walec „{pending.mesh}”: promień {fixed(pending.fit.radius * 1000)} mm, odchyłka {fixed(pending.fit.residual * 1000, 3)} mm, {pending.fit.faces} ścian, zasięg {fixed(pending.fit.coverageDeg, 0)}°
                    {edit && <> · zmiana kierunku osi {fixed(edit.tiltDeg)}°{edit.snapped ? " · przyciągnięta do osi układu" : ""}</>}
                    {pending.fit.coverageDeg < 90 && <strong> · mały wycinek walca — dokładność kierunku ograniczona</strong>}</span> :
                    <span>Punkt TCP na „{pending.mesh}”: {pending.point.toArray().map(v => fixed(v * 1000)).join(" / ")} mm (układ bazy)</span>}
                <span className="robot-model-pick-actions">
                    {pending.target.kind === "joint" && <button type="button" onClick={() => setPending(current => current && {...current, reverse: !current.reverse})}>Odwróć zwrot osi</button>}
                    <button type="button" className="configuration-apply" onClick={confirm}>{pending.target.kind === "joint" ? `Ustaw układ J${pending.target.joint} w szkicu` : "Ustaw TCP w szkicu"}</button>
                    <button type="button" onClick={() => setPending(null)}>Wskaż ponownie</button>
                    <button type="button" onClick={() => { setPending(null); setTarget(null); }}>Anuluj</button>
                </span>
            </div>}
            <p className="robot-model-checks">Kontrola szkicu: J2 ∥ J3 — odchyłka {fixed(checks.j2j3ParallelDeg, 3)}° · nadgarstek: {checks.wrist.map(item => `${item.axes} ${fixed(item.distanceMm)} mm`).join(", ")} (0 = osie się przecinają)</p>
        </fieldset>
    </section>;
}
