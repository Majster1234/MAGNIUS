import { useCallback, useEffect, useRef, useState } from "react";
import * as THREE from "three";
import type { Notice } from "./eventLog";
import { moduleNotice, useCondition } from "./messageCenter";
import { deleteMesh, elementOf, elementRoleLabels, fetchRobotModel, saveElement, uploadMesh, type ElementRole, type MeshInfo, type RobotModel } from "./robotModel";
import { loadChainMeshes, type ModelChain } from "./modelChain";
import type { ModelPick, PickOverlay } from "./configurationScene";
import type { PreviewModel } from "./ConfigurationPreview";
import type { Values } from "./configurationData";
import { ElementEditor } from "./ElementEditor";
import { ModelChainBuilder } from "./ModelChainBuilder";

const stateLabels: Record<MeshInfo["state"], string> = {queued: "w kolejce", processing: "przetwarzanie…", ready: "wczytana", failed: "błąd"};
const fixed = (value: number | undefined, digits = 2) => value === undefined ? "—" : value.toLocaleString("pl-PL", {maximumFractionDigits: digits});

// Model 3D robota w trybie modułowym: łańcuch elementów od podstawy (ModelChainBuilder) — z niego układy przegubów
// trafiają do szkicu konfiguracji (zastosowanie: „Zapisz do ROS”) — oraz biblioteka wgranych plików. Podgląd konfiguracji
// pokazuje siatki łańcucha (ze szkicu) w układzie bazy robota.
export function RobotModelPanel({values, onDraft, onNotice, onPreview}: {
    values: Values; onDraft: (updates: Record<string, string>) => void; onNotice: (notice: Notice) => void;
    onPreview: (preview: {model: PreviewModel | null; display: {safety: boolean; primitives: boolean}; overlay: PickOverlay; onPick: ((pick: ModelPick) => void) | null; hint: string}) => void;
    onPicking?: (picking: boolean) => void;
}) {
    const [model, setModel] = useState<RobotModel | null>(null);
    const [error, setError] = useState("");
    const [units, setUnits] = useState("mm");
    const [busy, setBusy] = useState(false);
    const [display, setDisplay] = useState({safety: false, primitives: false});
    const [preview, setPreviewModel] = useState<PreviewModel | null>(null);
    const [chainDraft, setChainDraft] = useState<Pick<ModelChain, "up" | "elements"> | null>(null);
    const [editing, setEditing] = useState<string | null>(null);
    const fileInput = useRef<HTMLInputElement>(null);
    const refresh = useCallback(async () => {
        try { setModel(await fetchRobotModel()); setError(""); }
        catch (failure) { setError(failure instanceof Error ? failure.message : "Model 3D niedostępny"); }
    }, []);
    useEffect(() => { void refresh(); }, [refresh]);
    useCondition("config-model", error ? moduleNotice({ source: "config-model", severity: "warning", title: "Model 3D niedostępny", message: error }, "Konfiguracja · model 3D", "#config") : null);
    const processing = !!model?.meshes.some(mesh => mesh.state === "queued" || mesh.state === "processing" || mesh.safety.state === "queued" || mesh.safety.state === "processing");
    useEffect(() => {
        if (!processing) return;
        const timer = setInterval(() => void refresh(), 1500);
        return () => clearInterval(timer);
    }, [processing, refresh]);
    // Podgląd konfiguracji: siatki łańcucha ze szkicu (także niezapisanego) w układzie bazy robota.
    const loadKey = model ? JSON.stringify([chainDraft ?? model.chain, display.safety, model.meshes.map(mesh => [mesh.id, mesh.state, mesh.safety.state, mesh.element?.revision])]) : "";
    useEffect(() => {
        if (!model) return;
        let closed = false;
        const chain = chainDraft ? {revision: "draft", ...chainDraft} : model.chain;
        void loadChainMeshes({chain, meshes: model.meshes}, display.safety).then(items => {
            if (!closed) setPreviewModel({items, matrix: new THREE.Matrix4()});
        }).catch(failure => onNotice({source: "robot-model", severity: "warning", message: `Nie wczytano siatek modelu 3D: ${failure instanceof Error ? failure.message : "błąd"}`}));
        return () => { closed = true; };
    }, [loadKey]);  // eslint-disable-line react-hooks/exhaustive-deps
    useEffect(() => {
        onPreview({model: preview, display: {safety: display.safety, primitives: display.primitives || !preview?.items.length}, overlay: {}, onPick: null, hint: ""});
    }, [preview, display, onPreview]);

    async function upload(files: FileList | null) {
        if (!files?.length) return;
        setBusy(true);
        for (const file of Array.from(files)) {
            try { await uploadMesh(file, units); onNotice({source: "robot-model", severity: "info", message: `Wgrano „${file.name}” — trwa odczyt i generowanie siatki bezpieczeństwa.`}); }
            catch (failure) { onNotice({source: "robot-model", severity: "error", message: `„${file.name}”: ${failure instanceof Error ? failure.message : "błąd wgrywania"}`}); }
        }
        setBusy(false); await refresh();
    }
    async function remove(mesh: MeshInfo) {
        try { await deleteMesh(mesh.id); if (editing === mesh.id) setEditing(null); await refresh(); }
        catch (failure) { onNotice({source: "robot-model", severity: "warning", message: failure instanceof Error ? failure.message : "Nie usunięto siatki"}); }
    }
    // Status elementu z tabeli zapisywany od razu; zmiana z modelu osi na inny status usuwa wskazaną oś.
    async function setRole(mesh: MeshInfo, role: ElementRole | null) {
        const element = elementOf(mesh);
        try {
            await saveElement(mesh.id, {role, axis: role === "axis" ? element.axis : null}, element.revision);
            if (element.axis && role !== "axis") onNotice({source: "robot-element", severity: "info", message: `„${mesh.name}”: oś usunięta razem ze statusem modelu osi.`});
            if (role === "axis" && !element.axis) setEditing(mesh.id);
            await refresh();
        } catch (failure) { onNotice({source: "robot-element", severity: "error", message: failure instanceof Error ? failure.message : "Nie zapisano statusu elementu"}); await refresh(); }
    }
    const meshes = model?.meshes ?? [];
    const used = new Set((model?.chain?.elements ?? []).map(element => element.mesh));
    const edited = meshes.find(mesh => mesh.id === editing) ?? null;
    return <section className="robot-model-panel" aria-label="Model 3D robota">
        <header><h2>Model 3D robota</h2>
            <span className={`badge ${error ? "error" : "ok"}`}>{error ? "Niedostępny" : `${model?.chain?.elements.length ?? 0} elementów · ${meshes.length} plików`}</span></header>
        <p className="robot-model-help">Robot budowany z elementów od podstawy: każdy element wgrywany osobno (STL lub OBJ, z Fusion 360 „Zapisz jako siatkę”), we własnym układzie pliku. Przy każdym kolejnym elemencie wybierasz: <strong>element konstrukcyjny</strong> (liczy się do kinematyki, gdy za nim jest oś) albo <strong>miejsce łożyskowania osi</strong> (część obracająca się wokół tej osi — od niej zaczyna się człon), i spasowujesz go z poprzednikiem wskazaniami walców i czół. Z łańcucha powstają układy przegubów: „Przenieś do szkicu konfiguracji”, potem „Zapisz do ROS”.</p>
        {error && <p className="robot-model-error" role="status">{error}</p>}
        {model && <ModelChainBuilder model={model} values={values} onDraft={onDraft} onChainDraft={setChainDraft} onRefresh={() => refresh()} onNotice={onNotice}/>}
        <details className="robot-model-legacy"><summary>Biblioteka plików <small>({meshes.length}; status, oś pliku, siatki bezpieczeństwa, usuwanie)</small></summary>
        <div className="robot-model-row">
            <label>Jednostki pliku<select value={units} onChange={event => setUnits(event.target.value)} disabled={busy}>
                <option value="mm">mm (Fusion domyślnie)</option><option value="cm">cm</option><option value="m">m</option><option value="in">cale</option></select></label>
            <input ref={fileInput} hidden type="file" multiple accept=".stl,.obj" onChange={event => { void upload(event.target.files); event.target.value = ""; }}/>
            <button type="button" onClick={() => fileInput.current?.click()} disabled={busy || !!error}>Wgraj siatki STL/OBJ…</button>
            <label className="robot-model-toggle"><input type="checkbox" checked={display.safety} onChange={event => setDisplay(current => ({...current, safety: event.target.checked}))}/>Siatki bezpieczeństwa (podgląd konfiguracji)</label>
            <label className="robot-model-toggle"><input type="checkbox" checked={display.primitives} onChange={event => setDisplay(current => ({...current, primitives: event.target.checked}))}/>Bryły zastępcze</label>
        </div>
        {meshes.length > 0 && <div className="robot-model-table-scroll"><table className="robot-model-table">
            <thead><tr><th>Plik</th><th>Trójkąty</th><th>Siatka bezpieczeństwa</th><th>Status elementu</th><th/></tr></thead>
            <tbody>{meshes.map(mesh => <tr key={mesh.id} data-state={mesh.state} aria-selected={editing === mesh.id}>
                <td><strong title={mesh.name}>{mesh.name}</strong><small>{stateLabels[mesh.state]}{mesh.state === "failed" && mesh.message ? ` — ${mesh.message}` : ""}{used.has(mesh.id) ? " · w łańcuchu" : ""}</small></td>
                <td>{fixed(mesh.triangles, 0)}</td>
                <td title={mesh.safety.message}>{mesh.safety.state === "ready" ? <>{fixed(mesh.safety.triangles, 0)} trójk. · {fixed(mesh.safety.offset_mm)} mm<small>odległość {fixed(mesh.safety.min_mm)}–{fixed(mesh.safety.max_mm)} mm (śr. {fixed(mesh.safety.mean_mm)}) · komórka {fixed(mesh.safety.cell_mm)} mm{mesh.safety.coarser_than_surface === false ? " · gęstsza od powierzchni" : ""}{mesh.safety.simplified_mm ? ` · gęsty eksport uproszczony (${fixed(mesh.safety.simplified_mm)} mm), odsunięcie zachowawcze` : ""}</small></> :
                    <small>{stateLabels[mesh.safety.state]}{mesh.safety.state === "failed" && mesh.safety.message ? ` — ${mesh.safety.message}` : ""}</small>}</td>
                <td><ElementCell mesh={mesh} onRole={role => void setRole(mesh, role)} onEdit={() => setEditing(current => current === mesh.id ? null : mesh.id)} open={editing === mesh.id}/></td>
                <td><button type="button" className="robot-model-delete" aria-label={`Usuń ${mesh.name}`} title={used.has(mesh.id) ? "Plik jest użyty w łańcuchu — najpierw usuń go z łańcucha i zapisz model" : "Usuń plik"} onClick={() => void remove(mesh)} disabled={mesh.state === "queued" || mesh.state === "processing" || used.has(mesh.id)}>×</button></td>
            </tr>)}</tbody></table></div>}
        {edited && <ElementEditor key={edited.id} mesh={edited} onSaved={() => refresh()} onClose={() => setEditing(null)} onNotice={onNotice}/>}
        </details>
    </section>;
}

function ElementCell({mesh, open, onRole, onEdit}: {mesh: MeshInfo; open: boolean; onRole: (role: ElementRole | null) => void; onEdit: () => void}) {
    const element = elementOf(mesh);
    return <div className="element-cell">
        <select aria-label={`Status elementu ${mesh.name}`} value={element.role ?? ""} onChange={event => onRole((event.target.value || null) as ElementRole | null)}>
            <option value="">— nieokreślony —</option>{(Object.keys(elementRoleLabels) as ElementRole[]).map(role => <option key={role} value={role}>{elementRoleLabels[role]}</option>)}</select>
        {element.role === "axis" && <small className={element.axis ? "" : "element-warning"}>{element.axis ? `oś: R ${fixed(element.axis.radius_mm)} mm · ${fixed(element.axis.coverage_deg, 0)}°` : "oś nie wskazana"}</small>}
        <button type="button" aria-pressed={open} disabled={mesh.state !== "ready"} onClick={onEdit}>{element.role === "axis" ? "Oś…" : "Podgląd"}</button>
    </div>;
}
