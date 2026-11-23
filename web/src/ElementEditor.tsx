import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import type { Notice } from "./eventLog";
import { fitCylinder } from "./axisFit";
import { axisFromFit, reverseAxis, elementOf, elementRoleLabels, meshGeometry, saveElement, type ElementAxis, type ElementRole, type MeshInfo } from "./robotModel";
import type { createElementScene, ElementPick } from "./elementScene";

const fixed = (value: number | undefined, digits = 2) => value === undefined ? "—" : value.toLocaleString("pl-PL", {maximumFractionDigits: digits});
const vector = (values: number[], digits = 2) => values.map(value => fixed(Math.abs(value) < 5e-13 ? 0 : value, digits)).join(" / ");
const sameAxis = (a: ElementAxis | null, b: ElementAxis | null) => JSON.stringify(a) === JSON.stringify(b);

// Definicja elementu (tryb modułowy, etap 1): status i oś kinematyczna modelu osi w układzie pliku elementu.
// Oś = walec wskazany kliknięciem (czop, otwór, gniazdo łożyska); zwrot = dodatni obrót przegubu wg reguły prawej dłoni.
// Położenie elementu w robocie i kinematyka z drzewa elementów — kolejne etapy.
export function ElementEditor({mesh, onSaved, onClose, onNotice}: {
    mesh: MeshInfo; onSaved: () => Promise<void> | void; onClose: () => void; onNotice: (notice: Notice) => void;
}) {
    const saved = elementOf(mesh);
    const [role, setRole] = useState<ElementRole | null>(saved.role);
    const [axis, setAxis] = useState<ElementAxis | null>(saved.axis);
    const [picking, setPicking] = useState(false);
    const [showSafety, setShowSafety] = useState(false);
    const [busy, setBusy] = useState(false);
    const [error, setError] = useState("");
    const host = useRef<HTMLDivElement>(null), scene = useRef<ReturnType<typeof createElementScene> | null>(null);
    const geometry = useRef<THREE.BufferGeometry | null>(null);
    const dirty = role !== saved.role || !sameAxis(role === "axis" ? axis : null, saved.axis);
    // Nowa rewizja z serwera (zapis tutaj lub w innym panelu) → szkic od stanu zapisanego.
    useEffect(() => { setRole(saved.role); setAxis(saved.axis); }, [saved.revision]);  // eslint-disable-line react-hooks/exhaustive-deps
    useEffect(() => {
        let closed = false;
        void import("./elementScene").then(async engine => {
            if (closed || !host.current) return;
            try { scene.current = engine.createElementScene(host.current); }
            catch { host.current.replaceChildren(); setError("Podgląd elementu niedostępny — sprawdź obsługę WebGL"); return; }
            const surface = await meshGeometry(mesh.id);
            if (closed) return;
            geometry.current = surface; scene.current.setElement(surface);
        }).catch(failure => { if (!closed) setError(failure instanceof Error ? failure.message : "Nie wczytano elementu"); });
        return () => { closed = true; scene.current?.dispose(); scene.current = null; geometry.current = null; };
    }, [mesh.id]);
    useEffect(() => {
        if (!showSafety || mesh.safety.state !== "ready") { scene.current?.setSafety(false); return; }
        let closed = false;
        void meshGeometry(mesh.id, "safety").then(safety => {
            if (closed || !scene.current || !geometry.current) return;
            scene.current.setElement(geometry.current, safety); scene.current.setSafety(true);
        }).catch(() => onNotice({source: "robot-element", severity: "warning", message: "Nie wczytano siatki bezpieczeństwa elementu"}));
        return () => { closed = true; };
    }, [showSafety, mesh.id, mesh.safety.state, onNotice]);
    useEffect(() => {
        scene.current?.setPick(picking ? (pick: ElementPick) => {
            try {
                const fit = fitCylinder(pick.geometry, pick.face);
                setAxis(axisFromFit(fit)); setPicking(false);
            } catch (failure) {
                onNotice({source: "robot-element", severity: "warning", message: failure instanceof Error ? failure.message : "Nie dopasowano osi"});
            }
        } : null);
    }, [picking, onNotice]);
    useEffect(() => {
        scene.current?.setOverlay(role === "axis" && axis ? {point: new THREE.Vector3(...axis.point_mm).multiplyScalar(.001),
            direction: new THREE.Vector3(...axis.direction), radius: axis.radius_mm / 1000} : null);
    }, [role, axis]);

    async function save() {
        setBusy(true);
        try {
            await saveElement(mesh.id, {role, axis: role === "axis" ? axis : null}, saved.revision);
            onNotice({source: "robot-element", severity: "info", message: `Element „${mesh.name}”: ${role ? elementRoleLabels[role].toLowerCase() : "status nieokreślony"}${role === "axis" ? axis ? ", oś zapisana" : " — oś jeszcze nie wskazana" : ""}.`});
            await onSaved();
        } catch (failure) { onNotice({source: "robot-element", severity: "error", message: failure instanceof Error ? failure.message : "Nie zapisano elementu"}); }
        setBusy(false);
    }
    const ready = mesh.state === "ready";
    return <section className="element-editor" aria-label={`Element ${mesh.name}`}>
        <header><h3>Element: <span title={mesh.name}>{mesh.name}</span></h3>
            <span className={`badge ${dirty ? "warn" : "ok"}`}>{dirty ? "Niezapisane zmiany" : "Zapisany"}</span>
            <button type="button" className="element-editor-close" onClick={onClose} aria-label="Zamknij edytor elementu" title="Zamknij">×</button></header>
        <div className="element-editor-body">
            <div className="element-editor-viewport preview-viewport">
                <div className="preview-canvas" ref={host}/>
                {(error || !ready) && <p className="preview-overlay" role="status">{error || "Siatka nie jest jeszcze wczytana"}</p>}
                <button className="preview-home" aria-label="Dopasuj widok elementu" title="Dopasuj widok" disabled={!!error} onClick={() => scene.current?.fit()}><svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.6" aria-hidden="true"><path d="M3 11 12 3l9 8M5 10v11h5v-7h4v7h5V10"/></svg></button>
                <span className="configuration-legend">Układ pliku elementu: X czerwony · Y zielony · Z niebieski</span>
                {picking && <span className="configuration-pick-hint" role="status">Kliknij powierzchnię walcową osi (czop, otwór, gniazdo łożyska)</span>}
            </div>
            <div className="element-editor-form">
                <fieldset className="element-role" disabled={busy}><legend>Status elementu</legend>
                    <label><input type="radio" name={`role-${mesh.id}`} checked={role === "axis"} onChange={() => setRole("axis")}/>
                        <span><strong>Miejsce łożyskowania osi</strong><small>Część obracająca się wokół osi przegubu (z gniazdem łożyska lub czopem) — od niej zaczyna się człon tej osi.</small></span></label>
                    <label><input type="radio" name={`role-${mesh.id}`} checked={role === "structural"} onChange={() => { setRole("structural"); setPicking(false); }}/>
                        <span><strong>Element konstrukcyjny</strong><small>Sztywny, bez osi (rura, wspornik, przedłużenie, adapter). Liczy się do kinematyki, gdy za nim w łańcuchu jest kolejny model osi.</small></span></label>
                    <label><input type="radio" name={`role-${mesh.id}`} checked={role === null} onChange={() => { setRole(null); setPicking(false); }}/>
                        <span><strong>Nieokreślony</strong><small>Element nie wejdzie do złożenia modułowego.</small></span></label>
                </fieldset>
                {role === "axis" && <fieldset className="element-axis" disabled={busy || !ready}><legend>Oś kinematyczna</legend>
                    {axis ? <dl>
                        <dt>Punkt</dt><dd>{vector(axis.point_mm)} mm</dd>
                        <dt>Kierunek</dt><dd>{vector(axis.direction, 4)}</dd>
                        <dt>Walec</dt><dd>R {fixed(axis.radius_mm)} mm · odchyłka {fixed(axis.residual_mm, 3)} mm · {axis.faces} ścian · zasięg {fixed(axis.coverage_deg, 0)}°</dd>
                    </dl> : <p>Oś nie jest wskazana.</p>}
                    {axis && <label className="chain-check"><input type="checkbox" checked={!!axis.reversed} onChange={() => setAxis(current => current && reverseAxis(current))}/>Odwrócony zwrot osi (strzałka = dodatni obrót)</label>}
                    {axis && axis.coverage_deg < 90 && <p className="element-warning">Mały wycinek walca (&lt; 90°) — dokładność kierunku ograniczona; wskaż pełniejszą powierzchnię.</p>}
                    <p>Strzałka na podglądzie pokazuje zwrot osi: dodatni obrót przegubu według reguły prawej dłoni. Punkt leży na osi w środku wskazanego fragmentu walca.</p>
                    <div className="robot-model-pick-actions">
                        <button type="button" aria-pressed={picking} onClick={() => setPicking(current => !current)}>{picking ? "Anuluj wskazanie" : axis ? "Wskaż ponownie" : "Wskaż oś"}</button>
                        
                        {axis && <button type="button" onClick={() => setAxis(null)}>Usuń oś</button>}
                    </div>
                </fieldset>}
                <label className="robot-model-toggle element-safety"><input type="checkbox" checked={showSafety} disabled={mesh.safety.state !== "ready"} onChange={event => setShowSafety(event.target.checked)}/>Siatka bezpieczeństwa</label>
                <div className="robot-model-pick-actions">
                    <button type="button" className="configuration-apply" onClick={() => void save()} disabled={!dirty || busy}>Zapisz element</button>
                    {dirty && <button type="button" onClick={() => { setRole(saved.role); setAxis(saved.axis); setPicking(false); }} disabled={busy}>Cofnij zmiany</button>}
                </div>
            </div>
        </div>
    </section>;
}
