import { useEffect, useRef, useState } from "react";
import { emptyDocument, fieldErrors, valueOf, configurationKeys, type Draft, type Values } from "./configurationData";
import type { createConfigurationScene, ModelPick, PickOverlay } from "./configurationScene";
import type { LoadedLinkMesh } from "./robotModel";
import type { Matrix4 } from "three";

export type PreviewModel = {items: LoadedLinkMesh[]; matrix: Matrix4};
export function ConfigurationPreview({draft, view, selected, nativeErrors = {}, model = null, display = {safety: false, primitives: true}, onPick = null, overlay = {}, pickHint = ""}: {
    draft: Draft; view: "geometry" | "limits"; selected: number; nativeErrors?: Record<string,string>;
    model?: PreviewModel | null; display?: {safety: boolean; primitives: boolean}; onPick?: ((pick: ModelPick) => void) | null; overlay?: PickOverlay; pickHint?: string;
}) {
    const host = useRef<HTMLDivElement>(null), scene = useRef<ReturnType<typeof createConfigurationScene> | null>(null);
    const previous = useRef<Values>(emptyDocument().values);
    const [error, setError] = useState("");
    const errors = {...fieldErrors(draft), ...nativeErrors};
    const values = Object.fromEntries(configurationKeys.map(key => [key, errors[key] ? previous.current[key] : valueOf(key, draft[key] ?? "")]));
    const input = useRef({values, view, selected}); input.current = {values, view, selected};
    const extras = useRef({model, display, onPick, overlay}); extras.current = {model, display, onPick, overlay};
    useEffect(() => { previous.current = values; scene.current?.update(input.current); });
    useEffect(() => { scene.current?.setModel(model?.items ?? [], model?.matrix); }, [model]);
    useEffect(() => { scene.current?.setDisplay(display); }, [display.safety, display.primitives]);
    useEffect(() => { scene.current?.setPick(onPick); }, [onPick]);
    useEffect(() => { scene.current?.setOverlay(overlay); }, [overlay]);
    useEffect(() => {
        let closed = false;
        void import('./configurationScene').then(engine => {
            if (closed || !host.current) return;
            try {
                scene.current = engine.createConfigurationScene(host.current); scene.current.update(input.current);
                const {model: m, display: d, onPick: p, overlay: o} = extras.current;
                if (m) scene.current.setModel(m.items, m.matrix);
                scene.current.setDisplay(d); scene.current.setPick(p); scene.current.setOverlay(o);
            }
            catch { host.current.replaceChildren(); setError("Podgląd konfiguracji niedostępny — sprawdź obsługę WebGL"); }
        }).catch(() => { if (!closed) setError("Nie udało się wczytać podglądu konfiguracji"); });
        return () => { closed = true; scene.current?.dispose(); scene.current = null; };
    }, []);
    const incomplete = configurationKeys.filter(key => key.includes(".origin_")).some(key => values[key] === null);
    return <section className={`configuration-preview${model?.items.length ? " has-model" : ""}`} aria-label="Model konfiguracji robota">
        <header><h2>Model konfiguracji</h2><span className={`badge ${incomplete ? "warn" : "ok"}`}>{incomplete ? "Geometria częściowa · uzupełnienie demonstracyjne" : "Geometria szkicu"}</span><small>Poza zerowa · podgląd lokalny</small></header>
        <div className="preview-viewport"><div className="preview-canvas" ref={host}/>
            {error && <p className="preview-overlay" role="status">{error}</p>}
            <button className="preview-home" aria-label="Dopasuj model konfiguracji" title="Dopasuj model konfiguracji" disabled={!!error} onClick={() => scene.current?.fit()}><svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.6" aria-hidden="true"><path d="M3 11 12 3l9 8M5 10v11h5v-7h4v7h5V10"/></svg></button>
            <span className="configuration-legend">{view === "geometry" ? "Układy lokalne: X czerwony · Y zielony · Z niebieski" : "Łuk: zakres przegubu · dymek: przełożenie i granice w °"}</span>
            {pickHint && <span className="configuration-pick-hint" role="status">{pickHint}</span>}
        </div>
    </section>;
}
