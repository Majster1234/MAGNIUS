import { useEffect, useRef, useState } from "react";
import { emptyDocument, fieldErrors, valueOf, configurationKeys, type Draft, type Values } from "./configurationData";
import type { createConfigurationScene } from "./configurationScene";

export function ConfigurationPreview({draft, view, selected, nativeErrors = {}}: {draft: Draft; view: "geometry" | "limits"; selected: number; nativeErrors?: Record<string,string>}) {
    const host = useRef<HTMLDivElement>(null), scene = useRef<ReturnType<typeof createConfigurationScene> | null>(null);
    const previous = useRef<Values>(emptyDocument().values);
    const [error, setError] = useState("");
    const errors = {...fieldErrors(draft), ...nativeErrors};
    const values = Object.fromEntries(configurationKeys.map(key => [key, errors[key] ? previous.current[key] : valueOf(key, draft[key] ?? "")]));
    const input = useRef({values, view, selected}); input.current = {values, view, selected};
    useEffect(() => { previous.current = values; scene.current?.update(input.current); });
    useEffect(() => {
        let closed = false;
        void import('./configurationScene').then(engine => {
            if (closed || !host.current) return;
            try { scene.current = engine.createConfigurationScene(host.current); scene.current.update(input.current); }
            catch { host.current.replaceChildren(); setError("Podgląd konfiguracji niedostępny — sprawdź obsługę WebGL"); }
        }).catch(() => { if (!closed) setError("Nie udało się wczytać podglądu konfiguracji"); });
        return () => { closed = true; scene.current?.dispose(); scene.current = null; };
    }, []);
    const incomplete = configurationKeys.filter(key => key.includes(".origin_")).some(key => values[key] === null);
    return <section className="configuration-preview" aria-label="Model konfiguracji robota">
        <header><h2>Model konfiguracji</h2><span className={`badge ${incomplete ? "warn" : "ok"}`}>{incomplete ? "Geometria częściowa · uzupełnienie demonstracyjne" : "Geometria szkicu"}</span><small>Poza zerowa · podgląd lokalny</small></header>
        <div className="preview-viewport"><div className="preview-canvas" ref={host}/>
            {error && <p className="preview-overlay" role="status">{error}</p>}
            <button className="preview-home" aria-label="Dopasuj model konfiguracji" title="Dopasuj model konfiguracji" disabled={!!error} onClick={() => scene.current?.fit()}><svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.6" aria-hidden="true"><path d="M3 11 12 3l9 8M5 10v11h5v-7h4v7h5V10"/></svg></button>
            <span className="configuration-legend">{view === "geometry" ? "Układy lokalne: X czerwony · Y zielony · Z niebieski" : "Łuk: zakres przegubu · dymek: przełożenie i granice w °"}</span>
        </div>
    </section>;
}
