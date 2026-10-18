import { useEffect, useState, type ReactNode } from "react";

type Field = {
    key: string; label: string; unit?: string; placeholder?: string; min?: number;
    options?: [string, string][]; numeric?: boolean;
};
const robotFields: Field[] = [
    { key: "name", label: "Nazwa robota" },
    { key: "base_frame", label: "Układ bazowy", placeholder: "np. base_link" },
    { key: "flange_frame", label: "Układ kołnierza", placeholder: "Nazwa układu w modelu" },
];
const jointFields: Field[] = [
    { key: "gear_ratio", label: "Przełożenie silnik / przegub", numeric: true, min: 0, unit: ":1" },
    { key: "direction", label: "Kierunek osi", options: [["1", "+1"], ["-1", "−1"]] },
    { key: "zero_offset", label: "Offset zera przegubu", numeric: true, unit: "°" },
    { key: "lower", label: "Dolny limit przegubu", numeric: true, unit: "°" },
    { key: "upper", label: "Górny limit przegubu", numeric: true, unit: "°" },
    { key: "velocity", label: "Maksymalna prędkość", numeric: true, min: 0, unit: "°/s" },
    { key: "acceleration", label: "Maksymalne przyspieszenie", numeric: true, min: 0, unit: "°/s²" },
];
const transformFields = (prefix: string): Field[] => [
    ...["X", "Y", "Z"].map(axis => ({ key: `${prefix}_${axis.toLowerCase()}`, label: `${prefix === "tcp" ? "TCP" : "Początek przegubu"} ${axis}`, numeric: true, unit: "mm" })),
    ...["Roll", "Pitch", "Yaw"].map(axis => ({ key: `${prefix}_${axis.toLowerCase()}`, label: `${prefix === "tcp" ? "TCP" : "Przegub"} ${axis}`, numeric: true, unit: "°" })),
];
const geometryFields = transformFields("origin").map((field, index) => ({
    ...field, label: index < 3 ? `Translacja ${["X", "Y", "Z"][index]}` : `Rotacja ${["X (Roll)", "Y (Pitch)", "Z (Yaw)"][index - 3]}`,
}));
const toolFields: Field[] = [
    { key: "name", label: "Nazwa narzędzia" },
    { key: "frame", label: "Układ narzędzia", placeholder: "Nazwa układu TCP" },
    { key: "mass", label: "Masa narzędzia", numeric: true, min: 0, unit: "kg" },
    ...transformFields("tcp"),
    ...["X", "Y", "Z"].map(axis => ({ key: `com_${axis.toLowerCase()}`, label: `Środek masy ${axis}`, numeric: true, unit: "mm" })),
];
// v1: metry i radiany; v2: milimetry i stopnie. Wersja zapobiega podwójnemu przeliczeniu.
const storageKey = "robot.configuration.draft.v2";
const legacyStorageKey = "robot.configuration.draft.v1";
const draftFields: Record<string, Field> = Object.fromEntries([
    ...robotFields.map(field => [`robot.${field.key}`, field]),
    ...Array.from({ length: 6 }, (_, index) => [...jointFields, ...geometryFields].map(field => [`joint.${index + 1}.${field.key}`, field])).flat(),
    ...toolFields.map(field => [`tool.${field.key}`, field]),
]);
const emptyDraft: Record<string, string> = Object.fromEntries(Object.keys(draftFields).map(key => [key, ""]));
function legacyValue(value: string, field: Field) {
    const scale = field.unit === "mm" ? 1000 : field.unit?.startsWith("°") ? 180 / Math.PI : 1;
    if (!value.trim() || scale === 1) return value;
    const converted = Number(value) * scale;
    return Number.isFinite(converted) ? String(Number(converted.toPrecision(15))) : "";
}
function readDraft() {
    try {
        const current = sessionStorage.getItem(storageKey), legacy = current === null;
        const text = legacy ? sessionStorage.getItem(legacyStorageKey) : current;
        const stored = text && text.length <= 65536 ? JSON.parse(text) : null;
        return Object.fromEntries(Object.entries(draftFields).map(([key, field]) => {
            const value = stored && typeof stored[key] === "string" ? stored[key].slice(0, 256) : "";
            return [key, legacy ? legacyValue(value, field) : value];
        }));
    } catch { return { ...emptyDraft }; }
}
function SettingsCard({ title, description, children }: { title: string; description: string; children: ReactNode }) {
    return <details className="settings-card settings-extra">
        <summary>{title}</summary><p className="settings-description">{description}</p>{children}
    </details>;
}

export function ConfigurationPage() {
    const [section, setSection] = useState<"system" | "robot">("system");
    const [tableView, setTableView] = useState<"geometry" | "limits">("geometry");
    const [draft, setDraft] = useState(readDraft);
    const [stored, setStored] = useState(true);
    useEffect(() => {
        try {
            sessionStorage.setItem(storageKey, JSON.stringify(draft));
            sessionStorage.removeItem(legacyStorageKey); setStored(true);
        }
        catch { setStored(false); }
    }, [draft]);
    function control(prefix: string, field: Field, axis?: number) {
        const key = `${prefix}.${field.key}`, id = `config-${key}`;
        const label = axis ? `${field.label} J${axis}${field.unit ? ` [${field.unit}]` : ""}` : undefined;
        const change = (value: string) => setDraft(previous => ({ ...previous, [key]: value.slice(0, 256) }));
        return field.options ? <select id={id} aria-label={label} value={draft[key]} onChange={event => change(event.target.value)}>
            <option value="">{axis ? "—" : "Nie określono"}</option>{field.options.map(([value, text]) => <option key={value} value={value}>{text}</option>)}
        </select> : <input id={id} aria-label={label} type={field.numeric ? "number" : "text"} step={field.numeric ? "any" : undefined}
            min={field.min} maxLength={256} placeholder={axis ? "—" : field.placeholder ?? "Nie określono"} value={draft[key]}
            onChange={event => change(event.target.value)} />;
    }
    function fields(prefix: string, definitions: Field[]) {
        return <div className="settings-fields">{definitions.map(field => {
            const key = `${prefix}.${field.key}`, id = `config-${key}`;
            return <label key={key} htmlFor={id} className="settings-field">
                <span>{field.label}{field.unit && <small>{field.unit}</small>}</span>
                {control(prefix, field)}
            </label>;
        })}</div>;
    }
    return <section className="configuration-screen" aria-label="Konfiguracja">
        <header className="configuration-header"><h1>Konfiguracja</h1><span className="badge warn">Szkic lokalny</span><a href="#overview">← Przegląd systemu</a></header>
        <div className="configuration-tabs" role="tablist" aria-label="Obszar konfiguracji" onKeyDown={event => {
            if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
            event.preventDefault();
            const next = event.key === "Home" ? "system" : event.key === "End" ? "robot" : section === "system" ? "robot" : "system";
            setSection(next); document.getElementById(`config-${next}-tab`)?.focus();
        }}>
            <button id="config-system-tab" role="tab" tabIndex={section === "system" ? 0 : -1} aria-selected={section === "system"} aria-controls="config-system-panel" onClick={() => setSection("system")}>
                Ustawienia systemowe<small>Miejsce na przyszłe ustawienia</small>
            </button>
            <button id="config-robot-tab" role="tab" tabIndex={section === "robot" ? 0 : -1} aria-selected={section === "robot"} aria-controls="config-robot-panel" onClick={() => setSection("robot")}>
                Parametry robota<small>Dane maszynowe J1–J6</small>
            </button>
        </div>
        {section === "robot" && <p className="configuration-notice" role="status">{stored
            ? "Szkic jest zachowywany w tej karcie przeglądarki. Zastosowanie ustawień w robocie będzie dostępne w kolejnym etapie."
            : "Przeglądarka blokuje zapis szkicu. Wpisy pozostaną tylko do odświeżenia lub opuszczenia strony."}</p>}
        {section === "system" ? <div id="config-system-panel" role="tabpanel" aria-labelledby="config-system-tab" className="settings-system-empty">
            <p>Ustawienia będą dodawane wraz z rozwojem projektu.</p>
        </div> : <div id="config-robot-panel" role="tabpanel" aria-labelledby="config-robot-tab" className="robot-settings">
            <section className="machine-panel" aria-label="Dane maszynowe">
                <div className="machine-toolbar"><h2>Dane maszynowe</h2>
                    <div className="machine-view-control">
                        <span className={tableView === "geometry" ? "selected" : ""}>Geometria</span>
                        <input type="range" min={0} max={1} step={1} value={tableView === "geometry" ? 0 : 1}
                            aria-label="Widok danych maszynowych" aria-valuetext={tableView === "geometry" ? "Geometria" : "Przełożenia i ograniczenia"}
                            onChange={event => setTableView(event.target.value === "0" ? "geometry" : "limits")} />
                        <span className={tableView === "limits" ? "selected" : ""}>Przełożenia i ograniczenia</span>
                    </div>
                </div>
                <div className="machine-table-scroll"><table className="machine-table" aria-label={tableView === "geometry" ? "Geometria osi" : "Przełożenia i ograniczenia osi"}>
                    <thead><tr><th scope="col">Parametr / jednostka</th>{Array.from({ length: 6 }, (_, index) => <th scope="col" key={index}>J{index + 1}{" "}<small>CAN {index + 1}</small></th>)}</tr></thead>
                    <tbody>{(tableView === "geometry" ? geometryFields : jointFields).map(field => <tr key={field.key}>
                        <th scope="row"><span>{field.label}</span><small>{field.unit}</small></th>
                        {Array.from({ length: 6 }, (_, index) => <td key={index}>{control(`joint.${index + 1}`, field, index + 1)}</td>)}
                    </tr>)}</tbody>
                </table></div>
                <p className="machine-note">{tableView === "geometry" ? "Translacja i rotacja względem poprzedniego członu." : "Przełożenie: obroty silnika na obrót przegubu. Limity dotyczą przegubu."}</p>
            </section>
            <div className="settings-extras">
                <SettingsCard title="Dane robota" description="Nazwy i układy odniesienia wspólne dla modelu.">{fields("robot", robotFields)}</SettingsCard>
                <SettingsCard title="Narzędzie i TCP" description="TCP względem kołnierza; środek masy w układzie narzędzia.">{fields("tool", toolFields)}</SettingsCard>
            </div>
        </div>}
    </section>;
}
