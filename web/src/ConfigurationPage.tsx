import { useCallback, useEffect, useRef, useState, type ReactNode } from "react";
import { EventConsole, useEventConsole } from "./EventConsole";
import { ConfigurationPreview } from "./ConfigurationPreview";
import { dispatchModeKey, criticalFaultPolicyKey, systemConfigurationKeys, dirtyKeys, fieldErrors } from "./configurationData";
import { useConfiguration } from "./useConfiguration";
import { useSystemSettings } from "./planningContext";

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
const tcpMotionFields: Field[] = [
    {key:"max_linear_velocity",label:"Maksymalna prędkość przesuwu TCP",numeric:true,min:0,unit:"mm/s"},
    {key:"max_angular_velocity",label:"Maksymalna prędkość obrotu TCP",numeric:true,min:0,unit:"°/s"},
    {key:"linear_acceleration",label:"Przyspieszenie i hamowanie przesuwu TCP",numeric:true,min:0,unit:"mm/s²"},
    {key:"angular_acceleration",label:"Przyspieszenie i hamowanie obrotu TCP",numeric:true,min:0,unit:"°/s²"},
];
// v1: metry i radiany; v2: milimetry i stopnie. Wersja zapobiega podwójnemu przeliczeniu.
const storageKey = "robot.configuration.draft.v2";
const legacyStorageKey = "robot.configuration.draft.v1";
const draftFields: Record<string, Field> = Object.fromEntries([
    ...robotFields.map(field => [`robot.${field.key}`, field]),
    ...Array.from({ length: 6 }, (_, index) => [...jointFields, ...geometryFields].map(field => [`joint.${index + 1}.${field.key}`, field])).flat(),
    ...toolFields.map(field => [`tool.${field.key}`, field]),
    ...tcpMotionFields.map(field => [`tcp.${field.key}`, field]),
    [dispatchModeKey, {key:"ptp_lin_dispatch_mode",label:"Wysyłanie PTP/LIN"}],
    [criticalFaultPolicyKey, {key:"disable_enable_on_critical_fault",label:"Wyłącz ENABLE przy awarii krytycznej"}],
]);
const emptyDraft: Record<string, string> = Object.fromEntries(Object.keys(draftFields).map(key => [key, key===dispatchModeKey ? "streamed" : key===criticalFaultPolicyKey ? "true" : ""]));
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
            const value = stored && typeof stored[key] === "string" ? stored[key].slice(0, 256) : key===dispatchModeKey ? "streamed" : key===criticalFaultPolicyKey ? "true" : "";
            return [key, legacy ? legacyValue(value, field) : value];
        }));
    } catch { return { ...emptyDraft }; }
}
function SettingsCard({ title, description, children }: { title: string; description: string; children: ReactNode }) {
    return <details className="settings-card settings-extra">
        <summary>{title}</summary><p className="settings-description">{description}</p>{children}
    </details>;
}

function SettingHelp({id, label, children}: {id: string; label: string; children: ReactNode}) {
    const [open, setOpen] = useState(false);
    const button = useRef<HTMLButtonElement>(null);
    const [anchor, setAnchor] = useState<{left: number; top?: number; bottom?: number}>({left: 12, top: 0});
    function show() {
        const rect = button.current?.getBoundingClientRect();
        if (rect) setAnchor({left: Math.max(12, Math.min(rect.left, window.innerWidth - Math.min(340, window.innerWidth - 24) - 12)),
            ...(rect.bottom > window.innerHeight - 190 ? {bottom: window.innerHeight - rect.top + 8} : {top: rect.bottom + 8})});
        setOpen(true);
    }
    useEffect(() => {
        if (!open) return;
        const hide = () => setOpen(false);
        window.addEventListener("scroll", hide, true);window.addEventListener("resize", hide);
        return () => {window.removeEventListener("scroll", hide, true);window.removeEventListener("resize", hide);};
    }, [open]);
    return <span className="system-setting-help" onPointerEnter={show} onPointerLeave={() => setOpen(false)}>
        <button ref={button} type="button" className="system-setting-info" aria-label={`Opis: ${label}`} aria-describedby={`${id}-help`}
            aria-expanded={open} onFocus={show} onBlur={() => setOpen(false)} onClick={show}
            onKeyDown={event => {if (event.key === "Escape") {event.preventDefault();setOpen(false);}}}>i</button>
        <span id={`${id}-help`} role="tooltip" style={anchor} hidden={!open}>{children}</span>
    </span>;
}
function SystemSetting({id, label, description, status, pending = false, group = false, children}: {
    id: string; label: string; description: string; status: string; pending?: boolean; group?: boolean; children: ReactNode;
}) {
    return <div className="system-setting" data-pending={pending}>
        <div className="system-setting-copy"><div className="system-setting-title">{group ? <span id={`${id}-label`}>{label}</span> : <label htmlFor={id}>{label}</label>}
            <SettingHelp id={id} label={label}>{description}</SettingHelp></div>
            <span className={`system-setting-status${pending ? " pending" : ""}`}>{status}</span>
        </div>
        <div className="system-setting-control">{children}</div>
    </div>;
}

export function ConfigurationPage() {
    const system = useSystemSettings();
    const [section, setSection] = useState<"system" | "robot">("system");
    const [tableView, setTableView] = useState<"geometry" | "limits">("geometry");
    const [draft, setDraft] = useState(readDraft);
    const [nativeErrors, setNativeErrors] = useState<Record<string, string>>({});
    const replaceDraft = useCallback((next: Record<string,string>, preserveValidation = false) => {setDraft(next);if (!preserveValidation) setNativeErrors({});}, []);
    const [stored, setStored] = useState(true);
    const [selectedAxis, setSelectedAxis] = useState(0);
    const [confirmAxes, setConfirmAxes] = useState<number[]>([]);
    const {entries, notice} = useEventConsole();
    const hasLocal = useRef((() => {try { return sessionStorage.getItem(storageKey) !== null || sessionStorage.getItem(legacyStorageKey) !== null; } catch { return true; }})());
    const fileInput = useRef<HTMLInputElement>(null);
    const configuration = useConfiguration(draft, replaceDraft, true, hasLocal.current, notice, section === "robot");
    const dirty = dirtyKeys(draft, configuration.baseline), errors = {...fieldErrors(draft), ...nativeErrors}, invalid = Object.keys(errors).length > 0;
    const applying = configuration.status && !["idle", "done", "failed", "uncertain"].includes(configuration.status.operation.phase);
    const systemDirty = systemConfigurationKeys.filter(key => dirty.has(key));
    const systemError = systemConfigurationKeys.map(key => errors[key]).find(Boolean);
    const conflict = configuration.status && configuration.baseRevision !== configuration.status.active.revision;
    const invalidCalibration = configuration.status?.calibration_invalid ?? [];
    useEffect(() => {setConfirmAxes(previous => previous.filter(axis => invalidCalibration[axis-1]));}, [configuration.status]);
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
        const change = (value: string, bad = false) => {
            setDraft(previous => ({ ...previous, [key]: value.slice(0, 256) }));
            setNativeErrors(previous => {const next = {...previous};if(bad)next[key] = "Dokończ wpisywanie poprawnej liczby";else delete next[key];return next;});
        };
        const attributes = {className: `${dirty.has(key) ? "field-dirty" : ""} ${errors[key] ? "field-invalid" : ""}`, "data-dirty":dirty.has(key), "aria-invalid": !!errors[key], title:errors[key] || (dirty.has(key) ? "Zmieniono względem konfiguracji ROS" : ""), onFocus: () => setSelectedAxis(axis ?? 0), onMouseEnter: () => setSelectedAxis(axis ?? 0)};
        return field.options ? <select {...attributes} id={id} aria-label={label} value={draft[key]} onChange={event => change(event.target.value)}>
            <option value="">{axis ? "—" : "Nie określono"}</option>{field.options.map(([value, text]) => <option key={value} value={value}>{text}</option>)}
        </select> : <input {...attributes} id={id} aria-label={label} type={field.numeric ? "number" : "text"} step={field.numeric ? "any" : undefined}
            min={field.min} maxLength={256} placeholder={axis ? "—" : field.placeholder ?? "Nie określono"} value={draft[key]}
            onChange={event => change(event.target.value, event.currentTarget.validity.badInput)} />;
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
        <header className="page-title configuration-header"><div><div className="eyebrow">{section === "system" ? "USTAWIENIA APLIKACJI I STEROWANIA" : "PARAMETRY KINEMATYKI I NAPĘDÓW"}</div><h1>Konfiguracja</h1><p>{section === "system" ? "Opcje planowania, wykonania ruchu i zabezpieczeń." : "Geometria, przełożenia i ograniczenia sześciu osi. Snapshoty i zastosowanie do ROS."}</p></div>
            <div className="page-header-status"><span className={`badge ${dirty.size ? "warn" : "ok"}`}>{dirty.size ? `Szkic · ${dirty.size} zmian` : "Bez zmian w szkicu"}</span>{conflict && <span className="badge error">Konflikt rewizji</span>}</div></header>
        <EventConsole entries={entries}/>
        <div className="configuration-tabs" role="tablist" aria-label="Obszar konfiguracji" onKeyDown={event => {
            if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
            event.preventDefault();
            const next = event.key === "Home" ? "system" : event.key === "End" ? "robot" : section === "system" ? "robot" : "system";
            setSection(next); document.getElementById(`config-${next}-tab`)?.focus();
        }}>
            <button id="config-system-tab" role="tab" tabIndex={section === "system" ? 0 : -1} aria-selected={section === "system"} aria-controls="config-system-panel" onClick={() => setSection("system")}>
                Ustawienia systemowe<small>Ustawienia wspólne aplikacji</small>
            </button>
            <button id="config-robot-tab" role="tab" tabIndex={section === "robot" ? 0 : -1} aria-selected={section === "robot"} aria-controls="config-robot-panel" onClick={() => setSection("robot")}>
                Parametry robota<small>Dane maszynowe J1–J6</small>
            </button>
        </div>
        {section === "robot" && <p className="configuration-notice" role="status">{stored
            ? "Szkic i model aktualizują się lokalnie. Bursztynowe pola różnią się od konfiguracji ROS; snapshot nie stosuje zmian w sterowaniu."
            : "Przeglądarka blokuje zapis szkicu. Wpisy pozostaną tylko do odświeżenia lub opuszczenia strony."}</p>}
        {section === "system" ? <div id="config-system-panel" role="tabpanel" aria-labelledby="config-system-tab" className="system-settings-panel">
            <section className="system-settings-group" aria-labelledby="system-planning-title">
                <header><span className="eyebrow">PLANOWANIE I WYKONANIE</span><h2 id="system-planning-title">Dostępność napędów</h2></header>
                <SystemSetting id="system-ignore-missing" label="Ignoruj brak sterowników — tylko planowanie"
                    description="Odblokowuje JOG wszystkich osi bliźniaka. Ustawienie jest wspólne dla klientów i nie zmienia sterowania sprzętem. Zmiana zapisuje się od razu."
                    status={!system.settings ? "Brak danych" : system.settings.ignore_missing_drivers ? "Włączone" : "Wyłączone"}>
                    <input id="system-ignore-missing" className="system-setting-switch" type="checkbox" role="switch"
                        checked={system.settings?.ignore_missing_drivers ?? false} disabled={!system.settings || system.saving}
                        onChange={event => void system.save(event.target.checked)}/>
                </SystemSetting>
                <SystemSetting id="system-allow-missing" label="Zezwalaj na ruch bez wszystkich silników"
                    description="PTP/LIN obejmuje napędy obecne przy przygotowaniu programu. Brakujące osie są pomijane; pozycja i osiągnięcie celu TCP nie są potwierdzone. Utrata uczestniczącego napędu zatrzymuje ruch. Zmiana zapisuje się od razu."
                    status={!system.settings ? "Brak danych" : system.settings.allow_missing_motion ? "Włączone" : "Wyłączone"}>
                    <input id="system-allow-missing" className="system-setting-switch" type="checkbox" role="switch"
                        checked={system.settings?.allow_missing_motion ?? false} disabled={!system.settings || system.saving}
                        onChange={event => void system.save(event.target.checked, "allow_missing_motion")}/>
                </SystemSetting>
                {system.error && <p className="system-settings-error" role="status">{system.error}</p>}
            </section>
            <section className="system-settings-group" aria-labelledby="system-motion-title">
                <header><span className="eyebrow">STEROWANIE ROBOTEM</span><h2 id="system-motion-title">Wykonanie i zabezpieczenia</h2></header>
                <SystemSetting id="system-critical-fault" label="Wyłącz ENABLE wszystkich silników przy awarii krytycznej"
                    description="Domyślnie włączone. Dotyczy awarii sterowania osi, TCP i programów; wysyła ENABLE=0 do J1–J6. Wysłanie komend nie potwierdza wyłączenia napędów. Zwykły STOP operatora nie uruchamia reguły. Wymaga zapisu do ROS."
                    pending={dirty.has(criticalFaultPolicyKey)} status={dirty.has(criticalFaultPolicyKey) ? "Niezapisana zmiana" : !configuration.online ? "Szkic lokalny" : draft[criticalFaultPolicyKey] !== "false" ? "Włączone" : "Wyłączone"}>
                    <input id="system-critical-fault" className="system-setting-switch" type="checkbox" role="switch" aria-invalid={!!errors[criticalFaultPolicyKey]}
                        checked={draft[criticalFaultPolicyKey] !== "false"} disabled={Boolean(applying) || configuration.busy}
                        onChange={event => setDraft(previous => ({...previous, [criticalFaultPolicyKey]: String(event.target.checked)}))}/>
                </SystemSetting>
                <SystemSetting id="system-dispatch" group label="Sposób wysyłania PTP/LIN"
                    description="Streaming synchroniczny przygotowuje komendy osi i uruchamia je wspólnym SYNC. Dotyczy programów PTP/LIN; JOG zachowuje obecne działanie. Wymaga osobnego odbioru płynności na sprzęcie. Wybór stosuje się po zapisie do ROS w potwierdzonym postoju; zmiana unieważnia przygotowany program."
                    pending={dirty.has(dispatchModeKey)} status={dirty.has(dispatchModeKey) ? "Niezapisana zmiana" : !configuration.online ? "Szkic lokalny" : "Zapisane w konfiguracji"}>
                    <div id="system-dispatch" className="system-setting-choice" role="radiogroup" aria-label="Sposób wysyłania PTP/LIN" aria-invalid={!!errors[dispatchModeKey]}>
                        {[["streamed", "Streaming"], ["synchronized", "Synchroniczny"]].map(([value, label]) => <label key={value}>
                            <input type="radio" name="ptp-lin-dispatch" value={value} checked={(draft[dispatchModeKey] || "streamed") === value}
                                disabled={Boolean(applying) || configuration.busy} onChange={() => setDraft(previous => ({...previous, [dispatchModeKey]: value}))}/>
                            <span>{label}</span>
                        </label>)}
                    </div>
                </SystemSetting>
                <footer className="system-settings-actions"><span>Zabezpieczenie i sposób wysyłania wymagają zapisu do ROS.</span>
                    <button className="configuration-apply" onClick={configuration.applySystemSettings}
                        disabled={!configuration.online || configuration.busy || !!applying || !!conflict || !systemDirty.length || !!systemError || configuration.status?.operation.phase === "uncertain"}>
                        {applying ? "Stosowanie…" : "Zapisz ustawienia do ROS"}
                    </button>
                    {configuration.status?.operation.phase === "uncertain" && <button onClick={configuration.reconcile} disabled={configuration.busy}>Uzgodnij stan ROS</button>}
                </footer>
                {systemError && <p className="system-settings-error" role="status">{systemError}</p>}
                {conflict && <p className="system-settings-error" role="status">Konfiguracja ROS zmieniła się. Uzgodnij szkic w Parametrach robota przed zapisem.</p>}
            </section>
        </div> : <div id="config-robot-panel" role="tabpanel" aria-labelledby="config-robot-tab" className="robot-settings">
            <div className="snapshot-toolbar" aria-label="Snapshoty konfiguracji">
                <input aria-label="Nazwa snapshotu" placeholder="Nazwa snapshotu" maxLength={80} value={configuration.name} onChange={event => configuration.setName(event.target.value)}/>
                <button disabled={!configuration.online || configuration.busy || invalid} onClick={configuration.saveSnapshot}>Zapisz snapshot</button>
                <select aria-label="Zapisany snapshot" value={configuration.selected} onChange={event => configuration.setSelected(event.target.value)}><option value="">Wybierz snapshot</option>{configuration.items.map(item => <option key={item.id} value={item.id}>{item.name} · {new Date(item.created_at).toLocaleString("pl-PL")}</option>)}</select>
                <button disabled={!configuration.selected || configuration.busy} onClick={configuration.loadSnapshot}>Wczytaj</button>
                <button disabled={!configuration.selected || configuration.busy} onClick={configuration.deleteSnapshot}>Usuń</button>
                <button aria-label="Poprzednia strona snapshotów" disabled={configuration.cursors.length < 2 || configuration.busy} onClick={() => configuration.page(false)}>‹</button><button aria-label="Następna strona snapshotów" disabled={!configuration.nextCursor || configuration.busy} onClick={() => configuration.page(true)}>›</button>
                <input hidden ref={fileInput} type="file" accept=".json,application/json" onChange={event => {const file = event.target.files?.[0]; if(file) void configuration.importFile(file); event.target.value = "";}}/>
                <button onClick={() => fileInput.current?.click()} disabled={configuration.busy}>Importuj JSON</button>
                <button onClick={configuration.exportFile} disabled={configuration.busy || invalid}>Eksportuj JSON</button>
                <button onClick={configuration.example}>Wczytaj przykład</button>
                <button onClick={configuration.loadActive} disabled={!configuration.online || configuration.busy}>Wczytaj aktywną</button>
                <button className="configuration-apply" onClick={configuration.apply} disabled={!configuration.online || configuration.busy || !!applying || !!conflict || invalid || configuration.status?.operation.phase === "uncertain"}>{applying ? "Stosowanie…" : "Zapisz do ROS"}</button>
                {configuration.status?.operation.phase === "uncertain" && <button onClick={configuration.reconcile} disabled={configuration.busy}>Uzgodnij stan ROS</button>}
            </div>
            {invalid && <p className="configuration-validation" role="status">Popraw {Object.keys(errors).length} pól: {Object.values(errors)[0]}</p>}
            {invalidCalibration.some(Boolean) && <section aria-label="Potwierdzenie odniesienia osi">
                <p>Te osie wymagają potwierdzenia odniesienia. Sprawdź fizyczną pozycję i zapisany offset. Potwierdzenie wymaga postoju i ENABLE=0; zapis dotyczy aktywnej konfiguracji.</p>
                {invalidCalibration.map((value, i) => value && <label key={i}>
                    <input type="checkbox" checked={confirmAxes.includes(i+1)} onChange={event => setConfirmAxes(previous => event.target.checked ? [...previous, i+1] : previous.filter(axis => axis !== i+1))}/>
                    Potwierdzam odniesienie J{i+1}
                </label>)}
                <button disabled={!confirmAxes.length || !configuration.online || configuration.busy || !!applying || !!conflict || configuration.status?.operation.phase === "uncertain"}
                    onClick={() => void configuration.confirmReference(confirmAxes)}>Potwierdź odniesienie wybranych osi</button>
            </section>}
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
                <p className="machine-note">{tableView === "geometry" ? "Translacja i rotacja względem poprzedniego członu." : "Zmiana przełożenia zachowuje bieżący kąt przez przeliczenie offsetu. Zmianę offsetu lub kierunku wykonaj osobno. Bez pomiaru odniesienie pozostaje niepotwierdzone."}</p>
            </section>
            <ConfigurationPreview draft={draft} view={tableView} selected={selectedAxis} nativeErrors={nativeErrors}/>
            <div className="settings-extras">
                <SettingsCard title="Fizyczny JOG TCP — wymagane limity" description="Uzupełnij cztery dodatnie wartości dla rzeczywistego robota i wybierz Zapisz do ROS. Puste pole oznacza brak danych i blokuje fizyczny TCP. Wymagane są również kompletna geometria, sześć składników TCP narzędzia oraz sześć skalibrowanych osi.">{fields("tcp", tcpMotionFields)}</SettingsCard>
                <SettingsCard title="Dane robota" description="Nazwy i układy odniesienia wspólne dla modelu.">{fields("robot", robotFields)}</SettingsCard>
                <SettingsCard title="Narzędzie i TCP" description="TCP względem kołnierza; środek masy w układzie narzędzia.">{fields("tool", toolFields)}</SettingsCard>
            </div>
        </div>}
    </section>;
}
