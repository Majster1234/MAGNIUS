export const controlKeys = ["gear_ratio", "direction", "zero_offset", "lower", "upper", "velocity", "acceleration"];
export const originKeys = ["origin_x", "origin_y", "origin_z", "origin_roll", "origin_pitch", "origin_yaw"];
export const tcpKeys = ["tcp_x", "tcp_y", "tcp_z", "tcp_roll", "tcp_pitch", "tcp_yaw"];
export const tcpMotionKeys = ["max_linear_velocity", "max_angular_velocity", "linear_acceleration", "angular_acceleration"];
export const textKeys = ["robot.name", "robot.base_frame", "robot.flange_frame", "tool.name", "tool.frame"];
export const criticalFaultPolicyKey = "safety.disable_enable_on_critical_fault";
export const configurationKeys = [...textKeys, "tool.mass", "tool.com_x", "tool.com_y", "tool.com_z", ...tcpKeys.map(key => `tool.${key}`),
    ...Array.from({length: 6}, (_, i) => [...originKeys, ...controlKeys].map(key => `joint.${i + 1}.${key}`)).flat(), ...tcpMotionKeys.map(key => `tcp.${key}`), criticalFaultPolicyKey];
export type Values = Record<string, number | string | boolean | null>;
export type Draft = Record<string, string>;
export type ConfigurationDocument = { schema_version: 1; units: "mm-deg-kg"; name: string; created_at: string; values: Values };
export type ConfigurationStatus = { active: { revision: string; document: ConfigurationDocument }; calibration_invalid?: boolean[]; operation: { id: string; phase: string; message: string; success: boolean | null } };
export const demoTranslations = [[0, 0, 100], [0, 0, 100], [0, 0, 250], [220, 0, 0], [80, 0, 0], [60, 0, 0]];
export const jointAxes = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0]];
export function emptyDocument(): ConfigurationDocument {
    return {schema_version: 1, units: "mm-deg-kg", name: "", created_at: new Date().toISOString(), values: {...Object.fromEntries(configurationKeys.map(key => [key, null])), [criticalFaultPolicyKey]:true}};
}
export function toDraft(document: ConfigurationDocument): Draft {
    return Object.fromEntries(configurationKeys.map(key => [key, key===criticalFaultPolicyKey ? String(document.values[key] ?? true) : document.values[key] == null ? "" : String(document.values[key])]));
}
export function valueOf(key: string, value: string): number | string | boolean | null {
    if (key===criticalFaultPolicyKey) return value==="false" ? false : value==="true" || !value.trim() ? true : value;
    if (!value.trim()) return null;
    return textKeys.includes(key) ? value.trim() : Number(value);
}
export function fieldErrors(draft: Draft): Record<string, string> {
    const errors: Record<string, string> = {};
    for (const key of configurationKeys) {
        const raw = draft[key] ?? "", value = valueOf(key, raw), field = key.split(".").at(-1);
        if (raw.length > 256) errors[key] = "Maksymalnie 256 znaków";
        if (key===criticalFaultPolicyKey) {
            if (typeof value!=="boolean") errors[key]="Wybierz włączenie albo wyłączenie reguły";
            continue;
        }
        if (value === null) continue;
        if (!textKeys.includes(key)) {
            if (!Number.isFinite(value) || Math.abs(Number(value)) > 1e9) errors[key] = "Wymagana skończona liczba w zakresie ±1e9";
            if (["gear_ratio", "velocity", "acceleration"].includes(field!) && Number(value) <= 0) errors[key] = "Wartość musi być dodatnia";
            if (key.startsWith("tcp.") && Number(value) <= 0) errors[key] = "Wartość musi być dodatnia";
            if (field === "direction" && value !== -1 && value !== 1) errors[key] = "Wybierz kierunek +1 albo −1";
            if (key === "tool.mass" && Number(value) < 0) errors[key] = "Masa nie może być ujemna";
        }
    }
    for (const [v, a] of [["max_linear_velocity", "linear_acceleration"], ["max_angular_velocity", "angular_acceleration"]]) {
        if (Number(draft[`tcp.${v}`]) > 0 && Number(draft[`tcp.${a}`]) > 0 && Number(draft[`tcp.${v}`]) / Number(draft[`tcp.${a}`]) > 10)
            errors[`tcp.${a}`] = "Profil wymaga hamowania dłuższego niż 10 s";
    }
    for (let axis = 1; axis <= 6; axis++) {
        const lo = `joint.${axis}.lower`, hi = `joint.${axis}.upper`;
        if (draft[lo]?.trim() && draft[hi]?.trim() && Number(draft[lo]) >= Number(draft[hi]))
            errors[lo] = errors[hi] = "Dolny limit musi być mniejszy od górnego";
    }
    const frameKeys = ["robot.base_frame", "robot.flange_frame", "tool.frame"], defaults = ["base_link", "link_6", "tool0"];
    const names = frameKeys.map((key, i) => draft[key]?.trim() || defaults[i]);
    for (let i = 0; i < names.length; i++) {
        if (!/^[A-Za-z_][A-Za-z0-9_]*$/.test(names[i]) || names.filter(name => name === names[i]).length > 1 || /^link_[1-5]$/.test(names[i]))
            errors[frameKeys[i]] = "Nazwa układu musi być unikalna; litery, cyfry i podkreślenia";
    }
    return errors;
}
export function documentFromDraft(draft: Draft, name = ""): ConfigurationDocument {
    const errors = fieldErrors(draft);
    if (Object.keys(errors).length) throw Error(Object.values(errors)[0]);
    if (name.length > 80) throw Error("Nazwa snapshotu: do 80 znaków");
    return {...emptyDocument(), name: name.trim(), values: Object.fromEntries(configurationKeys.map(key => [key, valueOf(key, draft[key] ?? "")]))};
}
export function validateDocument(value: unknown): ConfigurationDocument {
    if (!value || typeof value !== "object") throw Error("Wymagany obiekt JSON snapshotu");
    const doc = value as ConfigurationDocument;
    if (Object.keys(doc).some(key => !["schema_version", "units", "name", "created_at", "values"].includes(key)) || doc.schema_version !== 1 || doc.units !== "mm-deg-kg") throw Error("Nieobsługiwana wersja lub jednostki snapshotu");
    if (typeof doc.name !== "string" || doc.name.length > 80 || typeof doc.created_at !== "string" || !/([zZ]|[+-]\d\d:\d\d)$/.test(doc.created_at) || !Number.isFinite(Date.parse(doc.created_at))) throw Error("Nieprawidłowa nazwa lub data snapshotu");
    if (!doc.values || Array.isArray(doc.values) || Object.keys(doc.values).some(key => !configurationKeys.includes(key)) ||
        configurationKeys.some(key => key!==criticalFaultPolicyKey && !key.startsWith("tcp.") && !(key in doc.values))) throw Error("Nieprawidłowy zestaw pól snapshotu");
    const values:Values = { ...Object.fromEntries(tcpMotionKeys.map(key => [`tcp.${key}`, null])), [criticalFaultPolicyKey]:true, ...doc.values };
    for (const key of configurationKeys) {
        const field = values[key];
        if (key===criticalFaultPolicyKey) {
            if (typeof field!=="boolean") throw Error(`Nieprawidłowa wartość ${key}`);
            continue;
        }
        if (field !== null && (textKeys.includes(key) ? typeof field !== "string" : typeof field !== "number" || !Number.isFinite(field))) throw Error(`Nieprawidłowa wartość ${key}`);
    }
    const normalized = documentFromDraft(toDraft({...doc, values}), doc.name);
    return {...normalized, created_at: doc.created_at};
}
export function dirtyKeys(draft: Draft, baseline: ConfigurationDocument): Set<string> {
    return new Set(configurationKeys.filter(key => valueOf(key, draft[key] ?? "") !== (baseline.values[key] ?? (key===criticalFaultPolicyKey ? true : null))));
}
export function mergeAppliedDraft(current: Draft, submitted: Draft, applied: ConfigurationDocument): Draft {
    const resolved = toDraft(applied);
    return Object.fromEntries(configurationKeys.map(key => [key, current[key] === submitted[key] ? resolved[key] : current[key]]));
}
export function withExample(draft: Draft): Draft {
    const next = {...draft};
    for (let i = 0; i < 6; i++) originKeys.forEach((key, c) => next[`joint.${i + 1}.${key}`] = String(c < 3 ? demoTranslations[i][c] : 0));
    tcpKeys.forEach((key, i) => next[`tool.${key}`] = String(i === 0 ? 80 : 0));
    return next;
}
