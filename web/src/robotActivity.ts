import { createContext, useContext, useEffect, useSyncExternalStore } from "react";
import type { ManualState } from "./manualTypes";
import { motionStateLabels, type MotionStatus } from "./motionStatus";
import type { MessageCenter } from "./messageCenter";
import type { Notice } from "./eventLog";
import { previewLimits } from "./previewLimits";

// Czynność robota w pasku komunikatów: co robot fizycznie robi albo co robił, gdy się zatrzymał.
// Stan programu celowo pomijany (osobny ekran); liczy się ruch, postój, operacja na osi, awaria i odzyskiwanie.
export type ActivityKind = "move" | "operation" | "idle" | "fault" | "recovery" | "unknown";
export type AxisActivityState = "moving" | "waiting" | "disabled" | "unknown";
export type AxisActivity = { axis: number; state: AxisActivityState; position: number | null; joint: boolean; speedRpm: number | null };
export type RobotActivity = { kind: ActivityKind; label: string; detail: string; axes: AxisActivity[] };
export type ActivityEvent = { at: number; kind: ActivityKind; label: string; detail: string };
export type ActivitySnapshot = RobotActivity & { since: number | null; log: ActivityEvent[] };

const operationLabels: Record<string, string> = { enable: "ENABLE", zero: "Zerowanie", stopping: "Zatrzymywanie", jog: "JOG" };
const shortReason = (text: string) => { const first = text.split(";")[0].trim(); return first.length > 110 ? `${first.slice(0, 107)}…` : first; };
const recovering = (state?: string) => !!state && state !== "idle" && state !== "idle_disabled";

export function deriveActivity(manual: ManualState | null, motion: MotionStatus | null): RobotActivity {
    const axes: AxisActivity[] = Array.from({ length: 6 }, (_, index) => {
        const axis = manual?.fresh ? manual.axes?.[index] : undefined;
        if (!axis || !axis.available) return { axis: index + 1, state: "unknown", position: null, joint: false, speedRpm: null };
        const joint = !!axis.joint_configured && !!axis.joint_position_known;
        const position = joint ? axis.joint_position_degrees ?? null : axis.position_known ? axis.position_degrees : null;
        const state: AxisActivityState = axis.motion === "w prędkości" ? "moving" : axis.enable_known && !axis.enabled ? "disabled" : axis.enable_known ? "waiting" : "unknown";
        return { axis: index + 1, state, position, joint, speedRpm: axis.speed_known ? axis.speed_rpm : null };
    });
    if (!manual || !manual.fresh) return { kind: "unknown", label: "BRAK DANYCH", detail: "brak aktualnego stanu sterowania", axes };
    if (manual.fault) return { kind: "fault", label: "ZATRZYMANY", detail: shortReason(manual.message || "awaria bez opisu"), axes };
    if (recovering(manual.recovery_state)) return { kind: "recovery", label: manual.recovery_state === "recovery_failed" ? "ODZYSKIWANIE NIEUDANE" : "ODZYSKIWANIE",
        detail: shortReason(manual.recovery_message || "odzyskiwanie po STOP"), axes };
    if (motion?.busy) return { kind: "move", label: "RUCH", detail: `${motion.motion_type ?? "Ruch"} · ${(motionStateLabels[motion.state] ?? motion.state).toLowerCase()}`, axes };
    if (manual.tcp_jog?.busy) return { kind: "move", label: "RUCH", detail: "JOG TCP", axes };
    if (manual.operation === "jog" && manual.active_axis) return { kind: "move", label: "RUCH", detail: `JOG J${manual.active_axis}`, axes };
    if (manual.busy && manual.operation && manual.operation !== "idle")
        return { kind: "operation", label: "OPERACJA", detail: `${operationLabels[manual.operation] ?? manual.operation}${manual.active_axis ? ` J${manual.active_axis}` : ""}`, axes };
    const moving = axes.filter(axis => axis.state === "moving").map(axis => `J${axis.axis}`);
    if (moving.length) return { kind: "move", label: "RUCH", detail: `w ruchu: ${moving.join(", ")}`, axes };
    const enabled = manual.axes.filter(axis => axis.enable_known && axis.enabled).length;
    return { kind: "idle", label: "POSTÓJ", detail: `ENABLE ${enabled}/6${manual.recovery_state === "idle_disabled" ? " · gotowy do ponownego załączenia" : ""}`, axes };
}

// Komunikaty stanu robota — jedno źródło dla Sterowania ręcznego (stan z WS, pełna częstotliwość) i globalnego odczytu
// paska (co 1 s, gdy Sterowanie jest zamknięte), dzięki czemu są widoczne na każdej stronie z tymi samymi kluczami
// i treścią. Przy nieświeżych danych zwraca tylko „freshness”, a pozostałe warunki zachowują ostatnią ocenę.
export type RobotCondition = [key: string, notice: Notice | null, resolution?: string];
const ROBOT_KEYS = new Set(["freshness", "axis-feedback", "controller", "recovery", "tcp-live-fault", "axes", "drive-enable"]);
export const isRobotStateKey = (key: string) => ROBOT_KEYS.has(key) || /^(profile-[1-6]|limit-[1-6]-(lower|upper))$/.test(key);
export function robotConditions(state: ManualState | null, fresh = state?.fresh === true): RobotCondition[] {
    const out: RobotCondition[] = [["freshness", fresh ? null : { source: "freshness", severity: "warning", code: "freshness", context: "Pomiary robota", message: "Dane sterowania nieaktualne — JOG zablokowany do powrotu pomiarów." }]];
    if (!fresh || !state) return out;
    const withoutFeedback = state.axes.filter(a => a.available && (!a.position_known || !Number.isFinite(a.position_age_seconds) || (a.position_age_seconds ?? Infinity) >= .25 || (a.position_age_seconds ?? 0) < 0 || (a.joint_configured && a.calibration_valid && !a.joint_position_known))).map(a => `J${a.axis}`);
    out.push(["axis-feedback", withoutFeedback.length ? { source: "axis-feedback", severity: "warning", code: "freshness", context: "Pomiary osi", description: `Brakuje aktualnej pozycji osi: ${withoutFeedback.join(", ")}. Stan połączenia panelu nie potwierdza świeżości tych pomiarów.`, message: `Brak świeżej pozycji: ${withoutFeedback.join(", ")}.` } : null, "Zaktualizowano ocenę pomiarów dostępnych osi."]);
    out.push(["controller", state.fault ? { source: "controller", severity: "error", code: "fault", context: "Robot fizyczny", message: state.message || "Brak opisu awarii" } : null]);
    out.push(["recovery", recoveryNotice(state)]);
    out.push(["tcp-live-fault", !state.fault && state.tcp_jog?.state === "failed" ? { source: "tcp-live-fault", severity: "error", code: "fault", context: "JOG TCP LIVE", message: state.tcp_jog.message || "Przerwano JOG TCP bez szczegółowego opisu" } : null]);
    const available = state.axes.filter(axis => axis.available).map(axis => `J${axis.axis}`);
    const missing = state.axes.filter(axis => !axis.available).map(axis => `J${axis.axis}`);
    out.push(["axes", missing.length ? { source: "axes", severity: "warning", code: "missing", context: "Robot fizyczny", description: `Nie dostarczają pozycji: ${missing.join(", ")}. Dostępne napędy: ${available.join(", ") || "brak"}.`, message: `Dostępne napędy: ${available.join(", ") || "brak"}. Niedostępne: ${missing.join(", ")}.` } : null]);
    const disabledAxes = state.axes.filter(a => a.available && (!a.enable_known || !a.enabled)).map(a => `J${a.axis}`);
    out.push(["drive-enable", disabledAxes.length ? { source: "drive-enable", severity: "warning", code: "enable", context: "Robot fizyczny", description: `Wyłączony lub nieznany ENABLE osi ${disabledAxes.join(", ")} uniemożliwia ich fizyczny ruch.`, message: `ENABLE wyłączony lub nieznany: ${disabledAxes.join(", ")}.` } : null]);
    for (const axis of state.axes) {
        const invalid = axis.joint_configured && !axis.calibration_valid, unprepared = axis.joint_configured && axis.enabled && !axis.prepared;
        out.push([`profile-${axis.axis}`, invalid || unprepared ? { source: `profile-${axis.axis}`, severity: "warning", code: invalid ? "calibration" : "profile", context: `Oś J${axis.axis}`, message: `J${axis.axis}: ${axis.configuration_message || (invalid ? "Kalibracja nieważna" : "Napęd nieprzygotowany")}` } : null]);
        const q = axis.joint_configured ? axis.joint_position_known ? axis.joint_position_degrees! : null : axis.position_known ? axis.position_degrees : null;
        const limits = previewLimits(axis, q, true);
        for (const side of ["lower", "upper"] as const) {
            const limit = limits.find(l => l.side === side);
            out.push([`limit-${axis.axis}-${side}`, limit ? { source: `limit-${axis.axis}-${side}`, severity: limit.level === "exceeded" ? "error" : "warning", code: "limits", context: `Robot · J${axis.axis}`, title: limit.level === "exceeded" ? "Przekroczony limit osi" : "Blisko limitu osi", message: `J${axis.axis}: ${side === "lower" ? "dolny" : "górny"} limit ${limit.limitDegrees}° · ${limit.level === "exceeded" ? "przekroczony" : "zbliżenie"}.` } : null]);
        }
    }
    return out;
}
export function recoveryNotice(manual: Pick<ManualState, "recovery_state" | "recovery_message" | "message" | "recovery_pending_mask">): Notice | null {
    if (!recovering(manual.recovery_state)) return null;
    const failed = manual.recovery_state === "recovery_failed";
    const pending = [1, 2, 3, 4, 5, 6].filter(axis => ((manual.recovery_pending_mask ?? 0) >> (axis - 1)) & 1).map(axis => `J${axis}`).join(", ");
    return { source: "recovery", severity: failed ? "error" : "warning", context: "Robot fizyczny",
        title: failed ? "Odzyskiwanie nieudane" : "Odzyskiwanie po STOP",
        message: `${manual.recovery_message || manual.message || (failed ? "Odzyskiwanie po awarii nie zostało potwierdzone." : "Trwa odzyskiwanie po STOP.")}${pending ? ` · oczekujące osie: ${pending}` : ""}`,
        description: failed ? "System nie potwierdził bezpiecznego stanu napędów po zatrzymaniu. Ruch jest zablokowany." : "Napędy są zatrzymywane i weryfikowane po STOP. Ruch jest zablokowany do zakończenia.",
        remedy: failed ? "Sprawdź zasilanie i połączenia wskazanej osi, potem użyj „Reset CAN” — odzyskiwanie uruchomi się od nowa." : "Zaczekaj na zakończenie odzyskiwania i sprawdź stan osi przed ponownym załączeniem.",
        href: "#diagnostics" };
}

export class ActivityStore {
    private listeners = new Set<() => void>();
    private raw: RobotActivity = deriveActivity(null, null);
    private current: RobotActivity = this.raw;
    private after: string | null = null;
    private since: number | null = null;
    private log: ActivityEvent[] = [];
    private snapshot: ActivitySnapshot | null = null;
    private provided: MotionStatus | null = null;
    constructor(private readonly clock: () => number = Date.now) {}
    // Stan ruchu PTP/LIN odbierany już przez Sterowanie ręczne (bez drugiego odpytywania /motion/status).
    provideMotion(motion: MotionStatus | null) { this.provided = motion; }
    subscribe = (listener: () => void) => { this.listeners.add(listener); return () => { this.listeners.delete(listener); }; };
    getSnapshot = (): ActivitySnapshot => this.snapshot ??= { ...this.current, since: this.since, log: this.log };
    update(manual: ManualState | null, motion: MotionStatus | null | undefined) {
        const raw = deriveActivity(manual, motion === undefined ? this.provided : motion), now = this.clock();
        if (raw.kind !== this.raw.kind || raw.label !== this.raw.label || raw.detail !== this.raw.detail) {
            // Postój po ruchu lub operacji: „po JOG J2” — co robot robił przed zatrzymaniem (do następnej czynności).
            if (raw.kind !== "idle") this.after = null;
            else if (this.raw.kind === "move" || this.raw.kind === "operation") this.after = this.raw.detail;
            if (raw.kind !== "unknown" || this.raw.kind !== "unknown") {
                this.log = [{ at: now, kind: raw.kind, label: raw.label, detail: raw.detail }, ...this.log].slice(0, 30);
                if (raw.kind !== this.raw.kind) this.since = now;
            }
            this.raw = raw;
        }
        const next = { ...raw, detail: raw.kind === "idle" && this.after ? `po ${this.after} · ${raw.detail}` : raw.detail };
        const changed = JSON.stringify(next) !== JSON.stringify(this.current);
        this.current = next;
        if (changed) { this.snapshot = null; for (const listener of [...this.listeners]) listener(); }
    }
}

export const defaultActivityStore = new ActivityStore();
export const ActivityContext = createContext<ActivityStore>(defaultActivityStore);
export function useActivity(): ActivitySnapshot {
    const store = useContext(ActivityContext);
    return useSyncExternalStore(store.subscribe, store.getSnapshot, store.getSnapshot);
}

async function getJson<T>(url: string, signal: AbortSignal): Promise<T | null> {
    try { const response = await fetch(url, { signal, cache: "no-store" }); return response.ok ? await response.json() as T : null; }
    catch { return null; }
}

// Lekki odczyt stanu robota co 1 s na każdej stronie (wstrzymany, gdy karta jest w tle).
export function useRobotFeed(center: MessageCenter, store: ActivityStore, periodMs = 1000) {
    useEffect(() => {
        let stopped = false, timer: ReturnType<typeof setTimeout> | undefined, controller: AbortController | null = null;
        const tick = async () => {
            if (!document.hidden) {
                controller = new AbortController();
                const abort = setTimeout(() => controller?.abort(), Math.max(300, periodMs - 100));
                // Przy otwartym Sterowaniu ręcznym stan ruchu podaje moduł (provideMotion) — bez podwójnego odpytywania.
                const shared = center.manualActive;
                const [manual, motion] = await Promise.all([getJson<ManualState>("/api/v1/manual/status", controller.signal),
                    shared ? Promise.resolve(undefined) : getJson<MotionStatus>("/api/v1/motion/status", controller.signal)]);
                clearTimeout(abort);
                if (stopped) return;
                store.update(manual && Array.isArray(manual.axes) ? manual : null, motion === undefined ? undefined : motion && typeof motion.busy === "boolean" ? motion : null);
                if (manual && Array.isArray(manual.axes) && !center.manualActive) for (const [key, notice, resolution] of robotConditions(manual)) center.condition(key, notice, resolution);
            }
            if (!stopped) timer = setTimeout(() => void tick(), periodMs);
        };
        void tick();
        return () => { stopped = true; clearTimeout(timer); controller?.abort(); };
    }, [center, store, periodMs]);
}
