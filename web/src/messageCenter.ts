import { createContext, useCallback, useContext, useEffect, useSyncExternalStore } from "react";
import { ControlMessages, type CurrentMessage } from "./controlMessages";
import type { ConsoleEntry, Notice } from "./eventLog";

// Wspólny magazyn komunikatów całej aplikacji (pasek nad stroną). Źródła: Sterowanie ręczne (gdy otwarte),
// stan robota odczytywany globalnie oraz stan podzespołów systemu. Kwitowanie: wynik operacji znika z bieżących,
// warunek zostaje (przygaszony), dopóki przyczyna nie ustąpi. NOWY (`fresh`) = nieobejrzany i niepokwitowany: obejrzenie
// (najechanie na kartę w panelu) zdejmuje tylko znacznik, nie kwituje; zmiana treści komunikatu przywraca NOWY.
export type MessageModule = "Sterowanie" | "Napędy" | "CAN" | "Konfiguracja" | "Programowanie" | "System";
export const messageModules: MessageModule[] = ["Sterowanie", "Napędy", "CAN", "Konfiguracja", "Programowanie", "System"];

export function messageModule(notice: Pick<Notice, "source" | "code">): MessageModule {
    const source = notice.source;
    if (source === "system-can" || source.startsWith("can-")) return "CAN";
    if (source.startsWith("config")) return "Konfiguracja";
    if (source.startsWith("diag")) return "System";
    if (source.startsWith("drive-startup")) return "Napędy";
    if (source.startsWith("system")) return "System";
    if (["drive-enable", "axes", "axis-feedback"].includes(source) || source.startsWith("operation-") || source.startsWith("profile-")) return "Napędy";
    if (notice.code === "calibration" || notice.code === "configuration") return "Konfiguracja";
    if (source.startsWith("program")) return "Programowanie";
    return "Sterowanie";
}

export type ActiveMessage = CurrentMessage & {
    uid: string; module: MessageModule; fresh: boolean; acknowledged: boolean; acknowledgedAt: number | null; since: number;
};
export type MessageSnapshot = {
    entries: ConsoleEntry[]; active: ActiveMessage[]; statuses: Notice[];
    unacknowledged: number; worst: "error" | "warning" | "ok";
};

const signature = (notice: Pick<Notice, "severity" | "title" | "message" | "code">) =>
    JSON.stringify([notice.severity, notice.title ?? "", notice.message, notice.code ?? ""]);

export class MessageCenter {
    readonly control = new ControlMessages();
    private listeners = new Set<() => void>();
    private acks = new Map<string, { signature: string; at: number }>();
    private seen = new Map<string, { signature: string; at: number }>();
    private viewed = new Map<string, string>();
    private dismissed = new Map<string, string>();
    private snapshot: MessageSnapshot | null = null;
    private manualSessions = 0;

    constructor(private readonly clock: () => number = Date.now) {}

    subscribe = (listener: () => void) => { this.listeners.add(listener); return () => { this.listeners.delete(listener); }; };
    getSnapshot = (): MessageSnapshot => this.snapshot ??= this.build();

    private changed() { this.snapshot = null; for (const listener of [...this.listeners]) listener(); }

    report(notice: Notice) {
        // Pokwitowany wynik nie wraca, dopóki źródło powtarza tę samą treść.
        const raw = signature(notice);
        if (this.dismissed.get(notice.source) === raw) return;
        this.dismissed.delete(notice.source);
        if (this.control.report(notice)) this.changed();
    }
    condition(key: string, notice: Notice | null, resolution?: string) {
        if (this.control.condition(key, notice, resolution)) this.changed();
    }
    // Zamknięcie modułu: jego warunki zostają w bieżących jako nieobserwowane (stan z chwili zamknięcia).
    suspend(keys: Iterable<string>) {
        let changed = false; const at = this.clock();
        for (const key of keys) changed = this.control.suspend(key, at) || changed;
        if (changed) this.changed();
    }
    drop(keys: Iterable<string>) {
        let changed = false;
        for (const key of keys) changed = this.control.drop(key) || changed;
        if (changed) this.changed();
    }
    acknowledge(uid: string) { if (this.ack(uid)) this.changed(); }
    markViewed(uid: string) {
        const message = this.getSnapshot().active.find(item => item.uid === uid);
        if (!message || !message.fresh) return;
        this.viewed.set(uid, signature(message)); this.changed();
    }
    acknowledgeAll() {
        let changed = false;
        for (const message of this.getSnapshot().active) if (!message.acknowledged) changed = this.ack(message.uid) || changed;
        if (changed) this.changed();
    }
    private ack(uid: string): boolean {
        const message = this.getSnapshot().active.find(item => item.uid === uid);
        if (!message || message.acknowledged) return false;
        if (message.kind === "condition" && message.unobservedSince !== undefined) this.control.drop(message.source); // nikt go nie odświeży
        else if (message.kind === "result") {
            this.dismissed.set(message.source, signature(message));
            this.control.dismiss(message.source);
        } else this.acks.set(uid, { signature: signature(message), at: this.clock() });
        return true;
    }

    // Sterowanie ręczne zasila komunikaty robota z pełną częstotliwością; globalny odczyt w tym czasie milczy.
    attachManual(): () => void {
        this.manualSessions++;
        let attached = true;
        return () => { if (attached) { attached = false; this.manualSessions--; } };
    }
    get manualActive() { return this.manualSessions > 0; }

    private build(): MessageSnapshot {
        const now = this.clock(), live = new Set<string>();
        const active = this.control.active.map(message => {
            const uid = `${message.kind}/${message.source}`, current = signature(message);
            live.add(uid);
            let seen = this.seen.get(uid);
            if (!seen || seen.signature !== current) { seen = { signature: current, at: now }; this.seen.set(uid, seen); }
            const ack = this.acks.get(uid), acknowledged = !!ack && ack.signature === current;
            const fresh = !acknowledged && this.viewed.get(uid) !== current;
            return { ...message, uid, module: messageModule(message), fresh, acknowledged, acknowledgedAt: acknowledged ? ack!.at : null, since: seen.at };
        });
        for (const uid of [...this.seen.keys()]) if (!live.has(uid)) { this.seen.delete(uid); this.acks.delete(uid); this.viewed.delete(uid); }
        const rank = (message: ActiveMessage) => message.severity === "error" ? 0 : message.severity === "warning" ? 1 : 2;
        active.sort((a, b) => Number(a.acknowledged) - Number(b.acknowledged) || rank(a) - rank(b) || b.since - a.since);
        const worst = active.some(message => message.severity === "error") ? "error" : active.some(message => message.severity === "warning") ? "warning" : "ok";
        return { entries: [...this.control.log.entries], active, statuses: this.control.statuses,
            unacknowledged: active.filter(message => !message.acknowledged).length, worst };
    }
}

export const defaultMessageCenter = new MessageCenter();
export const MessageCenterContext = createContext<MessageCenter>(defaultMessageCenter);
export function useMessageCenter() { return useContext(MessageCenterContext); }
export function useMessages(): MessageSnapshot {
    const center = useMessageCenter();
    return useSyncExternalStore(center.subscribe, center.getSnapshot, center.getSnapshot);
}

// Stan trwały modułu (np. błąd zapisu, brak modelu) widoczny w pasku, dopóki moduł go zgłasza; po zamknięciu modułu
// zostaje jako nieobserwowany (stan z chwili zamknięcia) do powrotu modułu albo pokwitowania.
export function useCondition(key: string, notice: Notice | null) {
    const center = useMessageCenter(), value = notice ? JSON.stringify(notice) : "";
    useEffect(() => { center.condition(key, value ? JSON.parse(value) as Notice : null); }, [center, key, value]);
    useEffect(() => () => center.suspend([key]), [center, key]);
}
// Wynik operacji (np. odrzucone polecenie): czeka w pasku na pokwitowanie, informacje trafiają tylko do archiwum.
export function useReport() {
    const center = useMessageCenter();
    return useCallback((notice: Notice) => center.report(notice), [center]);
}

// Komunikat modułu poza Sterowaniem: własny tytuł, kontekst i link; kod „module” wyłącza dopasowanie opisów Sterowania.
export function moduleNotice(event: Notice, module: string, href: string): Notice {
    const text = event.message.trim();
    return { ...event, code: event.code ?? "module", context: event.context ?? module, title: event.title ?? (text.length > 90 ? `${text.slice(0, 87)}…` : text),
        remedy: event.remedy ?? (event.severity === "info" ? undefined : `Szczegóły i dalsze kroki w module ${module}.`), href: event.href ?? href };
}
