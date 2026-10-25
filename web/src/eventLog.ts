export type Notice = { source: string; severity: "info" | "warning" | "error"; message: string;
    code?: string; title?: string; description?: string; remedy?: string; details?: string; context?: string; href?: string; progress?:number };
export type ConsoleEntry = Notice & { id: number; timestamp: number; repetitions?: number };

// Historia wyłącznie w pamięci otwartego modułu. Powtarzane snapshoty nie są zdarzeniami.
export class EventLog {
    readonly entries: ConsoleEntry[] = [];
    private previous = new Map<string, string>();
    private sequence = 0;
    static readonly limit = 100;

    append(notice: Notice, timestamp = Date.now()): boolean {
        const message = notice.message.trim().slice(0, 800);
        if (!message) return false;
        const signature = JSON.stringify([notice.severity, message, notice.code, notice.title, notice.description, notice.remedy, notice.context]);
        if (this.previous.get(notice.source) === signature) return false;
        this.previous.delete(notice.source);
        this.previous.set(notice.source, signature);
        if (this.previous.size > 64) this.previous.delete(this.previous.keys().next().value!);
        const last = this.entries.at(-1);
        // Wyłącznie prezentacja wahań świeżości; awarie i blokady sterowania nie są opóźniane.
        if (notice.source === "freshness" && notice.severity !== "error" && last?.source === "freshness" &&
            last.severity !== "error" && timestamp >= last.timestamp && timestamp - last.timestamp < 500) {
            this.entries[this.entries.length - 1] = { ...notice, message, timestamp, id: last.id, repetitions: (last.repetitions ?? 1) + 1 };
            return true;
        }
        this.entries.push({ ...notice, message, timestamp, id: ++this.sequence });
        if (this.entries.length > EventLog.limit) this.entries.shift();
        return true;
    }
}
