export type Notice = { source: string; severity: "info" | "warning" | "error"; message: string };
export type ConsoleEntry = Notice & { id: number; timestamp: number };

// Historia wyłącznie w pamięci otwartego modułu. Powtarzane snapshoty nie są zdarzeniami.
export class EventLog {
    readonly entries: ConsoleEntry[] = [];
    private previous = new Map<string, string>();
    private sequence = 0;
    static readonly limit = 100;

    append(notice: Notice, timestamp = Date.now()): boolean {
        const message = notice.message.trim().slice(0, 800);
        if (!message) return false;
        const signature = `${notice.severity}/${message}`;
        if (this.previous.get(notice.source) === signature) return false;
        this.previous.delete(notice.source);
        this.previous.set(notice.source, signature);
        if (this.previous.size > 32) this.previous.delete(this.previous.keys().next().value!);
        this.entries.push({ ...notice, message, timestamp, id: ++this.sequence });
        if (this.entries.length > EventLog.limit) this.entries.shift();
        return true;
    }
}
