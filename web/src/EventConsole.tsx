import { useCallback, useLayoutEffect, useRef, useState } from "react";
import { EventLog, type ConsoleEntry, type Notice } from "./eventLog";

export function useEventConsole() {
    const [log] = useState(() => new EventLog());
    const [entries, setEntries] = useState<ConsoleEntry[]>([]);
    const notice = useCallback((event: Notice) => {
        if (log.append(event)) setEntries([...log.entries]);
    }, [log]);
    return { entries, notice };
}

const labels = { info: "INFO", warning: "UWAGA", error: "AWARIA" };
export function EventConsole({ entries }: { entries: ConsoleEntry[] }) {
    const viewport = useRef<HTMLDivElement>(null);
    const follow = useRef(true);
    useLayoutEffect(() => {
        if (follow.current && viewport.current) viewport.current.scrollTop = viewport.current.scrollHeight;
    }, [entries]);
    return <div className="event-console" role="log" aria-label="Konsola zdarzeń" aria-live="polite" aria-relevant="additions" tabIndex={0}
        ref={viewport} onScroll={() => {
            const element = viewport.current!;
            follow.current = element.scrollHeight - element.clientHeight - element.scrollTop < 12;
        }}>
        {entries.map(entry => <div key={entry.id} className={`console-entry console-${entry.severity}`}>
            <time dateTime={new Date(entry.timestamp).toISOString()}>{new Date(entry.timestamp).toLocaleTimeString("pl-PL", { hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false, fractionalSecondDigits: 3 })}</time>
            <span className="console-level">{labels[entry.severity]}</span><span>{entry.message}</span>
        </div>)}
    </div>;
}
