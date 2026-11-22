import { useCallback, useEffect, useLayoutEffect, useRef } from "react";
import type { ConsoleEntry, Notice } from "./eventLog";
import { useMessageCenter, useMessages } from "./messageCenter";
import { isRobotStateKey } from "./robotActivity";

// Sterowanie ręczne zgłasza komunikaty do wspólnego magazynu (pasek nad stroną). Po zamknięciu modułu warunki stanu
// robota podtrzymuje globalny odczyt paska (te same klucze), pozostałe zostają jako nieobserwowane (stan z chwili
// zamknięcia) do powrotu modułu albo pokwitowania; wyniki operacji czekają na pokwitowanie.
export function useControlMessages() {
    const center = useMessageCenter();
    const snapshot = useMessages();
    const keys = useRef(new Set<string>()), closing = useRef(false);
    useEffect(() => {
        const detach = center.attachManual();
        return () => detach();
    }, [center]);
    // Zamknięcie strony: cleanup efektu układu rodzica wykonuje się przed cleanupami useEffect dzieci, więc warunki
    // zawieszamy tutaj, a późniejsze „wyczyść przy odmontowaniu” z komponentów (TCP, parametry symulacji) ignorujemy —
    // komunikat zostaje jako nieobserwowany. Zmiana trybu w obrębie strony nadal normalnie rozwiązuje warunek.
    useLayoutEffect(() => {
        closing.current = false;
        const owned = keys.current;
        return () => { closing.current = true; center.suspend([...owned].filter(key => !isRobotStateKey(key))); };
    }, [center]);
    const notice = useCallback((value: Notice) => center.report(value), [center]);
    const condition = useCallback((key: string, value: Notice | null, resolution?: string) => {
        if (closing.current) return;
        keys.current.add(key); center.condition(key, value, resolution);
    }, [center]);
    return { entries: snapshot.entries, active: snapshot.active, statuses: snapshot.statuses, notice, condition };
}

export function Explanation({ notice, expanded = false }: { notice: Notice; expanded?: boolean }) {
    const showReason = notice.description && notice.message.trim() && !notice.description.includes(notice.message);
    return <>{showReason && <p className="message-reason"><strong>Powód:</strong> {notice.message}</p>}<p>{notice.description ?? notice.message}</p>{notice.remedy && <p className="message-remedy"><strong>Co zrobić:</strong> {notice.remedy}</p>}
        {notice.href && <a className="message-link" href={notice.href}>Otwórz {notice.href === "#config" ? "konfigurację" : notice.href === "#can" ? "magistralę CAN" : "diagnostykę"} →</a>}
        {notice.details && (expanded ? <div className="message-details"><strong>Szczegóły techniczne</strong><pre>{notice.details}</pre></div> :
            <details><summary>Szczegóły techniczne</summary><pre>{notice.details}</pre></details>)}</>;
}

// Ogłoszenia niezależne od otwarcia panelu: czytnik ekranu słyszy awarie i ostatni komunikat.
export function ControlAnnouncements({ entries, active }: { entries: ConsoleEntry[]; active: (Notice & { acknowledged?: boolean })[] }) {
    return <><span className="sr-only control-announcement" role="alert" aria-live="assertive">{active.filter(n => n.severity === "error" && !n.acknowledged).map(n => `${n.context}: ${n.title}`).join(". ")}</span>
        <span className="sr-only control-announcement" role="status" aria-live="polite">{entries.at(-1)?.severity !== "error" ? entries.at(-1)?.title ?? entries.at(-1)?.message : ""}</span></>;
}
