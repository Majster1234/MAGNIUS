import { expect, it } from "vitest";
import { EventLog } from "./eventLog";

it("grupuje szybkie zmiany świeżości, zachowując ostatni stan i natychmiastowe awarie", () => {
    const log = new EventLog();
    const fresh = {source:"freshness", severity:"info", message:"Aktualne"} as const;
    const stale = {...fresh, severity:"warning", message:"Nieaktualne"} as const;
    log.append(fresh, 1000); log.append(stale, 1300); log.append(fresh, 1600);
    expect(log.entries).toHaveLength(1);
    expect(log.entries[0]).toMatchObject({message:"Aktualne", repetitions:3, timestamp:1600});
    log.append(stale, 2100);
    expect(log.entries).toHaveLength(2);
    log.append({...stale, severity:"error", message:"Awaria"}, 2110);
    expect(log.entries).toHaveLength(3);
    log.append(fresh, 2120);
    expect(log.entries).toHaveLength(4);
    expect(log.entries[2].severity).toBe("error");
});

it("zapisuje zmiany stanu, awarię i powrót zamiast powtarzanych snapshotów", () => {
    const log = new EventLog();
    const online = { source: "connection", severity: "info", message: "Połączenie działa" } as const;
    expect(log.append(online, 1000)).toBe(true);
    log.append({ source: "model", severity: "info", message: "Gotowy" }, 1001);
    for (let i = 0; i < 200; i++) expect(log.append(online, 1100 + i)).toBe(false);
    expect(log.append({ ...online, severity: "warning" }, 1400)).toBe(true);
    expect(log.append({ ...online, severity: "error", message: "Brak połączenia" }, 1500)).toBe(true);
    expect(log.append(online, 1600)).toBe(true);
    expect(log.entries.map(entry => entry.timestamp)).toEqual([1000, 1001, 1400, 1500, 1600]);
    expect(log.entries.map(entry => entry.severity)).toEqual(["info", "info", "warning", "error", "info"]);
});

it("ogranicza pamięć do 100 wpisów i 800 znaków na komunikat", () => {
    const log = new EventLog();
    expect(log.append({ source: "test", severity: "info", message: "  " })).toBe(false);
    for (let i = 0; i < 1000; i++) log.append({ source: "test", severity: "info", message: `${i}: ${"x".repeat(1000)}` }, i);
    expect(log.entries).toHaveLength(100);
    expect(log.entries[0].timestamp).toBe(900);
    expect(log.entries[99].timestamp).toBe(999);
    expect(log.entries.every(entry => entry.message.length <= 800)).toBe(true);
});
