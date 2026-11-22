import { describe, expect, it } from "vitest";
import { matchSnapshot, type FaultListItem } from "./faultSnapshot";

const at = (ms: number, detail: string, id = String(ms)): FaultListItem => ({ id, time: "", utc_ns: ms * 1e6, duration_s: 15, rows: 1, axis: 1, triggers: [{ event: "bus_fault", detail }] });
const fault = "J1 (CAN ID 1): ruch bez komendy w postoju — odchyłka 0.652° przegubu (742 imp. wału) od pozycji trzymania, limit ±0.500°; zatrzymanie awaryjne";

describe("dopasowanie komunikatu do migawki awarii", () => {
    it("treść komunikatu wskazuje migawkę także dla starszej awarii", () => {
        const snapshots = [at(1000, fault, "a"), at(5000, "przekroczono termin etapu verifying_disabled", "b"), at(900_000, "J4 (CAN ID 4): ucieczka osi", "c")];
        expect(matchSnapshot({ source: "controller", severity: "error", message: fault }, 3_600_000, snapshots)?.id).toBe("a");
        expect(matchSnapshot({ source: "recovery", severity: "error", message: "przekroczono termin etapu verifying_disabled · oczekujące osie: J1" }, 6000, snapshots)?.id).toBe("b");
    });
    it("awaria robota bez wspólnej treści: najbliższa w czasie (±20 s / +10 s)", () => {
        const snapshots = [at(10_000, "inny opis", "a"), at(31_000, "inny opis", "b")];
        expect(matchSnapshot({ source: "controller", severity: "error", message: "Brak opisu awarii" }, 30_000, snapshots)?.id).toBe("b");
        expect(matchSnapshot({ source: "controller", severity: "error", message: "Brak opisu awarii" }, 60_000, snapshots)).toBeNull();
    });
    it("komunikaty spoza awarii robota i uwagi nie dostają cudzej migawki", () => {
        const snapshots = [at(1000, fault)];
        expect(matchSnapshot({ source: "program-message", severity: "error", message: "Nie zapisano programu" }, 1500, snapshots)).toBeNull();
        expect(matchSnapshot({ source: "controller", severity: "warning", message: fault }, 1500, snapshots)).toBeNull();
    });
});
