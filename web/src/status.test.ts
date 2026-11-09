import { describe, expect, it } from "vitest";
import { dataIsStale, isSnapshot, type Snapshot } from "./status";

const snapshot = { fresh: true, age_seconds: 0.25 } as Snapshot;
describe("świeżość danych", () => {
    it("uwzględnia wiek danych po stronie serwera oraz czas bez wiadomości", () => {
        expect(dataIsStale(snapshot, 1000, 3749)).toBe(false);
        expect(dataIsStale(snapshot, 1000, 3750)).toBe(true);
    });
    it("odrzuca brak danych oraz raport o nieaktualnym monitorze", () => {
        expect(dataIsStale(null, 0, 0)).toBe(true);
        expect(dataIsStale({ ...snapshot, fresh: false }, 1000, 1000)).toBe(
            true,
        );
    });
    it("nie przyjmuje niepełnych i niezgodnych wiadomości", () => {
        expect(isSnapshot({})).toBe(false);
        expect(isSnapshot(null)).toBe(false);
        expect(
            isSnapshot({
                schema_version: 1,
                components: {},
                network: {},
                fresh: true,
                age_seconds: 0,
                updated_at: null,
            }),
        ).toBe(false);
    });
});
