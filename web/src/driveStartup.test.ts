import { describe, expect, it } from "vitest";
import { applyErrors, axisList, differenceRows, formatDriveValue, type DriveStartupState } from "./driveStartup";

const entry = (expected_value: unknown, actual_value: unknown) => ({ expected: "00", actual: "01", expected_value, actual_value, writable: true });

describe("driveStartup", () => {
    it("układa różnice według numeru osi i podpisuje nastawy po polsku", () => {
        const rows = differenceRows({ "10": { user_id: entry(1, 2) }, "2": { work_current_ma: entry(1600, 2000), vfoc_kp_ki: entry([220, 100], [300, 120]) } });
        expect(rows.map(row => row.axis)).toEqual(["J2", "J2", "J10"]);
        expect(rows[0]).toMatchObject({ label: "Prąd roboczy [mA]", expected: "1600", actual: "2000" });
        expect(rows[1]).toMatchObject({ expected: "220, 100", actual: "300, 120" });
    });
    it("nazywa brak odczytu i zostawia nieznaną nastawę bez zmian", () => {
        expect(formatDriveValue(null)).toBe("brak odczytu");
        expect(differenceRows({ "1": { nowa: entry(0, null) } })[0]).toMatchObject({ label: "nowa", expected: "0", actual: "brak odczytu" });
    });
    it("sortuje listę osi", () => { expect(axisList(["5", 2, "1"])).toBe("J1, J2, J5"); });
    it("zbiera błędy zapisu z całej operacji i osi", () => {
        const state = { last_apply: { error: "Kopia nieudana", requires_power_cycle: false,
            axes: { "1": { written: ["microstep"], error: null }, "4": { written: [], error: "wymaga ENABLE=0" } } } } as unknown as DriveStartupState;
        expect(applyErrors(state)).toEqual(["Kopia nieudana", "J4: wymaga ENABLE=0"]);
        expect(applyErrors({ last_apply: null } as DriveStartupState)).toEqual([]);
    });
});
