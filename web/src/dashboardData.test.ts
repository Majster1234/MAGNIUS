import { expect, it } from "vitest";
import { axisPresence, busStateLabel } from "./dashboardData";
import type { CanSnapshot } from "./canStatus";
import { pageFromHash } from "./AppShell";

it("mapuje trasy i zachowuje poprzedni adres konfiguracji", () => {
    for (const page of ["overview", "can", "manual", "config", "architecture"] as const) expect(pageFromHash(`#${page}`)).toBe(page);
    expect(pageFromHash("#configuration")).toBe("config");
    expect(pageFromHash("")).toBe("overview");
    expect(pageFromHash("#nieznana")).toBe("overview");
});
it("pokazuje wyłącznie potwierdzoną obecność CAN 1–6 i wygasza stare dane", () => {
    const snapshot: CanSnapshot = {schema_version:1, updated_at:null, interface:"can0", fresh:true, age_seconds:0,
        summary:{state:"ok", message:"", values:{}}, scan:{state:"ok", message:"", values:{}},
        devices:[1, 2, 5, 9].map(id => ({id, state:"ok", message:"Odpowiada", values:{online:true}}))};
    expect(axisPresence(snapshot, true, 1000, 1200)).toEqual([true, true, false, false, true, false]);
    snapshot.devices[0].values.online = false;
    expect(axisPresence(snapshot, true, 1000, 1200)[0]).toBe(false);
    expect(axisPresence(snapshot, true, 1000, 4000)).toEqual(Array(6).fill(null));
    expect(axisPresence(snapshot, false, 1000, 1200)).toEqual(Array(6).fill(null));
    expect(axisPresence(null, true, 1000, 1200)).toEqual(Array(6).fill(null));
});
it("odróżnia normalny stan ERROR-ACTIVE od alarmów magistrali", () => {
    expect(busStateLabel("ERROR-ACTIVE")).toBe("OK (ERROR-ACTIVE)");
    expect(busStateLabel("BUS-OFF")).toBe("BUS-OFF");
    expect(busStateLabel(undefined)).toBe("—");
});
