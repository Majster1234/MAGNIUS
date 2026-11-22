import { describe, expect, it } from "vitest";
import type { CanSnapshot } from "./canStatus";
import { barPercent, gigabytes, loadColor, loadGradient, navResources, percentText } from "./navStatus";
import type { Snapshot } from "./status";

const status = { components: { host: { values: {
    cpu_percent: 38.2, cpu_per_core_percent: [41, 28.5, "x", 47], memory_percent: 34.1,
    memory_used_bytes: 2.6 * 2 ** 30, memory_total_bytes: 7.6 * 2 ** 30 } } } } as unknown as Snapshot;
const can = { summary: { values: { scheduled_utilization_percent: 10.6, total_bus_planning_limit_percent: 60 } } } as unknown as CanSnapshot;

describe("navResources", () => {
    it("czyta CPU (także per wątek), RAM i obciążenie CAN ze świeżych danych", () => {
        const r = navResources(status, true, can, true);
        expect(r.cpu).toBe(38.2);
        expect(r.cores).toEqual([41, 28.5, 47]);
        expect(r.ram).toBe(34.1);
        expect(gigabytes(r.ramUsed)).toBe("2,6");
        expect(r.canLoad).toBe(10.6);
        expect(r.canLimit).toBe(60);
    });
    it("bez świeżych danych nie pokazuje starych wartości", () => {
        const r = navResources(status, false, can, false);
        expect(r).toEqual({ cpu: null, cores: [], ram: null, ramUsed: null, ramTotal: null, canLoad: null, canLimit: null });
        expect(percentText(r.cpu)).toBe("—");
    });
    it("starszy monitor bez listy wątków daje pustą listę", () => {
        const old = { components: { host: { values: { cpu_percent: 5 } } } } as unknown as Snapshot;
        expect(navResources(old, true, null, false).cores).toEqual([]);
    });
    it("ogranicza pasek do 0–100 %", () => {
        expect(barPercent(140)).toBe(100);
        expect(barPercent(-3)).toBe(0);
        expect(barPercent(null)).toBe(0);
    });
});

describe("gradient obciążenia", () => {
    it("zielony przy 0, bursztynowy przy 75 %, czerwony od 100 %", () => {
        expect(loadColor(0)).toBe("rgb(33, 208, 122)");
        expect(loadColor(.75)).toBe("rgb(255, 176, 32)");
        expect(loadColor(1)).toBe("rgb(255, 59, 78)");
        expect(loadColor(1.7)).toBe("rgb(255, 59, 78)");
        expect(loadColor(.25)).toBe("rgb(132, 208, 91)");
        expect(loadColor(null)).toBeUndefined();
    });
    it("skala paska CAN kończy czerwień na limicie pasma", () => {
        expect(loadGradient("90deg", 60)).toBe("linear-gradient(90deg, rgb(33, 208, 122) 0%, rgb(230, 208, 60) 30.0%, rgb(255, 176, 32) 45.0%, rgb(255, 59, 78) 60.0%)");
    });
});
