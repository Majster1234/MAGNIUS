import { describe, expect, it } from "vitest";
import { describe as text, jointRanges, limitProblems, poseLimits, usedPositions, violations } from "./programLimits";
import { emptyProgram } from "./programModel";

const values = { "joint.1.lower": -170, "joint.1.upper": 170, "joint.2.lower": -200, "joint.2.upper": 30, "joint.3.lower": null, "joint.3.upper": 50 };

describe("zakresy osi w programie", () => {
    it("zakresy z konfiguracji (null = brak danych) i naruszenia", () => {
        const ranges = jointRanges(values);
        expect(ranges.slice(0, 3)).toEqual([[-170, 170], [-200, 30], null]);
        expect(violations([0, 35, 999, 0, 0, 0], ranges)).toEqual([{ axis: 2, value: 35, lower: -200, upper: 30 }]);
        expect(violations([0, 30, 0, 0, 0, 0], ranges)).toEqual([]);
        expect(text(violations([0, 35, 0, 0, 0, 0], ranges))).toBe("J2 = 35° (zakres -200…30°)");
    });
    it("wskaźniki limitów modelu jak w sterowaniu ręcznym", () => {
        const limits = poseLimits(jointRanges(values), [0, 32, 0, 0, 0, 0]);
        expect(limits[1]).toEqual([expect.objectContaining({ side: "upper", level: "exceeded", limitDegrees: 30 })]);
        expect(limits[0]).toEqual([]); expect(poseLimits(jointRanges(values), null).every(l => !l.length)).toBe(true);
    });
    it("uwagi tylko dla pozycji używanych przez włączone bloki lub zmienne POZYCJA", () => {
        const p = emptyProgram("T");
        p.positions = [{ name: "A", joints_deg: [0, 35, 0, 0, 0, 0] }, { name: "B", joints_deg: [0, 40, 0, 0, 0, 0] }, { name: "C", joints_deg: [0, 50, 0, 0, 0, 0] }];
        p.variables = [{ name: "cel", type: "POZYCJA", init: "C" }];
        p.blocks = [{ id: "m", command: "move_ptp", params: { position: { position: "A" } } }, { id: "off", command: "move_ptp", disabled: true, params: { position: { position: "B" } } }];
        expect([...usedPositions(p)]).toEqual([["A", "m"], ["C", null]]);
        expect(limitProblems(p, jointRanges(values)).map(x => [x.block, x.text.split(":")[0]])).toEqual([["m", "Pozycja A poza zakresem osi"], [undefined, "Pozycja C poza zakresem osi"]]);
    });
});
