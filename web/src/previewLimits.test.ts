import { expect, it } from "vitest";
import { previewLimits } from "./previewLimits";

const profile = { joint_configured: true, calibration_valid: true, joint_lower_degrees: -90, joint_upper_degrees: 90 };
it.each([
    [15.01, null], [15, "warning"], [5.01, "warning"], [5, "danger"],
    [1, "danger"], [0, "danger"], [-.01, "exceeded"],
])("rozróżnia progi obu limitów przy odległości %s°", (distance, level) => {
    for (const side of ["lower", "upper"] as const) {
        const position = side === "lower" ? -90 + distance : 90 - distance;
        const indicators = previewLimits(profile, position, true);
        if (level === null) expect(indicators).toEqual([]);
        else {
            expect(indicators).toHaveLength(1);
            expect(indicators[0]).toMatchObject({ side, level, critical: distance <= 1, fresh: true });
            expect(indicators[0].distanceDegrees).toBeCloseTo(distance);
        }
    }
});
it("korzysta z limitów przegubu, zachowuje zero i wielokrotne obroty", () => {
    expect(previewLimits({ ...profile, joint_upper_degrees: 0 }, -10, true)[0]).toMatchObject({ side: "upper", level: "warning", limitDegrees: 0 });
    expect(previewLimits({}, 3601, true)[0]).toMatchObject({ side: "upper", level: "exceeded", distanceDegrees: -1, limitDegrees: 3600 });
    expect(previewLimits({}, -7200, true)[0]).toMatchObject({ side: "lower", level: "exceeded", distanceDegrees: -3600 });
    expect(previewLimits({ ...profile, joint_lower_degrees: -720, joint_upper_degrees: 720 }, 350, true)).toEqual([]);
});
it("nie wymyśla danych i zachowuje ostrzeżenie ostatniego pomiaru", () => {
    expect(previewLimits(undefined, 90, true)).toEqual([]);
    expect(previewLimits(profile, null, true)).toEqual([]);
    expect(previewLimits(profile, NaN, true)).toEqual([]);
    expect(previewLimits({ ...profile, joint_upper_degrees: undefined }, 90, true)).toEqual([]);
    expect(previewLimits({ ...profile, joint_lower_degrees: 90 }, 90, true)).toEqual([]);
    expect(previewLimits({ ...profile, calibration_valid: false }, 90, true)).toEqual([]);
    expect(previewLimits(profile, 91, false)[0]).toMatchObject({ level: "exceeded", fresh: false });
});
it("pokazuje niezależnie obie granice w wąskim zakresie", () => {
    expect(previewLimits({ ...profile, joint_lower_degrees: -4, joint_upper_degrees: 4 }, 0, true).map(i => [i.side, i.level]))
        .toEqual([["lower", "danger"], ["upper", "danger"]]);
});
