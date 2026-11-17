import { describe, expect, it } from "vitest";
import { jointBlocker, jointSpeed, sameIntent, StickSelector, tcpSpeed } from "./pendantJog";
import type { Axis } from "./manualTypes";

describe("drążek teach pendanta", () => {
    it("pierwszy kierunek wybiera oś pary; zmiana osi dopiero po powrocie do środka", () => {
        const s = new StickSelector();
        expect(s.update(.3, 0, "tcp", 0)).toBeNull();                                   // poniżej progu
        expect(s.update(.8, .2, "tcp", 0)).toEqual({ mode: "tcp", axis: "x", direction: 1 });
        expect(s.update(.1, .9, "tcp", 0)).toBeNull();                                  // oś X zablokowana, składowa X mała → postój
        expect(s.update(-.7, .5, "tcp", 0)).toEqual({ mode: "tcp", axis: "x", direction: -1 });
        expect(s.update(0, .1, "tcp", 0)).toBeNull();                                   // środek — blokada zwolniona
        expect(s.update(.1, -.9, "tcp", 1)).toEqual({ mode: "tcp", axis: "rz", direction: -1 });
        s.reset();
        expect(s.update(.9, 0, "joint", 2)).toEqual({ mode: "joint", axis: 5, direction: 1 });
        expect(sameIntent({ mode: "joint", axis: 5, direction: 1 }, { mode: "joint", axis: 5, direction: 1 })).toBe(true);
        expect(sameIntent({ mode: "joint", axis: 5, direction: 1 }, null)).toBe(false);
    });
    it("prędkości z profilu osi i limitu JOG TCP; blokady osi", () => {
        const axis = { available: true, joint_configured: true, calibration_valid: true, joint_position_known: true, enable_known: true, enabled: true,
            jog_min_velocity_degrees_s: .075, jog_max_velocity_degrees_s: 18 } as Axis;
        expect(jointSpeed(axis, .3)).toBeCloseTo(5.4); expect(jointSpeed(axis, .001)).toBe(.075); expect(jointSpeed(undefined, .3)).toBe(0);
        const status = { max_linear_velocity: .1, max_angular_velocity: Math.PI / 2 } as never;
        expect(tcpSpeed(status, "x", .3)).toBeCloseTo(30); expect(tcpSpeed(status, "rz", .1)).toBeCloseTo(9);
        expect(jointBlocker(axis, "rev")).toBeNull();
        expect(jointBlocker({ ...axis, enabled: false }, "rev")).toMatch(/ENABLE=0/);
        expect(jointBlocker({ ...axis, calibration_valid: false }, "rev")).toMatch(/kalibracja/);
        expect(jointBlocker(axis, undefined)).toMatch(/rewizji/);
    });
});
