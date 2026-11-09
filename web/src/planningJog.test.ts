import { expect, it } from "vitest";
import { PlanningJog } from "./planningJog";
import type { Axis, ManualState } from "./manualTypes";
import type { PreviewContext } from "./planningContext";

function axis(overrides: Partial<Axis> = {}): Axis {
    return { axis: 1, available: true, prepared: false, enable_known: false, enabled: false, position_known: true,
        position_degrees: 10, position_age_seconds: 0, speed_known: false, speed_rpm: 0, can_jog: false, can_zero: false, motion: "zatrzymany", ...overrides };
}
function state(first: Partial<Axis> = {}): ManualState {
    return { fresh: true, busy: false, fault: false, active_axis: 0, max_rpm: 30, operation: "idle", message: "", configuration_revision: "a",
        axes: Array.from({ length: 6 }, (_, i) => axis({ axis: i + 1, ...(i === 0 ? first : {}) })) };
}
function begin(input = state()) { const plan = new PlanningJog(); plan.receive(input, 0); expect(plan.begin(0)).toBe(true); return plan; }

it("kopiuje enkodery, porusza lokalnie i nie wymaga ENABLE", () => {
    const input = state(), plan = begin(input);
    expect(plan.snapshot(0).positions[0]).toBe(10);
    expect(plan.start(1, 1, 30, 0)).toBe(true); plan.tick(100);
    expect(plan.snapshot(100).positions[0]).toBe(28); expect(plan.activeAxis).toBe(1);
    expect(plan.start(2, 1, 30, 100)).toBe(false);
    input.axes[0].position_degrees = 999; plan.receive(input, 100); expect(plan.snapshot(100).positions[0]).toBe(28);
});

it("zatrzymuje się na limicie, blokuje zanik osi i synchronizuje później dostępną oś", () => {
    const input = state({ position_degrees: 3599 }); input.axes[1].available = false;
    const plan = begin(input); plan.start(1, 1, 30, 0); plan.tick(100);
    expect(plan.snapshot(100).positions[0]).toBe(3600); expect(plan.activeAxis).toBe(0);
    expect(plan.start(1, 1, 30, 100)).toBe(false);
    input.axes[1] = axis({axis: 2, position_degrees: 20}); plan.receive(input, 100); expect(plan.snapshot(100).positions[1]).toBe(20);
    plan.start(2, -1, 30, 100); input.axes[1].available = false; plan.receive(input, 120); expect(plan.activeAxis).toBe(0);
    input.axes[1].available = true; plan.receive(input, 140); expect(plan.activeAxis).toBe(0);
    expect(plan.start(2, -1, 30, 140)).toBe(true) /* po powrocie napędu ruch planu wymaga nowego wciśnięcia */;
});

it("używa limitu przegubu i unieważnia plan", () => {
    const plan = begin(state({joint_configured: true, calibration_valid: true, joint_position_known: true, joint_position_degrees: 89, joint_lower_degrees: -90, joint_upper_degrees: 90, jog_max_velocity_degrees_s: 5}));
    plan.start(1, 1, 500, 0); plan.tick(100); plan.tick(200);
    expect(plan.snapshot(200).positions[0]).toBe(90); plan.invalidate(); expect(plan.ready).toBe(false); expect(plan.snapshot(200).positions[0]).toBeNull();
});

it("kopiowanie wymaga świeżego enkodera, ale ruch planu nie zależy od wieku późniejszych próbek", () => {
    // Regresja 07.11.2026: przerwa w stanie (~150 ms) + wiek próbki (~70 ms) > 250 ms urywała ruch bliźniaka.
    const input = state({ position_age_seconds: .2 }), plan = begin(input);
    expect(plan.start(1, 1, 10, 0)).toBe(true); plan.tick(49); plan.tick(50); plan.tick(99);
    expect(plan.activeAxis).toBe(1); expect(plan.snapshot(99).available[0]).toBe(true);
    input.axes[0].position_age_seconds = null; plan.receive(input, 120); plan.tick(140);
    expect(plan.activeAxis).toBe(1); expect(plan.snapshot(600).available[0]).toBe(true);
    const stale = state(); stale.axes.forEach(a => { a.position_age_seconds = .26; }); const late = new PlanningJog(); late.receive(stale, 0);
    expect(late.begin(0)).toBe(false) /* nieświeży enkoder: brak kopii pozycji */;
});

it("powrót spoza limitu jest stopniowy i dostępny tylko do wewnątrz", () => {
    const plan = begin(state({ position_degrees: 3700 }));
    expect(plan.start(1, 1, 10, 0)).toBe(false); expect(plan.start(1, -1, 10, 0)).toBe(true);
    plan.tick(100); expect(plan.snapshot(100).positions[0]).toBe(3694);
});

it("luka animacji jest pomijana bez nadrabiania, odłączenie zatrzymuje bez wznowienia", () => {
    const input = state(), plan = begin(input); plan.start(1, 1, 10, 0); plan.tick(101);
    expect(plan.snapshot(101).positions[0]).toBe(10); expect(plan.activeAxis).toBe(1);
    plan.tick(151); expect(plan.snapshot(151).positions[0]).toBeCloseTo(13) /* 50 ms × 10 obr./min wału = 3° */; plan.stop();
    plan.receive(input, 110); plan.start(1, 1, 10, 110); plan.disconnect(); plan.receive(input, 120);
    expect(plan.activeAxis).toBe(0); expect(plan.snapshot(120).positions[0]).toBeCloseTo(13);
});

it("rewizja wymaga jawnego nowego początku, a zakończenie usuwa pozycje", () => {
    const input = state(), plan = begin(input); input.configuration_revision = "b"; input.axes[0].position_degrees = 45;
    expect(plan.receive(input, 100)).toBe(true); plan.receive(input, 120); expect(plan.ready).toBe(false);
    expect(plan.start(1, 1, 10, 120)).toBe(false); expect(plan.begin(120)).toBe(true);
    expect(plan.snapshot(120).positions[0]).toBe(45); plan.end(); expect(plan.snapshot(120).positions).toEqual(Array(6).fill(null));
});

it("odrzuca niepoprawny profil przegubu i nieważną kalibrację", () => {
    const input = state({ joint_configured: true, calibration_valid: false, joint_position_known: true, joint_position_degrees: 45 });
    const plan = begin(input); expect(plan.snapshot(0).positions[0]).toBeNull(); expect(plan.start(1, 1, 10, 0)).toBe(false);
});

it("ignorowanie napędów odblokowuje sześć osi z profilem, bez zmiany stanu sprzętu", () => {
    const input = state(); input.fresh = false;
    input.axes.forEach(axis => { axis.available = false; axis.position_known = false; });
    const context: PreviewContext = { model_id: "m", configuration_revision: "a", settings_revision: "s", planner_available: true,
        ignore_missing_drivers: true, available: Array(6).fill(false), preview_seed_degrees: Array(6).fill(0), seed_sources: Array(6).fill("zero"),
        profiles: Array.from({ length: 6 }, () => ({ configured: true, lower: -10, upper: 10, velocity: 2 })) };
    const plan = new PlanningJog(); plan.receive(input, 0); plan.configure(context, 0);
    expect(plan.begin(0)).toBe(true); expect(plan.snapshot(0).available).toEqual(Array(6).fill(true));
    expect(plan.start(6, 1, 100, 0)).toBe(true); plan.tick(100);
    expect(plan.snapshot(100).positions[5]).toBeCloseTo(.2);
    expect(input.axes[5].available).toBe(false); expect(input.axes[5].can_jog).toBe(false);
    plan.configure({ ...context, ignore_missing_drivers: false, settings_revision: "next" }, 100);
    expect(plan.activeAxis).toBe(0); expect(plan.snapshot(100).positions[5]).toBeCloseTo(.2);
    expect(plan.start(6, 1, 1, 100)).toBe(false);
});

it("przejściowy brak kontekstu planowania nie przerywa ruchu bliźniaka", () => {
    const input = state(), plan = begin(input);
    const context: PreviewContext = { model_id: "m", configuration_revision: "a", settings_revision: "s", planner_available: true,
        ignore_missing_drivers: false, available: Array(6).fill(true), preview_seed_degrees: Array(6).fill(0), seed_sources: Array(6).fill("measured"),
        profiles: Array.from({ length: 6 }, () => ({ configured: false, lower: -10, upper: 10, velocity: 2 })) };
    plan.configure(context, 0); expect(plan.start(1, 1, 10, 0)).toBe(true);
    plan.configure(null, 50); plan.tick(50); expect(plan.activeAxis).toBe(1);
    plan.configure(context, 100); plan.tick(100); expect(plan.activeAxis).toBe(1);
    plan.configure({ ...context, settings_revision: "inna" }, 120); expect(plan.activeAxis).toBe(0);
});
