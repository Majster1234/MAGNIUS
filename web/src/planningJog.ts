import type { Axis, ManualState } from "./manualTypes";
import { MANUAL_STATE_TIMEOUT_MS } from "./manualControl";
import type { PreviewContext } from "./planningContext";
import { contextKey } from "./planningContext";

export type PlanningSnapshot = { positions: (number | null)[]; activeAxis: number; ready: boolean; available: boolean[] };

function limits(axis: Axis): [number, number] | null {
    if (!axis.joint_configured) return [-3600, 3600];
    const lower = axis.joint_lower_degrees, upper = axis.joint_upper_degrees;
    return typeof lower === "number" && typeof upper === "number" && Number.isFinite(lower) && Number.isFinite(upper) && lower < upper ? [lower, upper] : null;
}

// Wyłącznie lokalny stan: brak transportu oraz wywołań API sterowania.
export class PlanningJog {
    private positions = Array<number | null>(6).fill(null);
    private active: { axis: number; direction: number; speed: number; at: number } | null = null;
    private state: ManualState | null = null;
    private receivedAt = -Infinity;
    private connected = false;
    private session = false;
    private valid = false;
    private revision: string | undefined;
    private simulation: PreviewContext | null = null;
    private contextAt = -Infinity;

    configure(context: PreviewContext | null, now: number) {
        const old = this.simulation;
        if (contextKey(old) !== contextKey(context)) {
            this.stop();
            if (old && context && (old.model_id !== context.model_id || old.configuration_revision !== context.configuration_revision)) this.invalidate();
        }
        this.simulation = context; this.contextAt = now;
    }
    private simulationEnabled(now: number) { return this.connected && !!this.simulation?.ignore_missing_drivers && now - this.contextAt < 3000; }

    get ready() { return this.valid; }
    get activeAxis() { return this.active?.axis ?? 0; }
    snapshot(now: number): PlanningSnapshot {
        return { positions: [...this.positions], activeAxis: this.activeAxis, ready: this.valid,
            available: Array.from({ length: 6 }, (_, i) => this.position(i + 1, now) !== null) };
    }
    receive(state: ManualState, now: number) {
        this.state = state; this.receivedAt = now; this.connected = true;
        const nextRevision = this.simulation?.ignore_missing_drivers ? this.simulation.configuration_revision : state.configuration_revision;
        const invalidated = this.session && this.revision !== nextRevision && this.valid;
        if (invalidated) this.invalidate();
        if (this.session && this.valid) {
            for (let i = 0; i < 6; i++) if (this.positions[i] === null) this.positions[i] = this.position(i + 1, now);
        }
        if (this.active && this.position(this.active.axis, now) === null) this.stop();
        return invalidated;
    }
    disconnect() { this.connected = false; this.stop(); }
    begin(now: number) {
        const positions = Array.from({ length: 6 }, (_, i) => this.position(i + 1, now));
        if (!positions.some(position => position !== null)) return false;
        this.stop(); this.positions = positions; this.session = true; this.valid = true;
        this.revision = this.simulation?.ignore_missing_drivers ? this.simulation.configuration_revision : this.state?.configuration_revision;
        return true;
    }
    end() { this.session = false; this.invalidate(); }
    invalidate() { this.stop(); this.valid = false; this.positions.fill(null); }
    stop() { this.active = null; }
    preview(positions: number[]) {
        if (!this.session || !this.valid || positions.length !== 6 || !positions.every(Number.isFinite)) return false;
        this.stop(); this.positions = [...positions]; return true;
    }
    canStart(axis: number, direction: number, speed: number, now: number) {
        if (!this.valid || this.active || ![1, -1].includes(direction) || !Number.isFinite(speed) || speed <= 0 || this.position(axis, now) === null) return false;
        const profile = this.axis(axis), position = this.positions[axis - 1];
        if (!profile || position === null || position === undefined || this.maximumSpeed(profile) <= 0) return false;
        const range = limits(profile);
        return !!range && (direction > 0 ? position < range[1] : position > range[0]);
    }
    start(axis: number, direction: number, speed: number, now: number) {
        if (!this.canStart(axis, direction, speed, now)) return false;
        this.active = { axis, direction, speed, at: now };
        return true;
    }
    tick(now: number) {
        const active = this.active;
        if (!active) return false;
        const elapsed = now - active.at, profile = this.axis(active.axis);
        // Przerwa animacji nigdy nie powoduje nadrabiania ruchu.
        if (this.position(active.axis, now) === null || !profile || elapsed < 0 || elapsed > 100) { this.stop(); return true; }
        const range = limits(profile), maximum = this.maximumSpeed(profile);
        if (!range || maximum <= 0) { this.stop(); return true; }
        const speed = Math.min(active.speed, maximum) * (profile.joint_configured ? 1 : 6);
        const index = active.axis - 1, current = this.positions[index];
        if (current === null) { this.stop(); return true; }
        const next = current + active.direction * speed * elapsed / 1000;
        this.positions[index] = active.direction > 0 ? Math.min(range[1], next) : Math.max(range[0], next);
        active.at = now;
        if (this.positions[index] === range[active.direction > 0 ? 1 : 0]) this.stop();
        return true;
    }
    controlProfile(id: number) { return this.axis(id); }
    private axis(id: number) {
        const original = this.state?.axes.find(axis => axis.axis === id);
        const profile = this.simulation?.ignore_missing_drivers ? this.simulation.profiles[id - 1] : null;
        if (!profile) return original;
        return { ...original, axis: id, joint_configured: profile.configured, joint_lower_degrees: profile.lower,
            joint_upper_degrees: profile.upper, jog_max_velocity_degrees_s: profile.velocity } as Axis;
    }
    private maximumSpeed(axis: Axis) {
        const value = axis.joint_configured ? axis.jog_max_velocity_degrees_s : this.state?.max_rpm ?? 30;
        return typeof value === "number" && Number.isFinite(value) && value > 0 ? value : 0;
    }
    private position(id: number, now: number): number | null {
        if (this.simulationEnabled(now)) return this.simulation!.preview_seed_degrees[id - 1];
        const elapsed = now - this.receivedAt, axis = this.axis(id);
        if (!this.connected || !this.state?.fresh || !Number.isFinite(now) || elapsed < 0 || elapsed >= MANUAL_STATE_TIMEOUT_MS || !axis?.available || !axis.position_known) return null;
        const age = axis.position_age_seconds;
        if (typeof age !== "number" || !Number.isFinite(age) || age < 0 || age * 1000 + elapsed >= 250) return null;
        if (axis.joint_configured && (!axis.calibration_valid || !axis.joint_position_known || !limits(axis))) return null;
        const position = axis.joint_configured ? axis.joint_position_degrees : axis.position_degrees;
        return typeof position === "number" && Number.isFinite(position) ? position : null;
    }
}
