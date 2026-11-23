import { describe, expect, it } from "vitest";
import { MessageCenter, messageModule, moduleNotice } from "./messageCenter";
import { ActivityStore, deriveActivity, isRobotStateKey, recoveryNotice, robotConditions } from "./robotActivity";
import { backendNotice, systemConditions } from "./systemMessages";
import type { ManualState } from "./manualTypes";
import type { Snapshot } from "./status";

const axis = (n: number, extra: Partial<ManualState["axes"][number]> = {}): ManualState["axes"][number] => ({ axis: n, available: true, prepared: true, enable_known: true, enabled: true,
    position_known: true, position_degrees: n * 10, speed_known: true, speed_rpm: 0, can_jog: true, can_zero: true, motion: "postój", ...extra });
const manual = (extra: Partial<ManualState> = {}): ManualState => ({ fresh: true, busy: false, fault: false, active_axis: 0, max_rpm: 30, operation: "idle", message: "",
    axes: [1, 2, 3, 4, 5, 6].map(n => axis(n)), recovery_state: "idle", ...extra });

describe("MessageCenter — kwitowanie", () => {
    it("pokwitowany warunek zostaje na liście, a nowa treść znosi pokwitowanie", () => {
        let now = 1000; const center = new MessageCenter(() => now);
        center.condition("drive-enable", { source: "drive-enable", severity: "warning", code: "enable", message: "ENABLE=0 J1" });
        const [first] = center.getSnapshot().active;
        expect(first.acknowledged).toBe(false); expect(center.getSnapshot().unacknowledged).toBe(1);
        now = 2000; center.acknowledge(first.uid);
        expect(center.getSnapshot().active[0]).toMatchObject({ acknowledged: true, acknowledgedAt: 2000, since: 1000 });
        expect(center.getSnapshot().unacknowledged).toBe(0);
        center.condition("drive-enable", { source: "drive-enable", severity: "warning", code: "enable", message: "ENABLE=0 J1, J2" });
        expect(center.getSnapshot().active[0].acknowledged).toBe(false);
    });
    it("warunek zamkniętego modułu zostaje jako nieobserwowany, wraca z modułem, a pokwitowanie go usuwa", () => {
        let now = 1000; const center = new MessageCenter(() => now);
        center.condition("config-model", { source: "config-model", severity: "warning", code: "module", message: "Model 3D niedostępny" });
        now = 5000; center.suspend(["config-model", "nieznany"]);
        const [stale] = center.getSnapshot().active;
        expect(stale).toMatchObject({ source: "config-model", unobservedSince: 5000, acknowledged: false });
        const entries = center.getSnapshot().entries.length;
        center.condition("config-model", { source: "config-model", severity: "warning", code: "module", message: "Model 3D niedostępny" });
        expect(center.getSnapshot().active[0].unobservedSince).toBeUndefined();
        expect(center.getSnapshot().entries).toHaveLength(entries);
        center.suspend(["config-model"]); center.acknowledge(center.getSnapshot().active[0].uid);
        expect(center.getSnapshot().active).toHaveLength(0);
        expect(center.getSnapshot().entries).toHaveLength(entries);
    });
    it("obejrzenie zdejmuje NOWY bez kwitowania, nowa treść przywraca NOWY", () => {
        const center = new MessageCenter();
        center.condition("drive-enable", { source: "drive-enable", severity: "warning", code: "enable", message: "ENABLE=0 J1" });
        const [first] = center.getSnapshot().active;
        expect(first.fresh).toBe(true);
        center.markViewed(first.uid);
        expect(center.getSnapshot().active[0]).toMatchObject({ fresh: false, acknowledged: false });
        expect(center.getSnapshot().unacknowledged).toBe(1);
        center.condition("drive-enable", { source: "drive-enable", severity: "warning", code: "enable", message: "ENABLE=0 J1, J2" });
        expect(center.getSnapshot().active[0].fresh).toBe(true);
        center.acknowledge(first.uid);
        expect(center.getSnapshot().active[0].fresh).toBe(false);
    });
    it("pokwitowany wynik znika z bieżących, zostaje w historii i nie wraca przy tej samej treści", () => {
        const center = new MessageCenter();
        const result = { source: "motion-prepare", severity: "error" as const, message: "Cel poza limitami osi J1" };
        center.report(result);
        const uid = center.getSnapshot().active[0].uid;
        center.acknowledge(uid);
        expect(center.getSnapshot().active).toHaveLength(0);
        expect(center.getSnapshot().entries.some(entry => entry.message === result.message)).toBe(true);
        center.report(result);
        expect(center.getSnapshot().active).toHaveLength(0);
        center.report({ ...result, message: "Cel poza limitami osi J2" });
        expect(center.getSnapshot().active).toHaveLength(1);
    });
    it("Kwituj wszystkie: warunki zostają pokwitowane, wyniki znikają; niepokwitowane są pierwsze", () => {
        const center = new MessageCenter();
        center.condition("controller", { source: "controller", severity: "error", code: "fault", message: "Awaria J1" });
        center.condition("axes", { source: "axes", severity: "warning", code: "missing", message: "Brak J3" });
        center.report({ source: "motion-execute", severity: "warning", message: "Start niepotwierdzony" });
        center.acknowledge(center.getSnapshot().active.find(m => m.source === "controller")!.uid);
        expect(center.getSnapshot().active.map(m => m.source)).toEqual(["axes", "motion-execute", "controller"]);
        center.acknowledgeAll();
        const snapshot = center.getSnapshot();
        expect(snapshot.active.map(m => [m.source, m.acknowledged])).toEqual([["controller", true], ["axes", true]]);
        expect(snapshot.unacknowledged).toBe(0); expect(snapshot.worst).toBe("error");
    });
    it("zamknięcie źródła usuwa warunek bez wpisu „ustąpił”; migawka jest stała do zmiany", () => {
        const center = new MessageCenter();
        center.condition("freshness", { source: "freshness", severity: "warning", code: "freshness", message: "Dane nieaktualne" });
        const before = center.getSnapshot(), count = before.entries.length;
        expect(center.getSnapshot()).toBe(before);
        center.drop(["freshness", "nieznany"]);
        expect(center.getSnapshot().active).toHaveLength(0);
        expect(center.getSnapshot().entries).toHaveLength(count);
    });
    it("sesja Sterowania ręcznego wycisza globalny odczyt", () => {
        const center = new MessageCenter();
        const detach = center.attachManual(); expect(center.manualActive).toBe(true);
        detach(); detach(); expect(center.manualActive).toBe(false);
    });
    it("przypisuje komunikaty do modułów", () => {
        expect(messageModule({ source: "system-can" })).toBe("CAN");
        expect(messageModule({ source: "system-ros" })).toBe("System");
        expect(messageModule({ source: "drive-enable" })).toBe("Napędy");
        expect(messageModule({ source: "profile-1", code: "calibration" })).toBe("Napędy");
        expect(messageModule({ source: "planner", code: "configuration" })).toBe("Konfiguracja");
        expect(messageModule({ source: "controller", code: "fault" })).toBe("Sterowanie");
        expect(messageModule({ source: "configuration-connection" })).toBe("Konfiguracja");
        expect(messageModule({ source: "drive-startup-axes" })).toBe("Napędy");
        expect(messageModule({ source: "diag-trace" })).toBe("System");
        expect(messageModule({ source: "can-scan" })).toBe("CAN");
        expect(messageModule({ source: "program-message" })).toBe("Programowanie");
    });
    it("komunikat modułu ma własny tytuł i nie dostaje opisu Sterowania", () => {
        const center = new MessageCenter();
        center.report(moduleNotice({ source: "configuration-connection", severity: "warning", message: "Konfiguracja ROS niedostępna: HTTP 503" }, "Konfiguracja", "#config"));
        expect(center.getSnapshot().active[0]).toMatchObject({ title: "Konfiguracja ROS niedostępna: HTTP 503", context: "Konfiguracja", href: "#config",
            remedy: "Szczegóły i dalsze kroki w module Konfiguracja.", module: "Konfiguracja" });
        center.report({ source: "configuration-connection", severity: "info", message: "Połączenie z węzłem konfiguracji ROS działa." });
        expect(center.getSnapshot().active).toHaveLength(0);
    });
});

describe("czynność robota", () => {
    it("rozpoznaje brak danych, awarię, odzyskiwanie, ruch, operację i postój", () => {
        expect(deriveActivity(null, null).kind).toBe("unknown");
        expect(deriveActivity(manual({ fault: true, message: "J1: ruch bez komendy w postoju; zatrzymanie awaryjne" }), null)).toMatchObject({ kind: "fault", label: "ZATRZYMANY", detail: "J1: ruch bez komendy w postoju" });
        expect(deriveActivity(manual({ recovery_state: "recovery_failed", recovery_message: "J3: odrzucony F5" }), null)).toMatchObject({ kind: "recovery", label: "ODZYSKIWANIE NIEUDANE" });
        expect(deriveActivity(manual(), { busy: true, motion_type: "LIN", state: "running" } as never)).toMatchObject({ kind: "move", detail: "LIN · w ruchu" });
        expect(deriveActivity(manual({ operation: "jog", active_axis: 2, busy: true }), null)).toMatchObject({ kind: "move", detail: "JOG J2" });
        expect(deriveActivity(manual({ operation: "zero", active_axis: 4, busy: true }), null)).toMatchObject({ kind: "operation", detail: "Zerowanie J4" });
        const idle = deriveActivity(manual({ axes: [1, 2, 3, 4, 5, 6].map(n => axis(n, { enabled: n < 3 })) }), null);
        expect(idle).toMatchObject({ kind: "idle", label: "POSTÓJ", detail: "ENABLE 2/6" });
        expect(idle.axes.map(a => a.state)).toEqual(["waiting", "waiting", "disabled", "disabled", "disabled", "disabled"]);
    });
    it("stan osi: ruch, czekanie, wyłączona, pozycja przegubu lub wału", () => {
        const state = manual({ axes: [axis(1, { motion: "w prędkości", speed_rpm: 12 }), axis(2, { joint_configured: true, joint_position_known: true, joint_position_degrees: 1.5 }), ...[3, 4, 5, 6].map(n => axis(n))] });
        const [first, second] = deriveActivity(state, null).axes;
        expect(first).toMatchObject({ state: "moving", position: 10, joint: false, speedRpm: 12 });
        expect(second).toMatchObject({ state: "waiting", position: 1.5, joint: true });
    });
    it("postój po ruchu pamięta poprzednią czynność i zapisuje oś czasu", () => {
        let now = 0; const store = new ActivityStore(() => now);
        store.update(manual({ operation: "jog", active_axis: 2, busy: true }), null);
        now = 3000; store.update(manual(), null);
        expect(store.getSnapshot()).toMatchObject({ kind: "idle", detail: "po JOG J2 · ENABLE 6/6", since: 3000 });
        now = 4000; store.update(manual(), null);
        expect(store.getSnapshot().log.map(e => e.detail)).toEqual(["ENABLE 6/6", "JOG J2"]);
        expect(store.getSnapshot().since).toBe(3000);
    });
    it("komunikaty robota z globalnego odczytu mają klucze i treść jak w Sterowaniu", () => {
        const conditions = new Map(robotConditions(manual({ fault: true, message: "Awaria J1", recovery_state: "recovery_failed" })).map(([key, notice]) => [key, notice]));
        expect(conditions.get("controller")).toEqual({ source: "controller", severity: "error", code: "fault", context: "Robot fizyczny", message: "Awaria J1" });
        expect(conditions.get("recovery")).toMatchObject({ source: "recovery", severity: "error", title: "Odzyskiwanie nieudane" });
        expect(conditions.get("freshness")).toBeNull();
        expect([...conditions.keys()].every(isRobotStateKey)).toBe(true);
        const off = new Map(robotConditions(manual({ axes: [1, 2, 3, 4, 5, 6].map(n => axis(n, { enabled: n !== 2 })) })).map(([key, notice]) => [key, notice]));
        expect(off.get("drive-enable")).toMatchObject({ severity: "warning", message: "ENABLE wyłączony lub nieznany: J2." });
        // Nieświeże dane: tylko „freshness”, pozostałe warunki zachowują ostatnią ocenę.
        expect(robotConditions(manual({ fresh: false })).map(([key]) => key)).toEqual(["freshness"]);
        expect(robotConditions(null)[0][1]).toMatchObject({ source: "freshness", severity: "warning" });
        expect(recoveryNotice({ recovery_state: "idle_disabled", message: "" })).toBeNull();
        expect(recoveryNotice({ recovery_state: "verifying_disabled", message: "" })).toMatchObject({ severity: "warning", title: "Odzyskiwanie po STOP" });
    });
});

describe("komunikaty systemowe", () => {
    const component = (state: string, message = "") => ({ id: "x", state, message, sampled_at: null, values: {} });
    const snapshot = { components: { api: component("ok"), ros: component("error", "Brak węzła"), host: component("ok"), can: component("warn", "Błędy magistrali"), controllers: component("not_configured"), drives: component("ok") } } as unknown as Snapshot;
    it("zgłasza tylko uwagi i błędy aktualnych podzespołów", () => {
        const conditions = Object.fromEntries(systemConditions(snapshot, true));
        expect(conditions["system-ros"]).toMatchObject({ severity: "error", title: "ROS 2: błąd", message: "Brak węzła" });
        expect(conditions["system-can"]).toMatchObject({ severity: "warning", href: "#can" });
        expect(conditions["system-host"]).toBeNull(); expect(conditions["system-controllers"]).toBeNull();
        expect(systemConditions(snapshot, false).every(([, notice]) => notice === null)).toBe(true);
    });
    it("brak backendu dopiero po 5 s", () => {
        expect(backendNotice(4000)).toBeNull();
        expect(backendNotice(6000)).toMatchObject({ source: "system-backend", severity: "error" });
    });
});
