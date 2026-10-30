import { expect, it } from "vitest";
import type { CanSnapshot } from "./canStatus";
import type { Snapshot } from "./status";
import { architectureMeters, architectureRows, isProblem, type Overview } from "./architectureData";
import { driveCells, nextColumn, pulseSections, PULSE_COLUMNS, worst } from "./systemPulse";

const component = (id: string, state = "ok", values: Record<string, unknown> = {}) =>
    ({ id, state, message: `Komunikat ${id}`, sampled_at: "2026-10-29T12:00:00Z", values }) as Snapshot["components"]["api"];
function status(overrides: Partial<Record<keyof Snapshot["components"], ReturnType<typeof component>>> = {}): Snapshot {
    return { schema_version: 1, updated_at: "2026-10-29T12:00:00Z", age_seconds: 0.4, fresh: true,
        network: { address: "robot.test", port: 8080, interface: "eth0", subnet: "LAN", url: "http://robot.test:8080" },
        components: { api: component("api"), ros: component("ros"), host: component("host", "ok", { cpu_percent: 23, memory_percent: 33 }),
            can: component("can", "ok", { interface: "can0", bus_state: "ERROR-ACTIVE", bitrate: 1000000 }),
            controllers: component("controllers", "not_configured"), drives: component("drives", "not_configured"), ...overrides } };
}
function can(devices = [1, 2, 3, 4, 5, 6], summary: Partial<CanSnapshot["summary"]> = {}): CanSnapshot {
    return { schema_version: 1, updated_at: "2026-10-29T12:00:00Z", age_seconds: 0, fresh: true, interface: "can0",
        summary: { state: "ok", message: "CAN działa", values: { scheduled_utilization_percent: 31, total_bus_planning_limit_percent: 60, rtt_max_ms: 19.1, response_timeout_ms: 24 }, ...summary },
        scan: { state: "ok", message: "", values: {} },
        devices: devices.map(id => ({ id, state: "ok", message: "Odpowiada", values: { online: true, enable: false, reported_speed_rpm: 0 } })) };
}
const input = (over: Partial<Parameters<typeof pulseSections>[0]> = {}) => ({ status: status(), statusOnline: true, statusFresh: true,
    statusConnecting: false, can: can(), canOnline: true, canFresh: true, build: "abc1234", ...over });
const states = (over: Partial<Parameters<typeof pulseSections>[0]> = {}) => pulseSections(input(over)).map(item => `${item.id}:${item.state}`);

it("wszystko świeże → cztery sekcje OK; brak integracji kontrolerów nie jest awarią", () => {
    expect(states()).toEqual(["SYS:ok", "ROS:ok", "API:ok", "WEB:ok"]);
    expect(pulseSections(input())[0].lines[0]).toBe("CPU 23% · RAM 33%");
});
it("awaria interfejsu CAN trafia do SYS z komunikatem w dymku", () => {
    const sections = pulseSections(input({ status: status({ can: { ...component("can", "error", { interface: "can0" }), message: "Magistrala CAN: BUS-OFF" } }) }));
    expect(sections[0].state).toBe("error");
    expect(sections[0].lines).toEqual(["Interfejs can0: Magistrala CAN: BUS-OFF"]);
});
it("nieświeża diagnostyka CAN ostrzega w ROS, a utrata monitora wygasza SYS i ROS", () => {
    expect(states({ canFresh: false, canOnline: false })).toEqual(["SYS:ok", "ROS:warn", "API:ok", "WEB:warn"]);
    expect(states({ statusFresh: false })).toEqual(["SYS:stale", "ROS:stale", "API:ok", "WEB:ok"]);
});
it("odzyskiwanie po awarii pokazuje komunikat procedury", () => {
    const sections = pulseSections(input({ can: can(undefined, { state: "error", message: "Awaria SYNC", values: { fault: true, recovery_message: "Czeka na operatora" } }) }));
    expect(sections[1].state).toBe("error");
    expect(sections[1].lines).toContain("Odzyskiwanie: Czeka na operatora");
});
it("backend niedostępny: API awaria, przy łączeniu brak danych", () => {
    expect(states({ statusOnline: false, statusFresh: false, canOnline: false })[2]).toBe("API:error");
    expect(states({ statusOnline: false, statusFresh: false, statusConnecting: true })[2]).toBe("API:stale");
});
it("napędy: brak wykrycia i brak odpowiedzi rozróżnione, nieświeże dane wygaszają wszystkie", () => {
    const snapshot = can([1, 2, 3, 5]);
    snapshot.devices[1].values.online = false; snapshot.devices[1].message = "Brak odpowiedzi";
    expect(driveCells(snapshot, true).map(cell => cell.state)).toEqual(["ok", "error", "ok", "stale", "ok", "stale"]);
    expect(driveCells(snapshot, true)[0].text).toBe("J1 · CAN ID 1: ENABLE 0 · 0 RPM");
    expect(driveCells(snapshot, false).every(cell => cell.state === "stale")).toBe(true);
    expect(worst(["ok", "warn", "stale"])).toBe("stale");
});
it("kanał przesuwa się o jedną kolumnę na takt i zachowuje szerokość", () => {
    let columns = Array<boolean>(PULSE_COLUMNS).fill(false);
    for (let seed = 1; seed <= 100; seed++) columns = nextColumn(columns, seed);
    expect(columns).toHaveLength(PULSE_COLUMNS);
    expect(columns.some(Boolean) && columns.some(value => !value)).toBe(true);
});

const overview: Overview = { schema_version: 1, age_seconds: 0.1, host: { os: "Ubuntu 22.04.5 LTS", kernel: "6.8.0", hostname: "robot" },
    ros: { distro: "humble", overlay: "install/stop-recovery-ros", nodes: ["/robot_web_bridge", "/robot/planner"],
        processes: [{ pid: 10, name: "planner", package: "robot_motion_planner", overlay: "install/stop-recovery-ros", cpu_percent: 2, memory_rss_bytes: 2 ** 26, threads: 12, uptime_seconds: 60 }] },
    backend: { pid: 11, cpu_percent: 3, memory_rss_bytes: 2 ** 27, threads: 9, uptime_seconds: 120, websocket_clients: { status: 2, can: 2 } },
    services: [{ unit: "robot-web-status.service", active_state: "active", sub_state: "running", result: "success", main_pid: 5 },
        { unit: "robot-can@can0.service", active_state: "failed", sub_state: "failed", result: "exit-code", main_pid: null }] };
const archInput = (over = {}) => ({ status: status(), statusFresh: true, statusOnline: true, can: can(), canFresh: true, canOnline: true,
    overview, overviewError: null, build: { revision: "abc1234", built_at: "" }, ...over });

it("strona Architektura: wiersze z przeglądu procesów i usług, problemy tylko z rzeczywistych stanów", () => {
    const rows = architectureRows(archInput());
    expect(rows.find(row => row.id === "proc:10")).toMatchObject({ group: "ROS", name: "planner", pid: 10, type: "robot_motion_planner" });
    expect(rows.find(row => row.id === "ws")!.message).toBe("status 2 · CAN 2 · ręczne 0");
    expect(rows.filter(isProblem).map(row => row.id)).toEqual(["unit:robot-can@can0.service"]);
    expect(rows.find(row => row.id === "ctrl")!.state).toBe("off");
});
it("bez przeglądu (stary backend) zostają dane statusu, a brak oznaczony jako problem z przyczyną", () => {
    const rows = architectureRows(archInput({ overview: null, overviewError: "Backend bez przeglądu systemu — wymaga restartu usługi" }));
    expect(rows.find(row => row.id === "procs")).toMatchObject({ state: "stale", message: "Backend bez przeglądu systemu — wymaga restartu usługi" });
    expect(rows.find(row => row.id === "can-link")!.message).toContain("1 Mbit/s");
});
it("wskaźniki: planowane obciążenie CAN z limitem i RTT względem timeoutu", () => {
    const meters = architectureMeters(archInput());
    expect(meters.map(meter => meter.text)).toEqual(["23%", "—/—", "31% / 60", "19,1/24 ms"]);
    expect(meters[2].limit).toBe(60);
    expect(architectureMeters(archInput({ canFresh: false }))[2].value).toBeNull();
});
