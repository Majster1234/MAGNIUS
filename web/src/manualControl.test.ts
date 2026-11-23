import { describe, expect, it } from "vitest";
import { JogControl, jogSpeedAtPercent, type JogCommand } from "./manualControl";
function rig() {
    let now = 1000;
    const commands: JogCommand[] = [];
    const socket = { readyState: 1, send: (body: string) => commands.push(JSON.parse(body)) };
    const control = new JogControl(socket, () => now);
    control.stateReceived(true);
    return { control, commands, socket, advance: (ms: number) => { now += ms; } };
}
describe("JOG przytrzymywany przez operatora", () => {
    it("zadaje prędkość po naciśnięciu i STOP po puszczeniu", () => {
        const r = rig(); expect(r.control.start(4, 1, 30)).toBe(true);
        expect(r.commands[0]).toMatchObject({ action: "start", axis: 4, rpm: 30 });
        r.control.stop(); expect(r.commands[1].action).toBe("stop");
        r.advance(50); r.control.pulse(); expect(r.commands).toHaveLength(2);
    });
    it("podtrzymuje co 50 ms i nie buduje kolejki", () => {
        const r = rig(); r.control.start(1, -1, 30); r.advance(50); r.control.pulse();
        expect(r.commands).toHaveLength(1);
        r.control.ack(1, true); r.control.pulse(); expect(r.commands[1].action).toBe("keepalive");
        r.control.pulse(); expect(r.commands).toHaveLength(2);
    });
    it("zatrzymuje po utracie statusów i wymaga nowego naciśnięcia", () => {
        const r = rig(); r.control.start(1, 1, 30); r.control.ack(1, true);
        r.advance(501); r.control.pulse(); expect(r.commands.at(-1)?.action).toBe("stop");
        r.control.stateReceived(true); r.advance(50); r.control.pulse();
        expect(r.commands.filter((command) => command.action === "start")).toHaveLength(1);
    });
    it("toleruje opóźnienie statusów bez przerywania potwierdzanych podtrzymań", () => {
        const r = rig(); r.control.start(1, 1, 5);
        for (let i = 0; i < 8; i++) {
            r.control.ack(r.commands.at(-1)!.sequence, true);
            r.advance(50); r.control.pulse();
        }
        expect(r.control.holding).toBe(true);
        expect(r.commands.some(command => command.action === "stop")).toBe(false);
    });
    it("utrata ACK wywołuje STOP mimo aktualnych statusów", () => {
        const r = rig(); r.control.start(1, 1, 30); r.advance(101); r.control.stateReceived(true); r.control.pulse();
        expect(r.commands.at(-1)?.action).toBe("stop"); expect(r.control.holding).toBe(false);
    });
    it("nie uruchamia drugiej osi i odrzuca stare ACK", () => {
        const r = rig(); r.control.start(1, 1, 30); expect(r.control.start(2, 1, 30)).toBe(false);
        r.control.ack(999, false); expect(r.control.holding).toBe(true);
        r.control.ack(1, false); expect(r.control.holding).toBe(false);
    });
    it("zamknięty socket nie wznowi ruchu", () => {
        const r = rig(); r.socket.readyState = 3; expect(r.control.start(1, 1, 30)).toBe(false);
    });
    it("JOG przegubu zachowuje jednostki i rewizję także przy STOP", () => {
        const r=rig();expect(r.control.start(2,-1,2.4,"revision-1")).toBe(true);
        expect(r.commands[0]).toMatchObject({space:"joint",velocity_degrees_s:2.4,rpm:0,revision:"revision-1"});
        r.control.ack(1,true);r.advance(50);r.control.pulse();
        expect(r.commands[1]).toMatchObject({action:"keepalive",revision:"revision-1"});
        r.control.stop();expect(r.commands.at(-1)).toMatchObject({action:"stop",space:"joint",revision:"revision-1"});
    });
});
describe("Podtrzymanie przy jitterze zegara panelu", () => {
    it("tyknięcie 45 ms po poprzedniej komendzie wysyła KEEPALIVE (bez przeskoku do 100 ms), 30 ms nie", () => {
        const r = rig(); r.control.start(2, 1, 30); r.control.ack(1, true);
        r.advance(30); r.control.pulse(); expect(r.commands).toHaveLength(1);
        r.advance(15); r.control.pulse(); expect(r.commands[1].action).toBe("keepalive");
    });
});

describe("Wspólna prędkość JOG wszystkich osi", () => {
    const joint = { joint_configured: true, jog_min_velocity_degrees_s: 0.4, jog_max_velocity_degrees_s: 7.199999999999999 };
    it("przegub: procent limitu w krokach minimalnej prędkości osi", () => {
        expect(jogSpeedAtPercent(50, joint, 30, false)).toBe(3.6);
        expect(jogSpeedAtPercent(1, joint, 30, false)).toBe(0.4);
        expect(jogSpeedAtPercent(100, joint, 30, false)).toBe(7.2);
    });
    it("przegub w planowaniu: bez kroku, w dół do 0,001 °/s", () => {
        expect(jogSpeedAtPercent(33, joint, 30, true)).toBe(2.376);
        expect(jogSpeedAtPercent(100, { joint_configured: true, jog_max_velocity_degrees_s: 7.199999999999999 }, 30, false)).toBe(7.199);
    });
    it("wał bez konfiguracji przegubu: całe obr./min w zakresie 1…max", () => {
        expect(jogSpeedAtPercent(50, {}, 30, false)).toBe(15);
        expect(jogSpeedAtPercent(1, {}, 30, false)).toBe(1);
        expect(jogSpeedAtPercent(250, {}, 30, false)).toBe(30);
    });
    it("przegub bez limitu prędkości nie zmienia nastawy", () => {
        expect(jogSpeedAtPercent(50, { joint_configured: true }, 30, false)).toBeNull();
    });
});
