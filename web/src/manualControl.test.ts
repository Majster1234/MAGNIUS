import { describe, expect, it } from "vitest";
import { JogControl, type JogCommand } from "./manualControl";
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
});
