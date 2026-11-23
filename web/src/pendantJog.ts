// Logika teach pendanta (PendantPage.tsx): drążek → jedno polecenie JOG (backend porusza jedną oś przegubu albo jeden
// kierunek TCP naraz, prędkość ustalana przy starcie). Pierwszy kierunek wychylenia wybiera oś pary; zmiana osi wymaga
// powrotu drążka do środka. Ruch tylko przy wciśniętym zezwoleniu (deadman).
import type { Axis } from "./manualTypes";
import type { TcpJogStatus } from "./physicalTcpJog";
import type { TcpAxis } from "./planningTcpJog";

export type PendantMode = "tcp" | "joint";
export const TCP_PAIRS: [TcpAxis, TcpAxis][] = [["x", "y"], ["z", "rz"], ["rx", "ry"]];
export const JOINT_PAIRS: [number, number][] = [[1, 2], [3, 4], [5, 6]];
export const SPEED_PRESETS = [{ id: "slow", label: "Wolno", fraction: .1 }, { id: "mid", label: "Średnio", fraction: .3 }, { id: "fast", label: "Szybko", fraction: .6 }] as const;
export type Intent = { mode: "joint"; axis: number; direction: 1 | -1 } | { mode: "tcp"; axis: TcpAxis; direction: 1 | -1 };

export const ENGAGE = .4;    // wychylenie (0–1) uruchamiające ruch
export const RELEASE = .2;   // poniżej — drążek w środku, blokada osi zwolniona
export const sameIntent = (a: Intent | null, b: Intent | null) => a === b || (!!a && !!b && a.mode === b.mode && a.axis === b.axis && a.direction === b.direction);
export const tcpLabel = (axis: TcpAxis) => axis.length === 2 ? `R${axis[1]}` : axis.toUpperCase();
export const axisLabel = (intent: Intent) => intent.mode === "joint" ? `J${intent.axis}` : tcpLabel(intent.axis);

export class StickSelector {
    private locked: 0 | 1 | null = null;
    reset() { this.locked = null; }
    // x, y w [-1, 1] (y w górę dodatni); para: TCP → [oś x, oś y], osie → [Jx, Jy].
    update(x: number, y: number, mode: PendantMode, pair: number): Intent | null {
        const magnitude = Math.hypot(x, y);
        if (magnitude < RELEASE) { this.locked = null; return null; }
        if (this.locked === null) {
            if (magnitude < ENGAGE) return null;
            this.locked = Math.abs(x) >= Math.abs(y) ? 0 : 1;
        }
        const value = this.locked === 0 ? x : y;
        if (Math.abs(value) < ENGAGE * .6) return null;   // zablokowana oś, ale składowa za mała → postój
        const direction = value > 0 ? 1 : -1;
        return mode === "joint" ? { mode, axis: JOINT_PAIRS[pair][this.locked], direction } : { mode, axis: TCP_PAIRS[pair][this.locked], direction };
    }
}

// Prędkość osi przegubu [°/s]: ułamek profilu JOG osi, nie mniej niż minimum osi.
export function jointSpeed(axis: Axis | undefined, fraction: number) {
    const max = axis?.jog_max_velocity_degrees_s ?? 0, min = axis?.jog_min_velocity_degrees_s ?? 0;
    return max > 0 ? Math.max(min, max * fraction) : 0;
}
// Prędkość TCP [mm/s lub °/s]: ułamek limitu z wykonawcy JOG TCP (status w m/s, rad/s).
export function tcpSpeed(status: TcpJogStatus | undefined, axis: TcpAxis, fraction: number) {
    if (!status) return 0;
    return ["x", "y", "z"].includes(axis) ? status.max_linear_velocity * 1000 * fraction : status.max_angular_velocity * 180 / Math.PI * fraction;
}
// Dlaczego osi nie można ruszyć (null = można).
export function jointBlocker(axis: Axis | undefined, revision: string | undefined): string | null {
    if (!axis?.available) return "napęd niedostępny";
    if (!axis.joint_configured) return "brak profilu przegubu (konfiguracja)";
    if (!axis.calibration_valid) return "nieważna kalibracja";
    if (!axis.joint_position_known) return "nieznana pozycja";
    if (axis.enable_known && !axis.enabled) return "napęd wyłączony (ENABLE=0)";
    if (!revision) return "brak rewizji konfiguracji";
    return null;
}
