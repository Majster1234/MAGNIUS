// Zakresy osi w edytorze programów: zakresy z aktywnej konfiguracji (joint.N.lower/upper; null = brak danych, nie 0),
// pozycje programu poza zakresem, wskaźniki limitów dla modelu 3D (jak w sterowaniu ręcznym). Odpowiednik backendu:
// motion_chain.joint_ranges / program_runner.position_limit_report.
import { previewLimits, type LimitIndicator } from "./previewLimits";
import { isRef, type Block, type Problem, type Program } from "./programModel";

export type JointRange = [number, number] | null;
export type Violation = { axis: number; value: number; lower: number; upper: number };

export function jointRanges(values: Record<string, unknown> | null): JointRange[] {
    return Array.from({ length: 6 }, (_, i) => {
        const low = values?.[`joint.${i + 1}.lower`], high = values?.[`joint.${i + 1}.upper`];
        return typeof low === "number" && typeof high === "number" && Number.isFinite(low) && Number.isFinite(high) && low < high ? [low, high] : null;
    });
}
export function violations(joints: number[], ranges: JointRange[]): Violation[] {
    const out: Violation[] = [];
    joints.forEach((value, i) => { const r = ranges[i]; if (r && (value < r[0] - 1e-6 || value > r[1] + 1e-6)) out.push({ axis: i + 1, value, lower: r[0], upper: r[1] }); });
    return out;
}
export const describe = (list: Violation[]) => list.map(v => `J${v.axis} = ${+v.value.toFixed(2)}° (zakres ${v.lower}…${v.upper}°)`).join(", ");

// Wskaźniki limitów dla pozy modelu (≤ 15° ostrzeżenie, ≤ 5° niebezpiecznie, < 0 przekroczenie).
export function poseLimits(ranges: JointRange[], joints: number[] | null): LimitIndicator[][] {
    return ranges.map((r, i) => joints && r ? previewLimits({ joint_configured: true, calibration_valid: true, joint_lower_degrees: r[0], joint_upper_degrees: r[1] },
        joints[i] ?? null, true) : []);
}

// Pozycje, do których program może pojechać: odwołania w blokach włączonych (→ pierwszy blok) i wartości POZYCJA zmiennych.
export function usedPositions(program: Program): Map<string, string | null> {
    const out = new Map<string, string | null>();
    const walk = (blocks: Block[]) => { for (const b of blocks) { if (b.disabled) continue;
        for (const v of Object.values(b.params)) if (isRef(v) && "position" in v && !out.has(v.position)) out.set(v.position, b.id);
        for (const [key, child] of Object.entries(b)) if (key !== "params" && Array.isArray(child)) walk(child as Block[]); } };
    walk(program.blocks);
    for (const v of program.variables) if (v.type === "POZYCJA" && typeof v.init === "string" && v.init && !out.has(v.init)) out.set(v.init, null);
    return out;
}
export function limitProblems(program: Program, ranges: JointRange[]): Problem[] {
    const used = usedPositions(program), out: Problem[] = [];
    for (const p of program.positions) {
        const bad = violations(p.joints_deg, ranges);
        if (bad.length && used.has(p.name)) out.push({ block: used.get(p.name) ?? undefined, text: `Pozycja ${p.name} poza zakresem osi: ${describe(bad)}` });
    }
    return out;
}
