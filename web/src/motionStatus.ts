export type MotionStatus = {
    dispatch_mode?: "streamed" | "synchronized"; sync_cycle?: number; sync_state?: string; sync_lateness_ms?: number;
    state: string; program_id: string; mode: string; message: string; busy: boolean; progress: number;
    duration: number; elapsed?: number; start_positions: number[]; target_positions: number[];
    motion_type?: "PTP" | "LIN" | "CHAIN" | null;
    preparation?: { id?: string; state: string; preview_plan_id?: string; program_id?: string; motion_type?: "PTP" | "LIN" | "CHAIN"; message?: string };
    participating: boolean[]; tcp_verified: boolean; execution_key: string;
    // Nastawy regulatora wykonania (zmieniane poza konfiguracją przez testy jakości ruchu); test=true gdy różne od pliku.
    execution_tuning?: { test: boolean; active: ExecutionTuning; baseline: ExecutionTuning } | null;
};

// Czas odbioru w przeglądarce, we wspólnej dziedzinie performance.now().
export type MotionObservation = { status: MotionStatus; receivedAt: number };
export const MOTION_STATUS_MAX_AGE_MS=750;
export function motionStatusFresh(receivedAt:number,now:number,failed=false):boolean {return !failed && Number.isFinite(receivedAt) && now>=receivedAt && now-receivedAt<MOTION_STATUS_MAX_AGE_MS;}
export function motionReady(status:MotionStatus|null,planId:string|null,type:"PTP"|"LIN"|null,expected:string|null,fresh:boolean):boolean {
    return !!(fresh && planId && type && status?.state==="ready" && status.busy===false && status.program_id && status.preparation?.id && status.preparation.state==="ready" &&
        status.preparation.preview_plan_id===planId && status.preparation.program_id===status.program_id &&
        status.motion_type===type && status.preparation.motion_type===type && (!expected || status.preparation.id===expected));
}
export const motionStateLabels:Record<string,string>={idle:"Oczekiwanie",ready:"Gotowy do zatwierdzenia",arming:"Weryfikacja napędów",warming:"Odczyt startu",staging:"Przygotowanie startu",running:"W ruchu",waiting_position:"Oczekiwanie na pozycję osi",waiting_feedback:"Oczekiwanie na pomiary",waiting_budget:"Oczekiwanie na budżet CAN",settling:"Potwierdzanie celu",restoring:"Przywracanie trybu napędów",succeeded:"Zakończony",failed:"Przerwany",invalidated:"Wymaga przygotowania"};

export type ExecutionTuning = { lookahead_time_sec: number; velocity_ff_scale: number; position_kp: number; integral_gain: number; integral_limit_rad: number; acceleration_code?: number; acceleration_ff_scale?: number; gain_schedule?: boolean; hold_period_cycles?: number; hold_rpm?: number; hold_acceleration?: number; hold_tail_ms?: number; hold_stop_mode?: number; hold_stop_ticks?: number; hold_tail_end_ms?: number; hold_mark_scale?: number; hold_silent?: number };
// Opis nastaw testowych dla Centrum komunikatów: tylko pola różne od wartości z pliku parametrów.
export function describeExecutionTuning(value: MotionStatus["execution_tuning"]): string | null {
    if (!value?.test) return null;
    const fields: [keyof ExecutionTuning, string, (v: number) => string][] = [
        ["lookahead_time_sec", "wyprzedzenie celu", v => `${Math.round(v * 1000)} ms`], ["velocity_ff_scale", "FF", v => String(+v.toFixed(3))],
        ["position_kp", "Kp", v => String(+v.toFixed(3))], ["integral_gain", "wzmocnienie całkujące", v => String(+v.toFixed(3))],
        ["integral_limit_rad", "limit całkujący", v => `${+(v * 180 / Math.PI).toFixed(2)}°`],
        ["acceleration_code", "acc F5", v => v < 0 ? "z konfiguracji" : v === 0 ? "0 (bez rampy)" : String(v)],
        ["acceleration_ff_scale", "składnik przyspieszenia F5 (LIN)", v => v < 0 ? "brak" : String(+v.toFixed(3))],
        ["hold_period_cycles", "okres ramek trzymania", v => v < 0 ? "brak" : `${v} cykli`],
        ["hold_rpm", "prędkość trzymania", v => v <= 0 ? "standardowa" : `${+v.toFixed(3)} RPM`],
        ["hold_acceleration", "acc końca ruchu", v => v < 0 ? "bez zmiany" : String(v)],
        ["hold_tail_ms", "końcówka trajektorii z acc końca", v => `${Math.max(v, 0)} ms`],
        ["hold_stop_mode", "zatrzymanie przy celu", v => v === 1 ? "F7" : v === 2 ? "F5 STOP" : "brak"],
        ["hold_stop_ticks", "odległość zatrzymania", v => `${Math.max(v, 0)} imp.`],
        ["hold_tail_end_ms", "koniec okna końcówki", v => `${Math.max(v, 0)} ms przed końcem`],
        ["hold_mark_scale", "mnożnik prędkości ramek okna", v => String(+v.toFixed(3))],
        ["hold_silent", "cisza po ramce celu", v => v === 1 ? "włączona" : "wyłączona"]];
    const number = (tuning: ExecutionTuning, key: keyof ExecutionTuning) => Number(tuning[key] ?? -1);
    const changed = fields.filter(([key]) => number(value.active, key) !== number(value.baseline, key))
        .map(([key, label, format]) => `${label} ${format(number(value.active, key))} (plik: ${format(number(value.baseline, key))})`);
    if (Boolean(value.active.gain_schedule) !== Boolean(value.baseline.gain_schedule))
        changed.push(`nastawy zależne od ruchu ${value.active.gain_schedule ? "włączone" : "wyłączone"}`);
    return `Na robocie działają testowe nastawy regulatora wykonania: ${changed.join(", ")}. Przywrócenie: zakończenie testu lub restart usługi.`;
}
