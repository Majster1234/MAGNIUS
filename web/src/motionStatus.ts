export type MotionStatus = {
    dispatch_mode?: "streamed" | "synchronized"; sync_cycle?: number; sync_state?: string; sync_lateness_ms?: number;
    state: string; program_id: string; mode: string; message: string; busy: boolean; progress: number;
    duration: number; elapsed?: number; start_positions: number[]; target_positions: number[];
    motion_type?: "PTP" | "LIN" | null;
    preparation?: { id?: string; state: string; preview_plan_id?: string; program_id?: string; motion_type?: "PTP" | "LIN"; message?: string };
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

export type ExecutionTuning = { lookahead_time_sec: number; velocity_ff_scale: number; position_kp: number; integral_gain: number; integral_limit_rad: number; acceleration_code?: number };
// Opis nastaw testowych dla Centrum komunikatów: tylko pola różne od wartości z pliku parametrów.
export function describeExecutionTuning(value: MotionStatus["execution_tuning"]): string | null {
    if (!value?.test) return null;
    const fields: [keyof ExecutionTuning, string, (v: number) => string][] = [
        ["lookahead_time_sec", "wyprzedzenie celu", v => `${Math.round(v * 1000)} ms`], ["velocity_ff_scale", "FF", v => String(+v.toFixed(3))],
        ["position_kp", "Kp", v => String(+v.toFixed(3))], ["integral_gain", "wzmocnienie całkujące", v => String(+v.toFixed(3))],
        ["integral_limit_rad", "limit całkujący", v => `${+(v * 180 / Math.PI).toFixed(2)}°`],
        ["acceleration_code", "acc F5", v => v < 0 ? "z konfiguracji" : v === 0 ? "0 (bez rampy)" : String(v)]];
    const changed = fields.filter(([key]) => (value.active[key] ?? -1) !== (value.baseline[key] ?? -1))
        .map(([key, label, format]) => `${label} ${format(value.active[key] ?? -1)} (plik: ${format(value.baseline[key] ?? -1)})`);
    return `Na robocie działają testowe nastawy regulatora wykonania: ${changed.join(", ")}. Przywrócenie: zakończenie testu lub restart usługi.`;
}
