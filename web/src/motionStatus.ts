export type MotionStatus = {
    state: string; program_id: string; mode: string; message: string; busy: boolean; progress: number;
    duration: number; elapsed?: number; start_positions: number[]; target_positions: number[];
    motion_type?: "PTP" | "LIN" | null;
    preparation?: { id?: string; state: string; preview_plan_id?: string; program_id?: string; motion_type?: "PTP" | "LIN"; message?: string };
    participating: boolean[]; tcp_verified: boolean; execution_key: string;
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
export const motionStateLabels:Record<string,string>={idle:"Oczekiwanie",ready:"Gotowy do zatwierdzenia",arming:"Weryfikacja napędów",warming:"Odczyt startu",staging:"Przygotowanie startu",running:"W ruchu",waiting_position:"Oczekiwanie na pozycję osi",waiting_feedback:"Oczekiwanie na pomiary",waiting_budget:"Oczekiwanie na budżet CAN",settling:"Potwierdzanie celu",succeeded:"Zakończony",failed:"Przerwany",invalidated:"Wymaga przygotowania"};
