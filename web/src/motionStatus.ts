export type MotionStatus = {
    state: string; program_id: string; mode: string; message: string; busy: boolean; progress: number;
    duration: number; elapsed?: number; start_positions: number[]; target_positions: number[];
    preparation?: { state: string; preview_plan_id?: string; program_id?: string; message?: string };
    participating: boolean[]; tcp_verified: boolean; execution_key: string;
};

// Czas odbioru w przeglądarce, we wspólnej dziedzinie performance.now().
export type MotionObservation = { status: MotionStatus; receivedAt: number };
