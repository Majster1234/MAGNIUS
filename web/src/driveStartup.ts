// Wynik sprawdzenia nastaw napędów po inicjalizacji (/api/v1/drive-startup).
export type DriveDifference = { expected: string | null; actual: string | null; expected_value: unknown; actual_value: unknown; writable: boolean };
export type AxisDifferences = Record<string, Record<string, DriveDifference>>;
export type DriveStartupState = {
    phase: "waiting" | "checking" | "ok" | "differences" | "applying" | "applied" | "skipped" | "check_failed" | "no_reference" | "disabled";
    generation: number | null;
    attention: boolean;
    axes: number[];
    differences: AxisDifferences;
    manual_differences: AxisDifferences;
    failed_axes: number[];
    message: string;
    last_apply: null | {
        axes: Record<string, { written: string[]; error: string | null }>;
        error: string | null;
        requires_power_cycle: boolean;
    };
};
export type DifferenceRow = { key: string; axis: string; name: string; label: string; expected: string; actual: string };

const LABELS: Record<string, string> = {
    work_mode: "Tryb pracy", work_current_ma: "Prąd roboczy [mA]", hold_current_ratio: "Prąd trzymania", microstep: "Mikrokrok",
    en_active_level: "Poziom aktywny EN", direction: "Kierunek", auto_screen_off: "Wygaszanie ekranu", stall_protection: "Ochrona przeciążeniowa",
    subdivision_interpolation: "Interpolacja", can_bitrate: "Prędkość CAN", response_mode: "Tryb odpowiedzi", group_can_id: "Grupowy ID CAN",
    key_lock: "Blokada klawiszy", position_reached_threshold: "Próg osiągnięcia pozycji", vfoc_kp_ki: "PID vFOC Kp, Ki", vfoc_kd_kv: "PID vFOC Kd, Kv",
    close_kp_ki: "PID CLOSE Kp, Ki", close_kd_kv: "PID CLOSE Kd, Kv", heartbeat_ms: "Heartbeat [ms]", position_error_protection: "Ochrona błędu pozycji",
    in1_port_mode: "Tryb wejścia IN_1", user_id: "ID użytkownika", sync_mode: "Tryb SYNC", no_crc_mode: "Brak weryfikacji CRC",
    home_parameters: "Parametry bazowania", home_offset_current: "Przesunięcie i prąd bazowania", single_turn_home: "Bazowanie jednoobrotowe",
    limit_port_mapping: "Mapowanie krańcówek", pulse_divider_output: "Dzielnik impulsów", power_on_autorun: "Autostart po włączeniu",
};

export function parameterLabel(name: string) { return LABELS[name] ?? name; }

export function formatDriveValue(value: unknown): string {
    if (value === null || value === undefined) return "brak odczytu";
    return Array.isArray(value) ? value.join(", ") : String(value);
}

export function differenceRows(differences: AxisDifferences): DifferenceRow[] {
    return Object.keys(differences).sort((a, b) => Number(a) - Number(b)).flatMap(axis =>
        Object.entries(differences[axis]).map(([name, entry]) => ({
            key: `${axis}-${name}`, axis: `J${axis}`, name, label: parameterLabel(name),
            expected: formatDriveValue(entry.expected_value), actual: formatDriveValue(entry.actual_value),
        })));
}

export function axisList(axes: Array<string | number>) {
    return [...axes].map(Number).sort((a, b) => a - b).map(axis => `J${axis}`).join(", ");
}

// Błędy zapisu pogrupowane do wyświetlenia; pusta lista oznacza zapis bez błędów.
export function applyErrors(state: DriveStartupState): string[] {
    const report = state.last_apply;
    if (!report) return [];
    const axes = Object.entries(report.axes).filter(([, entry]) => entry.error).map(([axis, entry]) => `J${axis}: ${entry.error}`);
    return report.error ? [report.error, ...axes] : axes;
}
