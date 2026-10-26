export type Axis = {
    axis: number; available: boolean; prepared: boolean; enable_known: boolean; enabled: boolean;
    position_known: boolean; position_degrees: number; position_age_seconds?: number | null;
    speed_known: boolean; speed_rpm: number; can_jog: boolean; can_zero: boolean; motion: string;
    joint_configured?: boolean; calibration_valid?: boolean; joint_position_known?: boolean;
    joint_position_degrees?: number; joint_lower_degrees?: number; joint_upper_degrees?: number;
    jog_min_velocity_degrees_s?: number; jog_max_velocity_degrees_s?: number; configuration_message?: string;
};
export type ManualState = {
    tcp_jog?: import("./physicalTcpJog").TcpJogStatus;
    fresh: boolean; busy: boolean; fault: boolean; recording?: boolean; active_axis: number;
    max_rpm: number; operation: string; message: string; axes: Axis[];
    configuration_revision?: string;
    state_monotonic_ns?: string;
};
