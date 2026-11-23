export type State = "ok" | "warn" | "error" | "stale" | "not_configured";
export type Component = {
    id: string;
    state: State;
    message: string;
    sampled_at: string | null;
    values: Record<string, unknown>;
};
export type Snapshot = {
    schema_version: 1;
    updated_at: string | null;
    age_seconds: number | null;
    fresh: boolean;
    network: {
        address: string;
        port: number;
        interface: string;
        subnet: string;
        url: string;
    };
    components: Record<
        "api" | "ros" | "host" | "can" | "controllers" | "drives",
        Component
    >;
};
const states = ["ok", "warn", "error", "stale", "not_configured"];
const object = (value: unknown): value is Record<string, unknown> =>
    typeof value === "object" && value !== null && !Array.isArray(value);

export function isSnapshot(value: unknown): value is Snapshot {
    if (
        !object(value) ||
        value.schema_version !== 1 ||
        !object(value.components) ||
        !object(value.network) ||
        typeof value.fresh !== "boolean" ||
        !(
            value.age_seconds === null ||
            (typeof value.age_seconds === "number" &&
                Number.isFinite(value.age_seconds) &&
                value.age_seconds >= 0)
        ) ||
        !(value.updated_at === null || typeof value.updated_at === "string")
    )
        return false;
    const network = value.network;
    if (
        !["address", "interface", "subnet", "url"].every(
            (key) => typeof network[key] === "string",
        ) ||
        typeof network.port !== "number"
    )
        return false;
    return ["api", "ros", "host", "can", "controllers", "drives"].every(
        (key) => {
            const item = (value.components as Record<string, unknown>)[key];
            return (
                object(item) &&
                typeof item.id === "string" &&
                typeof item.state === "string" &&
                states.includes(item.state) &&
                typeof item.message === "string" &&
                object(item.values) &&
                (item.sampled_at === null ||
                    typeof item.sampled_at === "string")
            );
        },
    );
}

export function dataIsStale(
    snapshot: Snapshot | null,
    receivedAt: number,
    now: number,
) {
    return (
        !snapshot ||
        !snapshot.fresh ||
        snapshot.age_seconds === null ||
        snapshot.age_seconds + Math.max(0, now - receivedAt) / 1000 >= 3
    );
}

export function numeric(value: unknown): number | null {
    return typeof value === "number" && Number.isFinite(value) ? value : null;
}
