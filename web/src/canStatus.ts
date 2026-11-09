import { useEffect, useState } from "react";
import type { State } from "./status";

export type CanDevice = {
    id: number;
    state: State;
    message: string;
    values: Record<string, unknown>;
};
export type CanSnapshot = {
    schema_version: 1;
    updated_at: string | null;
    age_seconds: number | null;
    fresh: boolean;
    interface: string;
    summary: { state: State; message: string; values: Record<string, unknown> };
    scan: { state: State; message: string; values: Record<string, unknown> };
    devices: CanDevice[];
};
const states = ["ok", "warn", "error", "stale", "not_configured"];
const object = (value: unknown): value is Record<string, unknown> =>
    typeof value === "object" && value !== null && !Array.isArray(value);
export function isCanSnapshot(value: unknown): value is CanSnapshot {
    return object(value) && value.schema_version === 1 && typeof value.fresh === "boolean" &&
        typeof value.interface === "string" && object(value.summary) && object(value.scan) &&
        Array.isArray(value.devices) && value.devices.every((device) => object(device) &&
            typeof device.id === "number" && typeof device.state === "string" &&
            states.includes(device.state) && object(device.values));
}
export function useCanStatus(enabled: boolean) {
    const [snapshot, setSnapshot] = useState<CanSnapshot | null>(null);
    const [connection, setConnection] = useState<"connecting" | "online" | "offline">("offline");
    const [receivedAt, setReceivedAt] = useState(0);
    useEffect(() => {
        if (!enabled) { setConnection("offline"); return; }
        let stopped = false, socket: WebSocket | null = null;
        let retry: ReturnType<typeof setTimeout>, delay = 500;
        const connect = () => {
            if (stopped) return;
            setConnection("connecting");
            socket = new WebSocket(`${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/api/v1/can/ws`);
            socket.onmessage = (event) => {
                try {
                    const value: unknown = JSON.parse(event.data);
                    if (!isCanSnapshot(value)) throw new Error("format");
                    setSnapshot(value); setReceivedAt(performance.now()); setConnection("online"); delay = 500;
                } catch { socket?.close(); }
            };
            socket.onerror = () => socket?.close();
            socket.onclose = () => { if (!stopped) { setConnection("offline"); retry = setTimeout(connect, delay); delay = Math.min(delay * 2, 5000); } };
        };
        connect();
        return () => { stopped = true; clearTimeout(retry); socket?.close(); };
    }, [enabled]);
    return { snapshot, connection, receivedAt };
}
