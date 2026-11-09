import { useEffect, useState } from "react";
import { isSnapshot, type Snapshot } from "./status";

export function useStatus() {
    const [snapshot, setSnapshot] = useState<Snapshot | null>(null);
    const [connection, setConnection] = useState<
        "connecting" | "online" | "offline"
    >("connecting");
    const [receivedAt, setReceivedAt] = useState(0);
    const [now, setNow] = useState(performance.now());
    useEffect(() => {
        let stopped = false;
        let socket: WebSocket | null = null;
        let retry: ReturnType<typeof setTimeout>;
        let delay = 1000;
        let lastMessage = performance.now();
        const connect = () => {
            if (stopped) return;
            setConnection("connecting");
            lastMessage = performance.now();
            socket = new WebSocket(
                `${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/api/v1/ws`,
            );
            socket.onmessage = (event) => {
                if (stopped) return;
                try {
                    const value: unknown = JSON.parse(event.data);
                    if (!isSnapshot(value))
                        throw new Error("Nieprawidłowy format statusu");
                    lastMessage = performance.now();
                    setSnapshot(value);
                    setReceivedAt(lastMessage);
                    setConnection("online");
                    delay = 1000;
                } catch {
                    socket?.close();
                }
            };
            socket.onerror = () => socket?.close();
            socket.onclose = () => {
                if (stopped) return;
                setConnection("offline");
                retry = setTimeout(connect, delay);
                delay = Math.min(delay * 2, 5000);
            };
        };
        connect();
        const timer = setInterval(() => {
            const timestamp = performance.now();
            setNow(timestamp);
            if (
                socket &&
                timestamp - lastMessage >= 3000 &&
                socket.readyState < WebSocket.CLOSING
            ) {
                setConnection("offline");
                socket.close();
            }
        }, 200);
        return () => {
            stopped = true;
            clearInterval(timer);
            clearTimeout(retry);
            socket?.close();
        };
    }, []);
    return { snapshot, connection, receivedAt, now };
}
