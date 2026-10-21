import { useEffect, useState } from "react";

export type PreviewContext = {
    model_id: string; configuration_revision: string; settings_revision: string;
    planner_available: boolean; ignore_missing_drivers: boolean; available: boolean[];
    preview_seed_degrees: number[]; seed_sources: ("measured" | "last" | "zero")[];
    profiles: { configured: boolean; lower: number; upper: number; velocity: number }[];
    tcp_jog_segmented?: boolean;
    tcp_step_available?: boolean;
};
export const contextKey = (context: PreviewContext | null) => context ? `${context.model_id}/${context.configuration_revision}/${context.settings_revision}` : "";
export async function planningRequest(path: string, data?: unknown, method = "POST") {
    const result = await fetch(`/api/v1/${path}`, data === undefined ? { signal: AbortSignal.timeout(2500) } : {
        method, headers: { "Content-Type": "application/json" }, body: JSON.stringify(data), signal: AbortSignal.timeout(5000),
    });
    const answer = await result.json();
    if (!result.ok) throw Error(answer.detail || "Operacja planowania niedostępna");
    return answer;
}
export function usePlanningContext() {
    const [context, setContext] = useState<PreviewContext | null>(null);
    useEffect(() => {
        let closed = false, running = false;
        async function update() {
            if (running) return; running = true;
            try {
                const data = await planningRequest("planning/context");
                if (!Array.isArray(data.profiles) || data.profiles.length !== 6 || !Array.isArray(data.preview_seed_degrees) || data.preview_seed_degrees.length !== 6) throw Error("Brak kontekstu symulacji");
                if (!closed) setContext(data);
            } catch { if (!closed) setContext(null); }
            finally { running = false; }
        }
        void update(); const timer = setInterval(() => void update(), 1000);
        return () => { closed = true; clearInterval(timer); };
    }, []);
    return context;
}

export function useSystemSettings() {
    const [settings, setSettings] = useState<{ revision: string; ignore_missing_drivers: boolean; allow_missing_motion?: boolean } | null>(null);
    const [error, setError] = useState(""), [saving, setSaving] = useState(false);
    useEffect(() => {
        let closed = false, running = false;
        async function update() {
            if (running || saving) return; running = true;
            try {
                const value = await planningRequest("configuration/system");
                if (typeof value.ignore_missing_drivers !== "boolean") throw Error("Brak ustawień systemowych");
                if (!closed) { setSettings(value); setError(old => old === "Ustawienia systemowe są niedostępne." ? "" : old); }
            } catch { if (!closed) { setSettings(null); setError("Ustawienia systemowe są niedostępne."); } }
            finally { running = false; }
        }
        void update(); const timer = setInterval(() => void update(), 1000);
        return () => { closed = true; clearInterval(timer); };
    }, [saving]);
    async function save(enabled: boolean, key: "ignore_missing_drivers" | "allow_missing_motion" = "ignore_missing_drivers") {
        setSaving(true); setError("");
        try { setSettings(await planningRequest("configuration/system", { [key]: enabled }, "PUT")); }
        catch (failure) { setError((failure as Error).message); }
        finally { setSaving(false); }
    }
    return { settings, error, saving, save };
}
