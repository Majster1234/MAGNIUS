import { useCallback, useEffect, useRef, useState } from "react";
import { dirtyKeys, documentFromDraft, emptyDocument, toDraft, validateDocument, withExample, type ConfigurationDocument, type ConfigurationStatus, type Draft } from "./configurationData";
import type { Notice } from "./eventLog";
type Snapshot = {id: string; name: string; created_at: string};
const revisionKey = "robot.configuration.base_revision.v1";
function storedRevision() { try { return sessionStorage.getItem(revisionKey); } catch { return null; } }
async function api(path = "", body?: unknown, method?: string) {
    const response = await fetch(`/api/v1/configuration${path}`, body === undefined ? {method: method ?? "GET"} : {
        method: method ?? "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify(body)});
    const value = await response.json();
    if (!response.ok) throw Error(typeof value.detail === "string" ? value.detail : "Operacja konfiguracji odrzucona");
    return value;
}
export function useConfiguration(draft: Draft, setDraft: (draft: Draft) => void, enabled: boolean, hasLocal: boolean, notice: (event: Notice) => void) {
    const [status, setStatus] = useState<ConfigurationStatus | null>(null), [baseline, setBaseline] = useState(emptyDocument);
    const [baseRevision, setBaseRevision] = useState<string | null>(storedRevision);
    const [online, setOnline] = useState(false), [busy, setBusy] = useState(false);
    const [items, setItems] = useState<Snapshot[]>([]), [selected, setSelected] = useState("");
    const [name, setName] = useState(""), [nextCursor, setNextCursor] = useState<string | null>(null);
    const [cursors, setCursors] = useState([""]);
    const current = useRef({draft, baseline, baseRevision, status}); current.current = {draft, baseline, baseRevision, status};
    const initial = useRef(true), ownOperation = useRef("");
    const bindRevision = useCallback((revision: string) => {
        setBaseRevision(revision);
        try { sessionStorage.setItem(revisionKey, revision); } catch { /* szkic nadal działa w pamięci */ }
    }, []);
    const receive = useCallback((data: ConfigurationStatus) => {
        const document = validateDocument(data.active.document), before = current.current;
        const different = before.status?.active.revision !== data.active.revision;
        if (initial.current) {
            if (!hasLocal) setDraft(toDraft(document));
            if (!before.baseRevision || !hasLocal) bindRevision(data.active.revision);
            initial.current = false;
        } else if (different && (ownOperation.current === data.operation.id || !dirtyKeys(before.draft, before.baseline).size)) {
            if (!dirtyKeys(before.draft, before.baseline).size) setDraft(toDraft(document));
            bindRevision(data.active.revision);
        }
        if (different) setBaseline(document);
        setStatus(data); setOnline(true);
        notice({source:"configuration-connection", severity:"info", message:"Połączenie z węzłem konfiguracji ROS działa."});
        notice({source:"configuration-operation", severity:data.operation.phase === "uncertain" ? "error" : data.operation.success === false ? "error" : "info", message:data.operation.message});
    }, [bindRevision, hasLocal, notice, setDraft]);
    useEffect(() => {
        if (!enabled) return;
        let closed = false, timer: ReturnType<typeof setTimeout>;
        async function poll() {
            try { const data = await api(); if (!closed) receive(data); }
            catch (error) { if (!closed) { setOnline(false); notice({source:"configuration-connection", severity:"warning", message:`Konfiguracja ROS niedostępna: ${error instanceof Error ? error.message : "brak odpowiedzi"}. Szkic i podgląd działają lokalnie.`}); } }
            if (!closed) timer = setTimeout(poll, 1000);
        }
        void poll();return () => {closed = true;clearTimeout(timer);};
    }, [enabled, notice, receive]);
    const refresh = useCallback(async (cursor = "") => {
        const data = await api(`/snapshots${cursor ? `?cursor=${encodeURIComponent(cursor)}` : ""}`);
        setItems(data.items); setNextCursor(data.next_cursor);
        setSelected(previous => data.items.some((item: Snapshot) => item.id === previous) ? previous : data.items[0]?.id ?? "");
    }, []);
    useEffect(() => { if (enabled) void refresh().catch(() => {}); }, [enabled, refresh]);
    async function run(action: () => Promise<void>) {
        setBusy(true);
        try { await action(); }
        catch (error) { notice({source:"configuration-action", severity:"error", message:error instanceof Error ? error.message : "Błąd konfiguracji"}); }
        finally {setBusy(false);}
    }
    function replace(document: ConfigurationDocument) {
        if (dirtyKeys(current.current.draft, current.current.baseline).size && !window.confirm("Zastąpić niezastosowany szkic? Zmiany, których nie zapisano w snapshocie lub pliku, zostaną utracone.")) return;
        setDraft(toDraft(document)); setName(document.name);
        if (status) bindRevision(status.active.revision);
    }
    const saveSnapshot = () => run(async () => {
        const saved = await api('/snapshots', {document:documentFromDraft(draft, name)});
        await refresh(); setSelected(saved.id); setCursors([""]);
        notice({source:"configuration-action", severity:"info", message:`Snapshot „${saved.name}” zapisany w JSON. Konfiguracja ROS nie została zmieniona.`});
    });
    const loadSnapshot = () => run(async () => {
        replace(validateDocument(await api(`/snapshots/${selected}`)));
    });
    const deleteSnapshot = () => run(async () => {
        if (!window.confirm("Usunąć wybrany snapshot z komputera robota?")) return;
        await api(`/snapshots/${selected}`, undefined, "DELETE"); await refresh(cursors.at(-1));
    });
    const importFile = (file: File) => run(async () => {
        if (file.size > 65536) throw Error("Plik JSON przekracza 64 KiB");
        let document = validateDocument(JSON.parse(await file.text()));
        if (online) document = validateDocument(await api('/validate', {document}));
        replace(document);
    });
    const exportFile = () => run(async () => {
        const document = documentFromDraft(draft, name), blob = new Blob([JSON.stringify(document, null, 2) + "\n"], {type:"application/json"});
        const url = URL.createObjectURL(blob), link = window.document.createElement('a');
        link.href = url;link.download = `robot-konfiguracja-${new Date().toISOString().replace(/[:.]/g,"-")}.json`;link.click();setTimeout(() => URL.revokeObjectURL(url), 1000);
    });
    const apply = () => run(async () => {
        const data = await api('/apply', {document:documentFromDraft(draft, name), expected_revision:baseRevision});
        ownOperation.current = data.operation.id; receive(data);
    });
    const reconcile = () => run(async () => {receive(await api('/reconcile', {}));});
    const loadActive = () => {if (status) replace(status.active.document);};
    const example = () => {
        if (Object.entries(draft).some(([key,value]) => (key.includes('.origin_') || key.startsWith('tool.tcp_')) && value.trim()) && !window.confirm("Zastąpić geometrię szkicu wartościami przykładowymi? Pozostałe parametry pozostaną zachowane.")) return;
        setDraft(withExample(draft));
    };
    const page = (next: boolean) => run(async () => {
        const updated = next && nextCursor ? [...cursors, nextCursor] : cursors.slice(0,-1);
        await refresh(updated.at(-1));setCursors(updated.length ? updated : [""]);
    });
    return {status, baseline, baseRevision, online, busy, items, selected, setSelected, name, setName, nextCursor, cursors,
        saveSnapshot, loadSnapshot, deleteSnapshot, importFile, exportFile, apply, reconcile, loadActive, example, page};
}
