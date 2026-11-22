// Górny pasek aplikacji (PC): przycisk „Teach pendant” z kodem QR do otwarcia pendantu na telefonie (trasa #pendant).
// Adres z /api/v1/status (network.url) — serwer przyjmuje tylko klientów z sieci LAN i tylko ten adres w nagłówku Host.
// Parametr fs=1: po otwarciu strona pokazuje „▶ Uruchom” — jedno dotknięcie przełącza w pełny ekran poziomo
// (przeglądarki pozwalają na pełny ekran tylko po geście użytkownika; po instalacji na ekranie głównym — od razu).
import { useEffect, useMemo, useRef, useState } from "react";
import { Icon } from "./Icon";
import { encodeQr, qrSvgPath } from "./qrCode";

export const pendantUrl = (base: string) => `${base.replace(/\/+$/, "")}/?fs=1#pendant`;

export function PendantLink({ baseUrl, subnet }: { baseUrl: string; subnet?: string }) {
    const [open, setOpen] = useState(false);
    const box = useRef<HTMLDivElement>(null);
    const url = pendantUrl(baseUrl);
    const qr = useMemo(() => qrSvgPath(encodeQr(url, "M")), [url]);
    useEffect(() => {
        if (!open) return;
        const close = (e: MouseEvent) => { if (!box.current?.contains(e.target as Node)) setOpen(false); };
        const key = (e: KeyboardEvent) => { if (e.key === "Escape") setOpen(false); };
        document.addEventListener("mousedown", close); document.addEventListener("keydown", key);
        return () => { document.removeEventListener("mousedown", close); document.removeEventListener("keydown", key); };
    }, [open]);
    return <div className="pendant-link" ref={box}>
        <button type="button" className={`pendant-button${open ? " on" : ""}`} aria-expanded={open} aria-haspopup="dialog" title="Kod QR — teach pendant na telefonie"
            onClick={() => setOpen(o => !o)}><Icon name="phone" /> Teach pendant</button>
        {open && <div className="pendant-popover" role="dialog" aria-label="Teach pendant na telefonie">
            <b>Teach pendant na telefonie</b>
            <svg className="pendant-qr" viewBox={`0 0 ${qr.size} ${qr.size}`} role="img" aria-label={`Kod QR: ${url}`} shapeRendering="crispEdges">
                <rect width={qr.size} height={qr.size} fill="#fff" /><path d={qr.d} fill="#000" /></svg>
            <code className="pendant-url">{url}</code>
            <ol>
                <li>Telefon w tej samej sieci WiFi{subnet ? ` (${subnet})` : ""}; zeskanuj kod aparatem.</li>
                <li>Dotknij <b>▶ Uruchom</b> — pendant przejdzie w pełny ekran poziomo.</li>
                <li>Ruch tylko przy wciśniętym <b>ZEZWOLENIU</b> (lewy kciuk) i drążku (prawy).</li>
            </ol>
            <p className="pendant-note">iPhone: pełny ekran po „Udostępnij → Do ekranu początkowego”. Na Androidzie „Zainstaluj aplikację”
                otwiera pendant od razu w pełnym ekranie.</p>
            <a className="pendant-open" href="/?fs=1#pendant" target="_blank" rel="noreferrer">Otwórz w tej przeglądarce ↗</a>
        </div>}
    </div>;
}
