// Jeden zestaw ikon obrysowych (24×24, grubość z tokenu --ico-stroke) zamiast znaków Unicode i emoji.
const paths = {
    robot: <><path d="M5 21h14M8 21v-3h8v3M12 18v-4.5M13.4 11.6l3.2-3.2M18.4 6.6L20 5"/><circle cx="12" cy="12" r="2"/><circle cx="17.5" cy="7.5" r="1.5"/></>,
    overview: <><rect x="3" y="3" width="7" height="9" rx="1"/><rect x="14" y="3" width="7" height="5" rx="1"/><rect x="14" y="12" width="7" height="9" rx="1"/><rect x="3" y="16" width="7" height="5" rx="1"/></>,
    can: <><path d="M3 12h18M6 12V7M12 12v5M18 12V7"/><circle cx="6" cy="5" r="1.6"/><circle cx="12" cy="19" r="1.6"/><circle cx="18" cy="5" r="1.6"/></>,
    diag: <path d="M3 12h4l3-7 4 14 3-7h4"/>,
    manual: <path d="M12 3v18M3 12h18M9 6l3-3 3 3M9 18l3 3 3-3M6 9l-3 3 3 3M18 9l3 3-3 3"/>,
    program: <><rect x="4" y="3" width="10" height="5" rx="1"/><rect x="10" y="10" width="10" height="5" rx="1"/><rect x="4" y="17" width="10" height="4" rx="1"/><path d="M9 8v2.5h1M15 15v2h-1"/></>,
    config: <><path d="M4 6h10M18 6h2M4 12h4M12 12h8M4 18h12"/><circle cx="16" cy="6" r="2"/><circle cx="10" cy="12" r="2"/><circle cx="18" cy="18" r="2"/></>,
    arch: <><path d="M12 3l9 4.5-9 4.5-9-4.5z"/><path d="M3 12l9 4.5 9-4.5M3 16.5l9 4.5 9-4.5"/></>,
    phone: <><rect x="7" y="2.5" width="10" height="19" rx="2"/><path d="M11 18h2"/></>,
    alert: <><path d="M12 3.5l9.5 17h-19z"/><path d="M12 10v4.5M12 17.5v.5"/></>,
    check: <path d="M4.5 12.5l4.5 4.5L19.5 6.5"/>,
    collapse: <path d="M11 6l-6 6 6 6M19 6l-6 6 6 6"/>,
    power: <><path d="M12 3v9"/><path d="M6.3 6.8a8 8 0 1 0 11.4 0"/></>,
    scan: <><path d="M20 12a8 8 0 1 1-2.3-5.7"/><path d="M20 4v5h-5"/></>,
    info: <><circle cx="12" cy="12" r="9"/><path d="M12 11v6M12 7.5v.5"/></>,
} as const;
export type IconName = keyof typeof paths;

export function Icon({ name, className }: { name: IconName; className?: string }) {
    return <svg className={`ico${className ? ` ${className}` : ""}`} viewBox="0 0 24 24" aria-hidden="true" focusable="false">{paths[name]}</svg>;
}
