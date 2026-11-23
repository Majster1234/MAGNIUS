import type { ReactNode } from "react";
import { Icon, type IconName } from "./Icon";

/** Nagłówek strony w jednej linii: ikona, tytuł, opis w podpowiedzi ⓘ, stan, akcje (STOP zawsze na końcu akcji). */
export function PageHeader({ icon, title, info, state, actions }: {
    icon: IconName; title: string; info?: string; state?: ReactNode; actions?: ReactNode;
}) {
    return <header className="ph">
        <div className="ph-title"><Icon name={icon} /><h1>{title}</h1>{info && <InfoTip text={info} />}</div>
        {state && <div className="ph-state">{state}</div>}
        {actions && <div className="ph-actions">{actions}</div>}
    </header>;
}

export function InfoTip({ text }: { text: string }) {
    return <span className="info-tip" tabIndex={0} role="note" aria-label={text} data-tip={text}>i</span>;
}
