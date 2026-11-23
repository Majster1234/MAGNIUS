// Animacje interfejsu: włączenie/wyłączenie (Konfiguracja → Ustawienia systemowe → „Wyłącz animacje interfejsu”).
// Ustawienie tej przeglądarki (localStorage), bez zmiany rewizji ustawień systemowych ROS. Wyłączenie ustawia
// <html data-animations="off"> — reguła w style.css wyłącza wszystkie animacje i przejścia CSS; kod JS może pytać animationsEnabled().
import { useEffect, useState } from "react";

const KEY = "robot.ui.animations";
const EVENT = "robot-animations";

export function animationsEnabled(): boolean {
    try { return localStorage.getItem(KEY) !== "off"; } catch { return true; }
}
export function applyAnimations(enabled = animationsEnabled()) {
    document.documentElement.dataset.animations = enabled ? "on" : "off";
}
export function setAnimationsEnabled(enabled: boolean) {
    try { localStorage.setItem(KEY, enabled ? "on" : "off"); } catch { /* bez pamięci przeglądarki — tylko do odświeżenia */ }
    applyAnimations(enabled);
    window.dispatchEvent(new Event(EVENT));
}
// Start aplikacji: zastosuj zapisane ustawienie i nasłuchuj zmian z innych kart.
export function initAnimations() {
    applyAnimations();
    window.addEventListener("storage", event => { if (event.key === KEY) { applyAnimations(); window.dispatchEvent(new Event(EVENT)); } });
}
export function useAnimationsEnabled(): boolean {
    const [enabled, setEnabled] = useState(animationsEnabled);
    useEffect(() => { const sync = () => setEnabled(animationsEnabled()); window.addEventListener(EVENT, sync); return () => window.removeEventListener(EVENT, sync); }, []);
    return enabled;
}
