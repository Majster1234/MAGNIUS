# Teach pendant na telefonie (`#pendant`)

Wdrożony 11.11.2026 na podstawie makiety `temp/sterowanie_mobilne/v5_zezwolenie.html`. Widok JOG robota w przeglądarce
telefonu: tylko orientacja pozioma, pełny ekran, model 3D z limitami osi, drążek JOG TCP / osi, przycisk zezwolenia.

## Otwarcie z telefonu (kod QR)

- PC: górny pasek aplikacji → **📱 Teach pendant** → kod QR z adresem `<network.url>/?fs=1#pendant`
  (`network.url` z `/api/v1/status`, np. `http://192.168.1.69:8080`; `PendantLink.tsx`, koder `qrCode.ts` bez zależności).
- Telefon musi być w sieci LAN panelu (`LanOnly`: podsieć z konfiguracji mostu i nagłówek `Host` = adres panelu).
- `fs=1`: po otwarciu ekran „▶ Uruchom” — jedno dotknięcie włącza pełny ekran i blokuje orientację poziomą
  (`requestFullscreen` + `screen.orientation.lock`). Przeglądarki pozwalają na pełny ekran tylko po geście
  użytkownika, więc kod QR nie może go włączyć bez tego dotknięcia. Po „Zainstaluj aplikację” / „Do ekranu
  początkowego” (manifest `public/pendant.manifest.json`: `display: fullscreen`, `orientation: landscape`,
  `start_url: /?fs=1#pendant`) pendant otwiera się od razu w pełnym ekranie. iPhone: brak Fullscreen API dla stron —
  tylko przez ekran początkowy.
- W pionie cały ekran zasłania nakładka „Obróć telefon poziomo”.

## Sterowanie (`PendantPage.tsx`, logika `pendantJog.ts`)

- Ten sam WebSocket i klienci co sterowanie ręczne: `JogControl` (JOG osi, `space: joint`, prędkość °/s, rewizja
  konfiguracji) i `PhysicalTcpJog` (JOG TCP, `space: tcp`, układ base/tool). Podtrzymanie co 50 ms (TCP w potoku, bez
  czekania na ACK); cisza ACK ≥ 400 ms, stanu ≥ 500 ms lub rozłączenie → STOP. Serwer ucina JOG TCP po 150 ms bez
  podtrzymania. Wszystkie te zatrzymania są **łagodne** (dolot, napędy trzymają pozycję, ENABLE bez zmian) —
  `docs/tcp_jog.md` „Zatrzymania i nadzór”.
- Koniec JOG TCP przez serwer przy trzymanym drążku: po zakłóceniu łączności (KEEPALIVE) pendant wznawia ruch po
  300 ms; po końcu drogi (limit osi, osobliwość, uchyb) pokazuje przyczynę i czeka na powrót drążka do środka.
- Główny wątek telefonu: stan dla pętli sterowania w refie (bez czekania na render), render React ≤ 10 Hz, scena 3D
  ≤ 20 kl./s; znaczniki przerw (`browser_state_gap`, `client_stop`) trafiają do nagrania diagnostyki jak z panelu PC.
- **Zezwolenie (deadman):** ruch tylko przy przytrzymanym przycisku po lewej. Puszczenie, utrata fokusu, ukrycie
  karty, obrót do pionu → zwolnienie i STOP JOG. Drążek bez zezwolenia jest nieaktywny.
- **Drążek:** backend porusza jedną oś przegubu albo jeden kierunek TCP naraz, z prędkością ustaloną przy starcie.
  Pierwszy kierunek wychylenia (> 40 %) wybiera oś pary (TCP: X·Y, Z·Rz, Rx·Ry; osie: J1·J2, J3·J4, J5·J6), zmiana osi
  po powrocie drążka do środka (< 20 %); odwrócenie kierunku = stop i start w przeciwną stronę.
- **Prędkość:** Wolno / Średnio / Szybko = 10 / 30 / 60 % limitu JOG (osie: `jog_max_velocity_degrees_s`, nie mniej
  niż `jog_min_velocity_degrees_s`; TCP: `tcp_jog.max_linear_velocity` / `max_angular_velocity`).
- **STOP** (zatrzask, „▶ ODBLOKUJ”): zwolnienie zezwolenia, STOP JOG TCP i osi, `POST /api/v1/program-run/stop`.
- Napędy z ENABLE=0: przycisk „⚡ Załącz napędy” (`POST /api/v1/manual/enable-all`, z potwierdzeniem).
- Komunikaty: brak połączenia, nieaktualny stan, awaria, odrzucenie komendy, niegotowy JOG TCP, najbliższy limit osi.

## Model 3D i limity

`programPreviewScene.ts` (geometria aktywnej konfiguracji) w bieżących kątach przegubów; wskaźniki limitów
`previewLimits` z żywego stanu osi (≤ 15° ostrzeżenie, ≤ 5° niebezpiecznie, przekroczenie) jak w sterowaniu ręcznym
(`createLimitGlowsFor`); paski J1–J6 przy STOP.

## Testy

- `qrCode.test.ts`, `pendantJog.test.ts` (vitest). Koder QR sprawdzony bit w bit z niezależną implementacją
  (`qrcode-terminal/vendor/QRCode` z npm): 144 macierze, wersje 1–26, poziomy L/M, wszystkie maski.
- Playwright z atrapą WebSocketu (11.11.2026): brak komend bez zezwolenia; JOG TCP X+ (`velocity` 0,01 m/s przy
  „Wolno”, podtrzymanie, stop po powrocie drążka); JOG J5+ (1,8 °/s, stop po puszczeniu zezwolenia); STOP z zatrzaskiem
  i `program-run/stop`; przycisk ENABLE; limity na modelu; nakładka w pionie; pełny ekran po „▶ Uruchom”.

## Test długiej sesji na robocie (RUCH)

`python3 scripts/pendant_session.py --minutes 10 [--cpu 4] [--seed 1]` — zerowanie MKS, ENABLE, potem
`web/test/pendant-session.hw.cjs`: prawdziwa strona `#pendant` w Chromium jak telefon (dotyk wielopunktowy CDP:
zezwolenie + drążek, CPU spowolnione 4×), JOG osi do pozy startowej 0/−30/40/0/45/0°, a potem przez zadany czas
ruchy JOG TCP tam i z powrotem (X/Y/Z/RX/RY/RZ, 0,5–2 s, Wolno/Średnio/Szybko), powrót do pozy przy dryfie > 25°.
Obserwator stanu przez osobny WebSocket; po awarii skrypt załącza napędy przyciskiem pendanta i kontynuuje.
Wynik: `log/pendant-sessions/<czas>/summary.json` (przytrzymania, brak startu, przerwania przed puszczeniem drążka,
awarie) i `session.jsonl`; skrypt wypisuje nowe migawki awarii. Ruch fizyczny: stała zgoda tylko przy rozsprzęglonej
mechanice (AGENTS.md §5), inaczej karta i zgoda operatora.
