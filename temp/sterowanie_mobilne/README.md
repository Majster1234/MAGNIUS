# temp/sterowanie_mobilne — propozycje widoku sterowania robotem w przeglądarce telefonu

Makieta widoku mobilnego (10.11.2026; z pięciu propozycji operator wybrał v5, 11.11.2026): **tylko orientacja pozioma**, najlepiej pełny ekran, model 3D ze wskaźnikami
limitów osi i drążek ruchu TCP z przełączeniem na osie (J1–J6). Dane przykładowe, **bez połączenia z API/ROS/CAN** —
drążki sterują symulacją w przeglądarce. Wdrożenie wybranej propozycji to osobne zadanie w `web/src`.

**Odstępstwo od reguły „bez skryptów”** (`temp/README.md`): widok 3D i symulacja wymagają JavaScriptu. Pliki ładują
three.js 0.164.1 (ta sama wersja co `web/`) z `unpkg.com` przez importmap. Bez internetu widok 3D pokazuje komunikat,
a drążki, odczyty i wskaźniki zakresów działają dalej. Każdy plik jest samodzielny (wspólny rdzeń wklejony w każdy).

## Jak oglądać

- Telefon: otworzyć plik (np. przez udostępnienie katalogu lub serwer `python3 -m http.server` w `temp/`), obrócić
  poziomo, przycisk pełnego ekranu (ikona narożników) — przeglądarki z Fullscreen API dodatkowo blokują orientację
  poziomą (`screen.orientation.lock`). iOS (iPhone) nie ma pełnego ekranu dla stron: „Udostępnij → Do ekranu
  początkowego” (meta `apple-mobile-web-app-capable`).
- Komputer: narzędzia deweloperskie → tryb urządzenia (np. 844×390, dotyk). W pionie widać nakładkę „Obróć telefon”.

## Wspólne dla wszystkich wariantów

- **Orientacja:** w pionie pełnoekranowa nakładka „Obróć telefon poziomo” z przyciskiem „Pełny ekran — poziomo”;
  `user-scalable=no`, bez przewijania i gestów strony na elementach sterujących (`touch-action:none`), obszary
  bezpieczne (`viewport-fit=cover`, `env(safe-area-inset-*)`).
- **Model 3D:** geometria demonstracyjna robota (`configurationData.ts`: `demoTranslations`, `jointAxes`, TCP 80 mm),
  obrót/powiększenie palcami (OrbitControls), ⟲ widok domyślny, ślad TCP. **Limity jak w sterowaniu ręcznym**
  (`previewLimits` / `previewLimitGlow`): łuk + poświata na granicy zakresu przegubu, etykieta „J5 + LIMIT 4°” /
  „LIMIT!”, ostrzeżenie ≤ 15° (żółte), niebezpiecznie ≤ 5° i na limicie (czerwone, pulsuje), czerwony przegub.
  Dla sterowanej osi — sektor dopuszczalnego zakresu z czerwonymi krańcami i wskazówką kąta.
- **Drążek:** przytrzymaj i przechyl (hold-to-run) — puszczenie = natychmiastowe zero; strefa martwa 12 %, odpowiedź
  progresywna (precyzja przy małym wychyleniu). **TCP:** układ Baza/Narzędzie, Przesuw (X/Y/Z) lub Obrót (Rx/Ry/Rz);
  **Osie:** J1–J6. Prędkości 100 %: 100 mm/s, 30°/s TCP, 18°/s osi (jak `jog_max_velocity_degrees_s`).
- **Symulacja JOG TCP:** odwrotna kinematyka numeryczna (DLS) z bieżącej pozy; oś zatrzymuje się na limicie;
  ostrzeżenie „blisko osobliwości nadgarstka (J5 ≈ 0)”. Zakresy osi jak w aktywnej konfiguracji
  (J1 ±171°, J2 −200…30°, J3 −196…57°, J4 ±180°, J5 ±100°, J6 ±180°).
- **STOP** w każdym wariancie (w makiecie blokuje drążki do ponownego dotknięcia), pasek ostrzeżenia ## Wybór i wdrożenie

- 10.11.2026: 5 wariantów (panel boczny, dwa drążki, lista osi, tarcza z pierścieniem, przycisk zezwolenia).
- 11.11.2026: operator wybrał **`v5_zezwolenie.html`** (przycisk zezwolenia lewym kciukiem, drążek prawym, pary osi,
  prędkość Wolno/Średnio/Szybko); pozostałe usunięte. **Wdrożone** jako `web/src/PendantPage.tsx` (trasa `#pendant`)
  z kodem QR w górnym pasku aplikacji — opis: `docs/teach_pendant.md`. Makieta zostaje jako wzorzec wyglądu.

dkość jako Wolno/Średnio/Szybko (10/30/60 %) |

## Uwagi do wdrożenia (poza makietą)

- Ruch: istniejące API JOG (`/api/v1/manual/ws` — JOG osi; JOG TCP `tcp_jog` z układem base/tool) — drążek wysyła
  prędkość cyklicznie z podtrzymaniem (watchdog), brak ramek = zatrzymanie; puszczenie, utrata fokusu, obrót do pionu,
  zamknięcie karty i rozłączenie sieci = STOP.
- Ruch fizyczny z telefonu wymaga decyzji bezpieczeństwa: przycisk zezwolenia (v5) lub inne potwierdzenie obecności,
  ograniczenie prędkości z telefonu, jeden klient sterujący naraz (właściciel ruchu jak w `motion.claim`).
- Limity i model: `previewLimits.ts`, `previewLimitGlow.ts` (`createLimitGlowsFor`) i podgląd programu
  (`programPreviewScene.ts`) można użyć bez zmian.
