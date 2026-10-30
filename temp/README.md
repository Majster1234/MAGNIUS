# temp/ — propozycje wyglądu GUI

Samodzielne makiety HTML (bez skryptów, zewnętrznych zasobów i połączeń z API/ROS/CAN). Wszystkie wartości są przykładowe. Otwierać bezpośrednio w przeglądarce. Nie są częścią `web/` ani wdrożenia.

## Strona „Architektura” — analiza stanu obecnego

Źródło: `web/src/ArchitecturePage.tsx` (statyczna lista 6 etapów, brak danych na żywo).

Nieaktualne lub brakujące informacje:
- Karta SocketCAN podaje `500 kbit/s`; konfiguracja i AGENTS.md mówią o 1 Mbit/s.
- Opis „LIN oraz ciągłe TCP pozostają podglądem” jest nieaktualny (LIN, JOG TCP i tryb `synchronized` są już wykonywane).
- Brak stanu procesów, wersji buildu, aktywnej nakładki `install/*`, obciążenia CAN i bieżących problemów.

## Co można pokazać (dane już dostępne lub łatwe do dodania)

| Sekcja | Dostępne dziś (`/robot/system_status`, CAN snapshot) | Do dodania w backendzie |
| --- | --- | --- |
| System | CPU, RAM, czas pracy (`host`); stan can0, bitrate, RX/TX, błędy, bus-off (`can`) | stan usług systemd, obciążenie CAN w % względem limitu 60%, temperatura |
| ROS | stan `ros`, `controllers`, `drives`; urządzenia CAN ID 1–6 | lista węzłów i częstotliwości tematów, aktywna nakładka `install/*`, tryb PTP/LIN, stan odzyskiwania SYNC i maska awarii, czasy odczytu vs timeout |
| Backend Python | stan `api`, wiek snapshotu, adres sieciowy | PID, CPU/RAM procesu, liczba klientów WS, żądania/min, błędy 5xx |
| Frontend | — | hash/data buildu (wstrzyknięte przy `npm run build`), RTT WS tej karty, lista otwartych sesji |
| Aktywne problemy | komunikaty `message` komponentów o stanie `warn`/`error` | wspólny rejestr problemów z czasem wystąpienia i statusem aktywny/rozwiązany (możliwe połączenie z Centrum komunikatów) |

## Warianty

1. `architektura_v1_warstwy.html` — cztery warstwy w pionie (Frontend → Backend → ROS → System), pasek aktywnych problemów na górze. Najbliższy obecnemu stylowi, czytelny dla operatora.
2. `architektura_v2_mapa_przeplywu.html` — diagram SVG procesów i połączeń z kolorami świeżości, panel problemów i szczegółów wybranego węzła. Najlepiej pokazuje, gdzie w łańcuchu jest problem.
3. `architektura_v3_pulpit.html` — cztery ćwiartki z KPI i mini-wykresami, tabele procesów i oś czasu problemów z filtrami. Dobry jako ekran ciągłej obserwacji.
4. `architektura_v4_konsola.html` — **wybrany przez operatora.** Gęsta tabela w stylu `htop` z grupami sekcji, wskaźnikami i tabelą napędów. Czcionka jak w aplikacji (Inter, cyfry tabelaryczne), nagłówek `page-title` i animowany pasek przepływu Frontend → Backend → ROS → can0 → MKS.

## Znaczniki statusu w ramie aplikacji (widoczne w każdym module)

Animacja ma być dowodem pracy: w implementacji takt wyzwala odebrany snapshot, nie stały zegar. Brak danych = brak ruchu. Wszystkie warianty respektują `prefers-reduced-motion`.

- `status_ramy_a_chipy.html` — A: pięć chipów SYS/ROS/API/WEB/CAN w pasku górnym (zastępują „Połączono z backendem”), pulsująca kropka i linia pracy pod paskiem.
- `status_ramy_b_lancuch.html` — B: pionowy łańcuch systemów w dole paska bocznego z pakietami spływającymi po łączach; wersja zwinięta 68 px.
- `status_ramy_c_monitor.html` — C: „monitor” na środku paska górnego: linia EKG, diody systemów i 6 kwadratów napędów J1–J6.

Kolejne propozycje trafiają do podfolderów (np. `sterowanie_reczne/`), każdy z własnym `README.md`.
