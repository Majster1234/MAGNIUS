# Strona „Architektura” i monitor pracy

Obie funkcje są tylko do odczytu: nie wysyłają ramek CAN, poleceń ROS ani zmian konfiguracji.

## Monitor pracy (pasek górny, każdy widok)

| Element | Źródło | Takt kanału pikselowego |
| --- | --- | --- |
| SYS | `host` + `can` z `/robot/system_status` (monitor 1 Hz) | nowy `host.sampled_at` |
| ROS | `ros`, `controllers` + podsumowanie diagnostyki CAN | nowy `updated_at` diagnostyki CAN (5 Hz) |
| API | strumień `/api/v1/ws` | każdy odebrany snapshot |
| WEB | oba strumienie tej karty (status i CAN) | każda wiadomość z dowolnego strumienia |
| J1–J6 | urządzenia CAN ID 1–6 z `/api/v1/can/ws` | — |

Stany: OK (zielony), ostrzeżenie (bursztyn), brak danych (szary, pusty pasek), awaria (czerwony, miga). Kolejność ważności: OK < ostrzeżenie < brak danych < awaria. Kontrolery `not_configured` nie są traktowane jako awaria. Szczegóły i nazwa podsystemu są wyłącznie w dymku (najechanie, Tab, dotknięcie). Szerokość monitora nie zależy od stanu. Brak danych = kanał stoi. Przy `prefers-reduced-motion` kanały się nie przesuwają, kolory nadal się zmieniają.

## Strona „Architektura”

- Wiersze tabeli: System (host, usługi systemd, `can0`), ROS 2 (procesy, diagnostyka CAN, monitor, kontrolery, graf węzłów), Backend Python (`robot_web_bridge`, klienci WebSocket, snapshot), Frontend (build, ta karta).
- Wskaźniki: CPU, RAM, planowane obciążenie CAN względem `total_bus_planning_limit_percent` oraz maks. RTT względem `response_timeout_ms` z diagnostyki CAN.
- „Aktywne problemy” to wszystkie wiersze w stanie ostrzeżenia, braku danych lub awarii.
- Wersja panelu (`commit[+dirty]` i czas buildu) jest wstrzykiwana przez `web/vite.config.ts`.

## `GET /api/v1/system/overview`

Moduł `robot_web_bridge/system_overview.py`, wynik buforowany 1 s, liczony poza pętlą zdarzeń:

- `ros.processes`: procesy z `--ros-args` (psutil): nazwa z `__node:=` lub pliku wykonywalnego, pakiet i nakładka z ścieżki, CPU% (pierwszy pomiar `null`), RSS, wątki, czas pracy;
- `ros.nodes`: graf węzłów z `get_node_names_and_namespaces()` mostu;
- `services`: `systemctl show` dla parametru `overview_units` (domyślnie `robot-web-status.service`, `robot-can@can0.service`);
- `backend`: proces mostu i liczba otwartych WebSocketów `status`/`can`/`manual`.

Strona odpytuje endpoint co 2 s, tylko gdy jest otwarta. Stary backend zwraca 404, a strona pokazuje wtedy „wymaga restartu usługi” i nadal korzysta z danych statusu i CAN.
