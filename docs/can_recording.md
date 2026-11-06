# Nagrywanie diagnostyki ruchu ręcznego

Po zamknięciu poprzedniej instancji przy potwierdzonym postoju uruchom:

```bash
./scripts/start_web_status.sh --record
```

Odśwież `#manual`: pojawi się **Nagrywanie diagnostyki**. Sam start nie uruchamia
silników. Po przeładowaniu procesu przygotuj wybrane osie przyciskiem **Załącz**.
Nagranie nie dodaje zapytań CAN i korzysta z jednego właściciela transportu.

Pliki w `log/manual-can/` są pomijane przez Git. Opcjonalnie ustaw `ROBOT_TRACE_DIR`.
Przy starcie pliki poprzedniej sesji trafiają do `previous/` (starsza `previous/` jest usuwana).
`session.json` identyfikuje bieżące nagranie, aby nie mieszać pozostałości starszych sesji.

## Migawki awarii (`faults/faults.rfs`)

Wszystkie awarie trafiają do **jednego, zwartego pliku** `log/manual-can/faults/faults.rfs`: okno
**10 s przed i 5 s po** awarii ze wszystkich źródeł. Plik służy diagnostyce (analizie przez asystenta), nie
do czytania wprost: kodowanie jest bezstratne, a migawka 15 s zajmuje ok. 0,4 MiB zamiast 12 MiB JSONL (28×,
pomiar 05.11 na migawce 22:37:07; zapis ok. 2 s w osobnym procesie).

```bash
cd ros2_ws/src/robot_web_bridge   # albo z załadowanym overlay ROS
python3 -m robot_web_bridge.fault_snapshot --directory ~/Desktop/Projekty/log/manual-can --list
python3 -m robot_web_bridge.fault_snapshot --directory ~/Desktop/Projekty/log/manual-can --dump last --output /tmp/fault.jsonl
python3 -m robot_web_bridge.fault_snapshot --directory ~/Desktop/Projekty/log/manual-can --dump 20261105-223707-711
```

`--dump` odtwarza JSONL: nagłówek, potem oryginalne wiersze (z dodanym polem `time`, czas lokalny z µs).

| Źródło (`source`) | Zawartość |
| --- | --- |
| `can_cpp` | Wszystkie ramki TX/RX, zapytania, timeouty, punkty F5, zdarzenia JOG/programu/odzyskiwania |
| `bridge` | Stan ROS w moście, polecenia i znaczniki panelu (KEEPALIVE, przerwy przeglądarki), usługi ROS, CPU/RAM |
| `rosout` | Komunikaty logów wszystkich węzłów ROS (poziom, węzeł, plik, linia) |
| `journal` | Dziennik usługi systemd: stdout/stderr procesów, w tym wyjątki Pythona i padnięcia procesów |
| `kernel` | Dziennik jądra: `gs_usb`, rozłączenia USB, błędy SocketCAN |

Nagłówek: `snapshot`, `time`, `duration_s`, `rows`, wyzwalacze (źródło, zdarzenie, opis), okno czasowe, liczby
wierszy według źródła, użyte segmenty, aktywna konfiguracja robota (`active.json`, `system_settings.json`),
treść pliku parametrów wykonania i `session.json`.

- **Format** (`robot_web_bridge/fault_codec.py`): rekordy `RFS1` + nieskompresowane metadane (lista i retencja
  bez dekompresji) + dane xz. Wiersze jako krotki `[Δutc_ns, schemat, wartości…]`; `mono_ns` jako offset
  od UTC, `sequence` jako różnica, `request/deadline_ns` względem `mono_ns`, `kernel_utc_ns` względem `utc_ns`,
  stały `recording_id` raz w nagłówku. Test zgodności: dekodowanie migawki 22:37 dało identyczne 27 917 wierszy.
- **Pojemność:** zawsze co najmniej **900 s okien awarii** (60 migawek po 15 s, ok. 25 MiB); najstarsza
  migawka jest usuwana dopiero, gdy pozostałe nadal obejmują 900 s, albo przy przekroczeniu 256 MiB (najnowsza
  zostaje zawsze). Ustawienia: `ROBOT_FAULT_KEEP_S`, `ROBOT_FAULT_MAX_MIB`, `ROBOT_FAULT_PRE_S`, `ROBOT_FAULT_POST_S`.
- **Zapis:** dopisanie rekordu z `fsync`; przy usuwaniu najstarszych zachowany zakres jest kopiowany do
  `faults.rfs.tmp` i podmieniany atomowo. Niepełny rekord na końcu (zanik zasilania) jest pomijany i usuwany
  przy następnym zapisie.
- **Wyzwalacze:** zdarzenia C++ `bus_fault`, `manual_fault`, `recovery_failed`, `socket_transport_lost`,
  `critical_enable_off_failed`, `tcp_jog_failed`, a także narastające zbocze stanu awarii w moście. Wątek
  zapisu tworzy `faults/trigger-*.json` dopiero po zapisaniu zdarzenia na dysk; wątek CAN nie dotyka dysku.
  Wyzwalacze odległe o nie więcej niż 5 s od pierwszego trafiają do jednej migawki. Migawka powstaje ok. 6,5 s
  po awarii.
- Migawki zbiera osobny proces `fault_snapshot` (`robot_web_bridge`, nice 10, uruchamiany przez launch tylko
  przy `--record`, automatycznie wznawiany). Przy zamykaniu zapisuje zaległe wyzwalacze z dostępnymi danymi.
  Wyzwalacz z poprzedniej sesji jest szukany także w `previous/`. Dawne pliki JSONL (`faults.jsonl`,
  `fault-*.jsonl`) są przy starcie przenoszone do `faults.rfs`.
- Komunikaty `/rosout` sprzed startu procesu migawek nie są dostępne (bufor w pamięci, ostatnie ~75 s).
  Dziennik usługi istnieje tylko przy uruchomieniu przez systemd.

## Zegary i zdarzenia

Każdy JSONL ma `mono_ns`, `utc_ns`, PID i nazwę zdarzenia. Liczby zapisujemy
w nanosekundach; nie oznacza to nanosekundowej dokładności USB/Linuksa/napędu.
Opóźnienia na jednym komputerze wyliczamy z `CLOCK_MONOTONIC`, niezależnego od NTP.

| Zdarzenie | Znaczenie |
| --- | --- |
| `can_tx`, `can_rx` | CAN ID, opcode i bajty w hex; TX to przyjęcie przez SocketCAN, nie fizyczny moment nadania |
| `can_rx.kernel_utc_ns` | Programowy timestamp RX jądra (`SO_TIMESTAMPNS`); różnica z UTC odbioru pokazuje oczekiwanie na odczyt socketu |
| `request` | `request_ns`, `deadline_ns`, adres/opcode; `flags=1` oznacza sekwencję ręczną |
| `reply_ok/late/invalid` | Dopasowanie do tego samego adresu/opcode/`request_ns`, RTT w `duration_ns` |
| `request_timeout`, `reply_unmatched` | Przekroczony termin lub odpowiedź bez pasującego oczekiwania |
| `socket_rx_overflow`, `can_error_frame`, `can_send_error` | Utrata ramek w kolejce RX, błędy magistrali/wysyłki |
| `io_loop_gap`, `io_tick_slow` | Odstęp startów pętli >5 ms i cykl >2 ms; w `io_loop_gap` `request_ns/deadline_ns` oznaczają przebudzenie/uzyskanie mutexu, a `detail` spóźnienie przebudzenia względem planowanego terminu (pętla co 1 ms) i czas oczekiwania na mutex |
| `manual_feedback` | Co 20 ms podczas operacji: wiek pozycji, prędkości, ENABLE i stall; `duration_ns` to wiek F1, `request_ns` ostatnie podtrzymanie |
| `manual_command/stop/fault`, `bus_fault` | Przyjęcie/odrzucenie polecenia, przyczyna hamowania/globalnego błędu |
| `ros_publish`, `ros_axis` | Publikacja 20 Hz i wiek pomiarów; `request_ns` identyfikuje stan ROS, `ros_publish.duration_ns` to odstęp publikacji |
| `ros_state_received` | Ten sam `state_monotonic_ns` i opóźnienie dotarcia do callbacku backendu |
| `ros_service_request/result` | `issued_ns` odpowiada `manual_command.request_ns` |
| `ws_state_sent`, `ws_command_received`, `ws_error/close` | Wysyłka stanu, odbiór poleceń, rozłączenia; `connection` odróżnia klientów |
| `browser_state_received/gap`, `browser_client_stop` | Odbiór statusu i powód STOP w panelu, identyfikator stanu ROS |
| `system_status`, `can_summary` | Istniejące pomiary CPU/RAM, interfejsu i statystyk; bez dodatkowego odpytywania sprzętu |

`ros_axis.flags`: bity 0 dostępność, 1 znana pozycja, 2 znany ENABLE,
3 ENABLE=1, 4 dopuszczenie JOG, 5 globalny błąd. `manual_feedback.flags`:
1 przygotowanie, 2 start, 3 JOG, 4 hamowanie.
`browser_client_stop.reason`: 0 puszczenie/zdarzenie zatrzymujące, 1 timeout ACK,
2 timeout statusu, 3 zamknięty socket, 4 nieaktualny stan, 5 odrzucona komenda.

`client_epoch_ms/client_perf_ms` to zegary przeglądarki. Dla innego komputera nie
odejmujemy ich od zegara serwera bez synchronizacji: porównujemy odstępy klienta
i identyfikatory stanu. Znaczniki panelu mają osobny limit częstotliwości.

## Ograniczenia i interpretacja

C++ ma kolejkę MPSC 4096 zdarzeń i osobny wątek zapisu. Producent nie czeka
na dysk ani mutex. Backend ma kolejkę 512 wpisów i własny wątek. `dropped_total`
ujawnia pominięcia przy przepełnieniu. Pętla ciągła: CAN `ROBOT_TRACE_CAN_MIB` (domyślnie 2048 MiB =
128 × 16 MiB, ok. 35 min ciągłego ruchu przy ~1 MiB/s), backend `ROBOT_TRACE_BRIDGE_MIB` (domyślnie
1024 MiB = 128 × 8 MiB). Segmenty `can-N.jsonl`/`bridge-N.jsonl` nadpisywane po kolei; kolejność wg czasu
modyfikacji lub `utc_ns`. Drobne raporty/sesja są osobne. Błąd zapisu wyłącza
rejestrator i nie zatrzymuje aplikacji. Zapis nie przechowuje sesji uprawniających
do JOG, nagłówków HTTP ani zmiennych środowiskowych.

Od 05.11.2026 nagranie **nie zamraża się po awarii** (wcześniej 5 s po `bus_fault` zapis kończył się na
stałe). Incydent zachowuje migawka w `faults/`. `recording_finished` kończy plik tylko przy zamknięciu
procesu; `Ctrl+C` opróżnia bufory i kończy aplikację.
Nagłe zabicie procesu lub zanik zasilania może utracić ostatnie ~100 ms.

Istniejąca ochrona po błędzie operacji ręcznej celowo wstrzymuje TX całej
magistrali, aby diagnostyka nie maskowała ochrony komunikacyjnej firmware.
Niedostępność wszystkich osi **po** `bus_fault` nie dowodzi zatkania CAN.
Analizator pokazuje ±500 ms wokół błędu: należy znaleźć poprzedzający timeout,
spóźnioną odpowiedź, lukę pętli lub brak podtrzymania. Braki odpowiedzi SCAN
są raportowane osobno od timeoutów odczytów.

Testy automatyczne obejmują rotację i liczbę segmentów, zegary, równoczesnych producentów,
wyzwalacze awarii, scalanie migawek, analizator i znaczniki WS. Wyniki sprzętowe są osobne;
brak odtworzenia objawu nie potwierdza naprawy.
[Wyniki prób po 15 s w obu kierunkach](can_recording_results.md).
[Analiza zgłoszonego incydentu i poprawka API](can_incident_2026-10-18.md).
