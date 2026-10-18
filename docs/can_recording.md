# Nagrywanie diagnostyki ruchu ręcznego

Po zamknięciu poprzedniej instancji przy potwierdzonym postoju uruchom:

```bash
./scripts/start_web_status.sh --record
```

Odśwież `#manual`: pojawi się **Nagrywanie diagnostyki**. Sam start nie uruchamia
silników. Po przeładowaniu procesu przygotuj wybrane osie przyciskiem **Załącz**.
Nagranie nie dodaje zapytań CAN i korzysta z jednego właściciela transportu.

Pliki w `log/manual-can/` są pomijane przez Git. Opcjonalnie ustaw `ROBOT_TRACE_DIR`.
Kolejne uruchomienie używa tych samych plików; istotny incydent zachowaj wcześniej.
`session.json` identyfikuje bieżące nagranie, aby nie mieszać pozostałości starszych sesji.

```bash
python3 scripts/analyze_can_recording.py log/manual-can
python3 scripts/analyze_can_recording.py log/manual-can --json > /tmp/can-analysis.json
```

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
| `io_loop_gap`, `io_tick_slow` | Odstęp startów pętli >5 ms i cykl >2 ms; w `io_loop_gap` `request_ns/deadline_ns` oznaczają przebudzenie/uzyskanie mutexu |
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
ujawnia pominięcia przy przepełnieniu. CAN: 4 × 16 MiB, backend: 4 × 8 MiB,
razem najwyżej 96 MiB JSONL; drobne raporty/sesja są osobne. Błąd zapisu wyłącza
rejestrator i nie zatrzymuje aplikacji. Zapis nie przechowuje sesji uprawniających
do JOG, nagłówków HTTP ani zmiennych środowiskowych.

Po `bus_fault` CAN nagrywa jeszcze 5 sekund i zamraża zapis; backend robi to po
otrzymaniu `fault`. Pliki kończy `recording_finished`. Nie kasuje to błędu ani nie
wznawia ruchu. Bez błędu działa rotacja; `Ctrl+C` opróżnia bufory i kończy aplikację.
Nagłe zabicie procesu lub zanik zasilania może utracić ostatnie ~100 ms.

Istniejąca ochrona po błędzie operacji ręcznej celowo wstrzymuje TX całej
magistrali, aby diagnostyka nie maskowała ochrony komunikacyjnej firmware.
Niedostępność wszystkich osi **po** `bus_fault` nie dowodzi zatkania CAN.
Analizator pokazuje ±500 ms wokół błędu: należy znaleźć poprzedzający timeout,
spóźnioną odpowiedź, lukę pętli lub brak podtrzymania. Braki odpowiedzi SCAN
są raportowane osobno od timeoutów odczytów.

Testy automatyczne obejmują rotację, zegary, równoczesnych producentów,
zamrożenie, analizator i znaczniki WS. Wyniki sprzętowe są osobne;
brak odtworzenia objawu nie potwierdza naprawy.
[Wyniki prób po 15 s w obu kierunkach](can_recording_results.md).
[Analiza zgłoszonego incydentu i poprawka API](can_incident_2026-10-18.md).
