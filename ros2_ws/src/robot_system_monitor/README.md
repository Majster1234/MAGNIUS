# robot_system_monitor

Węzeł Python `monitor` publikuje `/robot/system_status`
(`diagnostic_msgs/msg/DiagnosticArray`) raz na sekundę. Zbiera CPU, RAM, uptime,
stan CAN oraz listę kontrolerów. Błąd jednego źródła nie przerywa pozostałych odczytów.

Parametry: `can_interface=can0`,
`controller_service=/controller_manager/list_controllers`.
Żądanie do kontrolerów jest asynchroniczne i ma limit 2 s. Monitor nie aktywuje
kontrolerów, nie otwiera socketu CAN i nie wysyła ramek. `ip -json -details
-statistics link show dev can0` odczytuje Netlink; proces ma limit 0,75 s.

Statusy: `host`, `can`, `controllers`, `drives`. Wartości `KeyValue` są kodowane
jako JSON, w tym `sampled_at` w UTC. Brak integracji napędów jest jawnym stanem,
a aktywny interfejs CAN nie potwierdza komunikacji z napędami.

Uruchomienie i testy: [docs/web_status.md](../../../docs/web_status.md).
