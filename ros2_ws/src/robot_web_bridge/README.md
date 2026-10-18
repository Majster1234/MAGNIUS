# robot_web_bridge

Backend Python `bridge`: FastAPI, Uvicorn i jeden executor `rclpy` w osobnym
wątku. Odbiera `/robot/system_status`, przechowuje ostatni zestaw pomiarów
i udostępnia stronę oraz API z jednego adresu. Liczba przeglądarek nie zmienia
częstotliwości pomiarów ani liczby węzłów ROS.

Parametry: `lan_interface=enp2s0`, `lan_subnet=192.168.1.0/24`, `port=8080` oraz
opcjonalny `static_dir`. Domyślnie zasoby panelu pochodzą z katalogu `web/`
w zainstalowanym pakiecie. Zbuduj frontend **przed** `colcon build`.

- `GET /healthz`: żywotność API, niezależna od monitora ROS.
- `GET /api/v1/status`: snapshot w formacie JSON.
- `/api/v1/ws`: snapshot co sekundę; wolny odbiorca jest rozłączany.

Snapshot zawiera `schema_version=1`, `updated_at`, `age_seconds`, `fresh`,
`network` i `components` (`api`, `ros`, `host`, `can`, `controllers`, `drives`).
Komponent ma `id`, `state`, `message`, `sampled_at` i `values`.
Stany: `ok`, `warn`, `error`, `stale`, `not_configured`.
Po 3 s bez wiadomości ROS dane monitora są `stale`; zachowane wartości są
ostatnim odczytem, a nie bieżącym pomiarem. API nadal może odpowiadać poprawnie.

Nasłuch obejmuje konkretny adres LAN. Polityka ASGI sprawdza adres klienta,
Host i Origin także przed rozpoczęciem WebSocket. Nagłówki proxy są ignorowane.
Brak kont i CORS. API ręczne przekazuje żądania do C++ przez ROS; Python
nie posiada transportu CAN ani pętli ruchu. Opis usług i WebSocket:
[sterowanie ręczne](../../../docs/manual_control.md).

Uruchomienie i testy: [docs/web_status.md](../../../docs/web_status.md).
