# Diagnostyka magistrali CAN

`CanBusManager` jest jedynym właścicielem komunikacji CAN. `robot_can_diagnostics`
uruchamia go samodzielnie; w trybie ruchu działa w pluginie `robot_hardware`.
Harmonogram i aktywację opisuje [strategia CAN](can_strategy.md).
Po starcie, gdy `can0` jest dostępny, skanuje adresy standardowego CAN od 1 do
2047. Adres 0 jest rozgłoszeniem i nie jest używany. Każdy adres otrzymuje
komendę identyfikacji `0x40`, a odpowiedź jest akceptowana dopiero po sprawdzeniu
CAN ID, komendy, siedmiobajtowego DLC i sumy kontrolnej. Długość 7 potwierdzono
na napędach z firmware 1.0.9 (np. `40 12 01 00 09 20 7D` dla adresu 1).

Profil odczytu dotyczy MKS SERVO42D CAN z firmware 1.0.9: ENABLE (`0x3A`),
stan pracy (`0xF1`), stall (`0x3E`), pozycja (`0x31`) oraz tryb pracy (`00 82`). Żadna ścieżka
diagnostyczna nie wysyła poleceń ruchu, ENABLE, zmiany trybu, adresu ani
parametrów. Znaczenie ramek pochodzi z [instrukcji Makerbase V1.0.9](https://github.com/makerbase-motor/MKS-SERVO42D-57D/blob/master/User%20Manual/V1.0.9/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.9.pdf).

## Zachowanie i ograniczenia

- Timeout odpowiedzi wynosi domyślnie 20 ms (`can_timeout_ms:=20`) i wymaga
  potwierdzenia pomiarem na docelowym sprzęcie. Skan bez odpowiedzi trwa około
  41 s. Brak odpowiedzi nie oznacza wolnego adresu.
- Wykryte SERVO42D mają odczyt pozycji 50 Hz, ENABLE/stall 10 Hz i publikację
  statusów 5 Hz. Budżet uwzględnia zapytania i odpowiedzi (do 50% CAN).
  Po trzech nieudanych odczytach urządzenie staje się offline,
  a ENABLE jest nieznane; sprawdzenie offline odbywa się raz na sekundę.
- Dane są przechowywane tylko jako ostatnie wartości i agregaty o stałym
  rozmiarze. Skan nie tworzy historii ramek ani bazy danych.
- `BUS-OFF` zatrzymuje wysyłanie. Nie ma automatycznej pętli restartów.
  Przycisk **Reset CAN** przerywa pracę, zamyka socket, restartuje wyłącznie
  `robot-can@can0.service`, otwiera socket i uruchamia nowy skan.
- Odbiór własnych ramek jest wyłączony w socketcie; echo lokalne nie może
  tworzyć pozycji urządzeń.

## Uprawnienie resetu

Węzeł i backend działają jako zwykły użytkownik. Reset wymaga jednorazowego
nadania temu kontu minimalnego uprawnienia:

```bash
sudo ./scripts/install_can_diagnostics_sudoers.sh "$USER"
```

Instalator sprawdza regułę przez `visudo` i zapisuje wyłącznie pozwolenie na
`/usr/bin/systemctl restart robot-can@can0.service`. Brak uprawnienia jest
widoczny w panelu jako błąd resetu; nie daje shellowi ani aplikacji prawa do
innych poleceń administracyjnych.

## Interfejs ROS i HTTP

- Temat `/robot/can/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`), 5 Hz.
- Usługi `std_srvs/Trigger`: `/robot/can/scan`, `/robot/can/reset`.
- `GET /api/v1/can/status`, `POST /api/v1/can/scan`, `POST /api/v1/can/reset`.
- WebSocket `/api/v1/can/ws` wysyła najnowszy snapshot co 200 ms tylko podczas
  otwartej strony Magistrala CAN. Obowiązują te same ograniczenia LAN, Host i
  Origin co dla całego panelu.

SCAN i Reset są blokowane podczas aktywnego sterowania. Przycisk SCAN odrzuca
powtórzenie podczas trwającego skanu. Kernelowe liczniki
interfejsu nie są zerowane przez reset; liczniki sesji aplikacji są zerowane.
