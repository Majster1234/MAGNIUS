# Incydent panelu podczas JOG — 18.10.2026

Po zgłoszeniu użytkownika zabezpieczono nagranie w
`log/incidents/20261018T172508Z/` (poza Git). `capture.json` potwierdza brak
rotacji plików podczas kopiowania; `analysis.json` i `analysis.txt` zawierają
wyniki analizy. Bieżąca rejestracja mogła dalej działać bez nadpisywania kopii.

## Potwierdzony przebieg

Czas lokalny Europe/Prague (UTC+02:00), wszystkie zdarzenia 18.10.2026:

| Czas | Zdarzenie |
| --- | --- |
| 19:24:21,006526303 | START J1, CW, 30 RPM, połączenie panelu nr 4 |
| 19:24:25,522823626 | Ostatnie przyjęte podtrzymanie, sekwencja 57 |
| 19:24:25,538313368 | Zamknięcie WebSocket przy aktywnej osi J1, 15,490 ms po ostatnim przyjętym podtrzymaniu |
| 19:24:25,539237234 | C++ rozpoczyna hamowanie po STOP z obsługi rozłączenia |
| 19:24:25,939164774 | Potwierdzony postój, około 400 ms od STOP |
| 19:24:26,553182030 | Nowe połączenie panelu, nr 5 |
| 19:24:26,559083827 | Panel zgłasza odstęp odbioru statusów 1030,2 ms |

Serwer wysłał na połączeniu nr 4 **172 aktualne statusy**, bez oznaczenia
podłączonych osi jako niedostępnych. Największy odstęp wysyłek wyniósł 56,960 ms,
a wiek stanu ROS przy wysyłaniu 50,334 ms. Brak timeoutów odczytów napędów,
globalnego błędu CAN, przepełnienia RX i utraconych wpisów rejestratora.
Wiek pozycji w zachowanym oknie ROS: J1 maks. 41,832 ms, J2 22,960 ms,
J4 22,850 ms. Wszystkie wartości poniżej lokalnego terminu pozycji 60 ms.

W tym przebiegu zanik wyświetlania dotyczył połączenia panel–API. Nie doszło
do zaniku dostępności wszystkich napędów na CAN. Około sekundy przerwy wynika
z zamknięcia i ponownego zestawienia WebSocket, a nie z samego RTT napędów.

## Wykryty błąd i granica dowodu

API zamykało połączenie przy dwóch kolejnych komendach innych niż STOP,
odebranych w odstępie krótszym niż 20 ms. To błędne założenie dla połączenia
sieciowego: okres wysyłania 50 ms nie zapewnia minimalnego odstępu odbiorów.
Opóźnienie jednego pakietu, a następnie szybki odbiór kolejnego mogły wywołać
zamknięcie i automatyczny STOP. Test dostarczający START i podtrzymania w krótkiej
serii odtworzył błąd: przed poprawką API potwierdzało tylko START i zamykało WS.

Odstęp 15,490 ms do zamknięcia w incydencie pasuje do tego mechanizmu.
**Stara wersja logu nie zapisywała przyczyny odrzucenia ani kodu rozłączenia**,
więc nie dowodzi, jaka dokładnie ramka lub zdarzenie zamknęły tę sesję.
Nie utożsamiamy wykrytego błędu z dowodem każdej możliwej utraty połączenia LAN.

## Poprawka i sprawdzenia

- Limit 20 komend/s z tolerancją krótkiej serii czterech, zamiast minimalnego
  odstępu 20 ms. Nadmierny strumień nadal zamyka sesję i zleca STOP.
- STOP omija limit; sesja/sekwencja i lokalny watchdog C++ 200 ms pozostają.
- Log zawiera teraz powód odrzucenia/zamknięcia oraz kod rozłączenia klienta.
- Panel rozróżnia nieaktualne dane połączenia od niedostępności napędu.
  Ostatnia pozycja ma wyraźną etykietę; nie dopuszcza to ruchu bez świeżych danych.
- Powrót połączenia nie wznawia JOG; wymagane jest nowe naciśnięcie.

Sprawdzenie tej poprawki: budowanie `robot_web_bridge`, 45 testów backendu
(w tym test serii podtrzymań i nadmiernego strumienia), TypeScript/build,
12 Vitest i Chromium — OK. Nie powtarzano testów ruchowych ani całego workspace.
Poprawka wymaga sprawdzenia na urządzeniu użytkownika, na którym wystąpił objaw.
