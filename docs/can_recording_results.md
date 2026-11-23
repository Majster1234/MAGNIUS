# Próby nagrywania i JOG — 2026-10-18

Uruchomiono rejestrację CAN/ROS/API i rzeczywistego panelu Chromium na komputerze
robota. Nie zmieniano terminów ruchu ani ochrony firmware. Przygotowano dostępne
J1/CAN1, J2/CAN2 i J4/CAN4; J3/J5/J6 nie odpowiadają i nie otrzymały ruchu.
Zgodnie z rozszerzoną zgodą użytkownika każdą dostępną oś sprawdzono osobno
w obu kierunkach po 15 s, najpierw przy 5, potem 30 obr./min. Bez zerowania osi.

| Oś | 5 RPM: CW / CCW [s] | 30 RPM: CW / CCW [s] | Przemieszczenie CW / CCW przy 30 RPM [°] |
| --- | --- | --- | --- |
| J1 | 15,015 / 15,011 | 15,022 / 15,020 | +2707,800 / −2708,086 |
| J2 | 15,016 / 15,012 | 15,014 / 15,006 | +2698,748 / −2697,034 |
| J4 | 15,012 / 15,008 | 15,022 / 15,009 | +2701,011 / −2699,275 |

Wszystkie 12 przejazdów zakończono puszczeniem przycisku, zadaniem 0 i pomiarowym
potwierdzeniem postoju. W obserwowanych statusach nie pojawiła się niedostępność
podłączonych osi. Po próbach J1/J2/J4 mają ENABLE=0; CAN pozostaje ERROR-ACTIVE,
500000 bit/s, błędy RX/TX i BUS-OFF wynoszą 0.

**Objawu użytkownika nie odtworzono. To nie jest potwierdzenie naprawy.**
W krótkiej próbie poprzedzającej długie testy wystąpiło odrzucenie START J2,
po którym panel wysłał STOP z powodem `ack_rejected`; nie było wtedy utraty
dostępności osi ani globalnego błędu. Nie utożsamiamy tego z opisanym objawem.

W analizowanym oknie nagrania z kolejką MPSC: brak timeoutów odczytów napędów,
globalnego błędu, przepełnienia RX i pominięć rejestratora. Przykładowe maksima:

- RTT pozycji: J1 4,937 ms, J2 6,185 ms, J4 4,958 ms;
- timestamp jądra → odczyt C++: 4,641 ms;
- publikacja ROS → callback backendu: 9,536 ms;
- odstęp wysyłek WS: 64,816 ms;
- odstęp pętli I/O: 6,597 ms, poniżej lokalnego terminu pozycji 60 ms.

To maksima zachowanego okna, nie gwarancje terminów. Rotacja może usuwać starsze
ramki: analizator podaje osobny zakres czasu C++ i backendu. Lokalne podsumowania
`log/manual-can/hardware-5rpm.json` i `hardware-30rpm.json` zachowują wyniki prób;
`analysis-30rpm.json` i `diagnosis.txt` opisują analizowane okno. Pierwsza wersja
bufora miała pominięcia wpisów; zastąpiono ją kolejką MPSC i ponowiono próby
30 RPM, wtedy `dropped_total=0`.

Sprawdzenia automatyczne: zbudowano 8 pakietów, końcowy `colcon test-result`
114 testów, zero błędów/niepowodzeń. TypeScript, build, 12 Vitest i Chromium OK.
Testy rejestratora obejmują rotację, zamrożenie, równoczesnych producentów oraz
odróżnianie sesji, także po zmianie zegara monotonicznego przy ponownym starcie.

Rejestracja pozostaje włączona. Do dalszej diagnozy należy odświeżyć panel na
urządzeniu, na którym występuje objaw, i odtworzyć go z aktywnym zapisem.
Testy z tej sesji korzystały z lokalnej przeglądarki, bez kontroli przeglądarki
na drugim fizycznym urządzeniu LAN. Interpretacja: [instrukcja nagrywania](can_recording.md).
