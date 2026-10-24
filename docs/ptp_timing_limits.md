# Progi strumieniowania PTP — 24.10.2026

Obowiązuje nowy kontrakt [100 Hz bez ACK ruchu](motion_execution.md).
Poprzednie ACK5/ACK10, 20 ms i pauzy pozycyjne są historycznymi wariantami
[raportu sprzętowego](motion_hardware_results.md).

| Operacja | Aktualna zasada |
| --- | --- |
| Referencja/F5 | 10 ms, wspólna faza q/v, lookahead 25 ms i FF 1,15 |
| Spóźnienie procesu | jedna aktualna klatka, pominięcie nieaktualnych, bez serii nadrabiania |
| F5/F6/FE | bez ACK, timeoutu ACK i SYNC; CanRSP=Disable, 4A=0 |
| Pozycje31 w ruchu | nominalnie 100 Hz/oś, RX w osobnym wątku |
| Tolerancja pośrednia | nie wstrzymuje strumienia |
| Koniec | ±0,5° przegubu; dwa świeże komplety pomiarów >=100 ms od siebie |
| Brak końcowej pozycji | waiting_feedback, bez limitu dolotu i bez ponownego F5 |
| Końcowa odchyłka | waiting_position; MKS kończy ostatni cel |
| Świeżość pozycji końca | <250 ms i zapytanie po rozpoczęciu kontroli końca |
| Start | F1=postój, pomiary zgodne ze startem; bez dokładnego32=0 |
| Uzbrajanie | <=3 s; konfiguracja/odczyt pojedynczo100 ms |
| Zwykły odczyt | timeout20 ms, cztery miejsca oczekiwania |
| ENABLE/stall | 10 Hz; jawny błąd przerywa ruch |
| Prędkość/F1 | 20 Hz; nie wyznacza tolerancji końcowej |
| Wątek IO / RX | pobudki1 ms / poll 1 ms; brak gwarancji twardego RT |
| Budżet | robot40%, rezerwa20 punktów, suma60%; sprawdzenie kosztu całej grupy |
| STOP | F7 wszystkich osi, następnie F6(0), natychmiast bez ACK, potem cisza |
| JTC | watchdog komend/pozycji 60 ms i lease 200 ms pozostają odrębne |

Sześć osi wraz z diagnostyką:26,58% przy 1 Mbit/s;53,16% przy 500 kbit/s.
Profil500k dla sześciu osi jest odrzucany. Sam brak odczytu pośredniego w PTP
nie wywołuje STOP; BUS-OFF/USB, jawny błąd napędu i STOP operatora pozostają aktywne.
Wyłączenie odpowiedzi ruchu oznacza, że wysłana ramka nie jest potwierdzeniem
przyjęcia rozkazu. STOP również wymaga fizycznego sprawdzenia skuteczności.

Po próbach1/2/4 osi dodano w PTP przerwę rozpoczynania odczytów 4 ms przed F5,
aby odpowiedź MKS z tym samym CAN ID nie kolidowała z komendą. Nie jest to
oczekiwanie na pomiar. W praktyce cztery osie: F5 średnio 10,000 ms, maksimum 12,140 ms;
pozycje średnio 11,12–11,32 ms. Dane i ograniczenia: [raport](motion_hardware_results.md).
F7 nie oczekuje ACK także podczas przygotowania ENABLE; postój nadal musi
potwierdzić następujący odczyt 32/F1 oraz pozycje. Testy: 109 CAN bez błędów.
