# Odbiór sprzętowy PTP

Najnowsza seria: [pięć prób limitów — 2026-10-22](#seria-limitów--2026-10-22).
Próba 5 spełniła kryteria końcowe w wariancie stanowiskowym. Płynność i pełny
odbiór wykonawcy produkcyjnego nadal nie są potwierdzone.

Wynik z 2026-10-21: **przerwany na jednym silniku; nie zaliczono odbioru PTP**.
Nie uruchamiano etapów dwóch i czterech silników po wykryciu ruchu po sekwencji
kasowania bufora. Fizyczny wykonawca ROS pozostaje wyłączony.

## Stanowisko i zatwierdzona procedura

Użytkownik potwierdził rozsprzęglenie silników i zatwierdził kolejność CAN 1,
1+2, następnie 1+2+3+5. Zatwierdzone parametry: dodatni kierunek enkodera/CW,
nominalnie +5° wału od bieżącego startu i powrót, najwyżej 1 RPM, nadzór celu
3 s, sprawdzenie F7/STOP i braku ruchu przed SYNC. Błąd przerywa dalsze etapy.
Nie jest to test geometrii, TCP, planera ani pętli wykonawcy ROS.

- Wykryte i odczytane CAN 1, 2, 3, 5: SERVO42D, firmware 1.0.9,
  kalibracja napędu zgłoszona jako poprawna, mikrokrok 16.
- `can0`, U2C/gs_usb, 500000 bit/s, stan ERROR-ACTIVE, brak nowych błędów
  magistrali, przejść error-passive i bus-off podczas prób.
- Przed testem wszystkie cztery napędy miały ENABLE=0, SR_vFOC,
  odpowiedzi 8C=01/01, watchdog 300 ms i synchronizację 4A=0.
- Narzędzie: `scripts/test_ptp_hardware.py`; jeden właściciel z blokadą
  `/tmp/robot-can-<uid>-can0.lock`, bez równoległego nadawania przez diagnostykę.
  Proces diagnostyki zamknięto SIGINT; po próbie odtworzono stos przez zamknięcie
  własnego launch i istniejącą politykę restartu systemd. Sudo wymagało hasła;
  nie korzystano z podwyższenia uprawnień użytkownika.
- Zadano F5 z raw RPM=1, acc=1. Cel nominalnych 5° kwantyzuje się do 228
  impulsów enkodera (5,0098°). Nie zmieniano mikrokroku ani zera wału.

## Wyniki

| Etap | Wynik | Obserwacja |
| --- | --- | --- |
| Przygotowanie CAN 1, próba pierwsza | Przerwane przed F5 | Po ENABLE wał nadal się stabilizował; 248→235 impulsów w 150 ms, około 0,29°. STOP i wyłączenie potwierdzone. |
| Przygotowanie CAN 1, próba druga | Zakończone | Narzędzie czeka na 100 ms stabilności w paśmie 3 impulsów, maksymalnie 2 s. Parametry ruchu bez zmian. |
| F5/ACK przed SYNC, CAN 1 | Zaliczone w tej próbie | ACK `F5 05`, pozycja 238 impulsów przez około 120 ms, brak wykonania przygotowanego celu. |
| F7 po przygotowaniu, potem SYNC | **Niezaliczone** | ACK F7=1; późniejszy SYNC spowodował narastanie pozycji 238→245 w około 54 ms. Próg przerwania: ponad 5 impulsów od pozycji sprzed SYNC. |
| F7 podczas normalnego przejazdu, pełne +5° i powrót | Niewykonane | Procedura przerwana wcześniej. |
| Dwa silniki: CAN 1+2 | Niewykonane | Warunek przejścia z jednego silnika nie został spełniony. |
| Cztery silniki: CAN 1+2+3+5 | Niewykonane | Jak wyżej. |

Ślad drugiej próby, czasy względem potwierdzenia przygotowania F5:

| Czas | CAN | Zdarzenie |
| --- | --- | --- |
| −0,83 ms | 1 | Wysłano `F5 00 01 01 00 01 CE C7`, cel 462 impulsy. |
| −0,15 ms | 1 | Odebrano `F5 05 FB`. |
| 2–114 ms | 1 | Odczyty pozycji: 238 impulsów. |
| 122,71 ms | 1 | Wysłano F7; ACK=1 po około 0,67 ms. |
| 124,67 ms | 0 | Wysłano `4B 4B` — sprawdzenie skasowania przygotowanego rozkazu. |
| 126,57–178,14 ms | 1 | Odczyty 238, 240, 240, 241, 243, 245. |
| 178,41 ms | 1 | Awaryjne F7; następnie F3=0 i 4A=0. |

Przesunięcie wykryte po SYNC wyniosło 7 impulsów, około 0,154°.
Wniosek z tej próby: **ACK F7=1 nie stanowi potwierdzenia opróżnienia
przygotowanego polecenia F5**. Nie potwierdzono skutecznego kasowania bufora
inną sekwencją. Nie jest to pomiar dokładności wspólnego startu ani płynności.

## Stan końcowy i dalsze warunki

Po obu próbach STOP i ENABLE=0 potwierdzono dla CAN 1, 2, 3, 5. Odtworzono
odczytane przed próbą tryb, odpowiedzi 8C, watchdog i 4A; cleanup obu prób
zakończył się bez błędów. Końcowy odczyt panelu: świeży stan, brak zajętości
i fault, wszystkie obecne napędy wyłączone, prędkości wyznaczane z enkoderów
bliskie zeru. `robot-web-status.service` ponownie aktywna. Rewizja aktywnej
konfiguracji pozostała bez zmian; nie zapisywano profilu ani flag odbioru.

Ślady lokalne, pomijane przez Git:
`log/hardware-ptp/20261021-224713.jsonl` i
`log/hardware-ptp/20261021-224743.jsonl`.

Następny krok: ustalić i osobno sprawdzić na jednym wałku sekwencję bezpiecznego
usuwania/zastępowania przygotowanego rozkazu. Dopiero po potwierdzeniu braku
starego ruchu po SYNC i testu zatrzymania można ponowić +5°/powrót oraz przejść
do 2 i 4 silników. Nie ustawiać `execution_staged_reset_verified=true`
na podstawie samej odpowiedzi F7.

Podstawa formatu ramek: [instrukcja producenta MKS CAN V1.0.9, §9.2.3 i §12.3](https://github.com/makerbase-motor/MKS-SERVO42D-57D/blob/master/User%20Manual/V1.0.9/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.9.pdf).
Powyższe zachowanie bufora wynika z pomiaru, a nie z deklaracji instrukcji.

## Zmiana procedury — 2026-10-22, bez nowej próby sprzętowej

Na polecenie użytkownika procedura STOP/kasowania przygotowanego celu otrzymała
sekwencję: F7 do wszystkich CAN 1–6, następnie F6 z RPM=0 do wszystkich CAN 1–6.
Nie wysyła przy tym SYNC i nie załącza ENABLE. Wykonawca zbiera osobno ACK obu
faz; brakujące lub negatywne odpowiedzi są raportowane. Po najwyżej dwóch
okresach timeoutu kończy nadawanie. Samo ACK nadal nie uprawnia do ustawienia
`execution_staged_reset_verified`.

Narzędzie `scripts/test_ptp_hardware.py` używa tej sekwencji przed próbą SYNC,
przy STOP w ruchu oraz w cleanup. Pozostałe napędy mogą zwrócić odmowę F6 przy
ENABLE=0; zapisujemy odpowiedź, bez załączania momentu. Wymagane ACK dotyczą
osi uczestniczących w danym etapie. Próba kasowania nadal kontroluje brak
starego ruchu po późniejszym SYNC i przerywa procedurę po przekroczeniu progu.

Nie wykonano nowego ruchu, resetu sprzętu ani odbioru tej sekwencji. Wynik próby
z 2026-10-21 pozostaje niezaliczony. Kolejny krok sprzętowy: osobny odbiór
F7 → F6(0) na jednej rozsprzęglonej osi, a po jego zaliczeniu kolejne etapy.

## Seria limitów — 2026-10-22

Wykonano **pięć prób PTP na CAN 1**, w tym jedną odrzuconą przed pierwszym
SYNC. Ostatnia spełniła końcowe kryteria stanowiska. Nie jest to pełny odbiór
robota, jego płynności ani aktywacja wykonania w panelu.

### Zgoda, zakres i warunki

Użytkownik zatwierdził CAN 1/J1, dodatnie +10° **wału**, maksymalną nastawę
1 RPM, rozsprzęglone wały i do pięciu prób. Przed każdą próbą wykonywano
funkcję `ManualMotorController::zero()` odpowiadającą „zeruj wał”. Pierwsze
zerowanie wykonano także przez istniejący endpoint panelu; potwierdzono 0°
i trwałe unieważnienie odniesienia przegubu CAN 1.

Kryteria serii: przeregulowanie dozwolone, tolerancja **końcowa ±0,3°**,
wymaganie RPM=0 dopiero po nominalnym końcu +100 ms. Zachowano dodatkowe
100 ms potwierdzania stabilnego końca i limit końca +1 s. Wyłączono odrzucanie
za przejściowy błąd śledzenia w izolowanym wariancie; nie oceniano tym samym
produkcyjnego nadzoru śledzenia podczas ruchu.

Profil quintic trwał 3,2 s, miał 160 odcinków po 20 ms i teoretyczny szczyt
0,977 RPM. Kompilator automatycznie wybrał `streamed`; ramki F5 miały najwyżej
raw RPM=1 przy odczytanym mikrokroku 16. Cel 10° kwantyzuje się do 455 impulsów
enkodera (9,99756°). Tolerancję oceniano względem nominalnych 10°.

Stanowisko używało rzeczywistych `CanBusManager`, kompilatora, kontrolera
ręcznego i transportu SocketCAN, bez akcji ROS i planera TCP. Proces widział
wyłącznie CAN 1; zapytania/odpowiedzi pozostałych napędów były filtrowane.
STOP nadal wysyłano do CAN 1–6. Profil 1:1 opisywał współrzędną samego wału,
nie zastępował konfiguracji geometrii robota. Pozostałe obecne napędy 2, 3 i 5
wyłączono przed próbami. Zachowano jednego właściciela magistrali.

### Wyniki i zmiany między próbami

| Próba | Zmienione warunki względem poprzedniej | Wynik | Dowód / przyczyna |
| --- | --- | --- | --- |
| 1 | Bazowe czasy: SYNC +1 ms, diagnostyka do 18 ms; końcowe kryteria jak wyżej | Niezaliczona, 0 SYNC | Uzbrajanie odrzuciło odczyt `32 FF FF` = −1 RPM przed startem. To warunek wartości, nie timeout. Enkoder był wyzerowany; rozkaz ruchu nie ruszył. |
| 2 | Przed startem pozostawiono potwierdzenie postoju F1 i stabilność enkodera; dokładne RPM=0 wymagane po końcu +100 ms. Bez zwiększenia czasów | Niezaliczona, 142 SYNC | Spóźnienie SYNC około **1,782 ms >1 ms**. Pozycja przy przerwaniu 9,910°. Komplet ACK był dostępny. |
| 3 | Tylko dopuszczalne spóźnienie SYNC: **1 →2,5 ms** | Niezaliczona, 19 SYNC | Odczyt F1 obsłużono po **2,093401 ms** przy dostępnym czasie **2,049027 ms**; przekroczenie 44,374 µs. Pozycja 0,176°. |
| 4 | Koniec odpowiedzi diagnostyki **18 →19 ms**, minimalny zapas na odpowiedź **2 →3 ms**. Nadawanie nadal [14,16) ms; rezerwa końcowa 2 →1 ms | Niezaliczona jako cała próba, 160 SYNC | Czasowo poprawna i cel spełniony: RPM=0, około 10,042° przy pierwszej ocenie końca. Potem fałszywy błąd śledzenia ze ścieżki JTC. |
| 5 | Izolowana poprawka przejścia po sukcesie: zapamiętanie źródła PTP na początku iteracji IO. Bez kolejnego zwiększania czasów | **Kryteria końcowe zaliczone**, 160 SYNC | Pozycja **9,90967°**, błąd **−0,09033°**, RPM=0. Maks. spóźnienie SYNC **1,63709 ms <2,5 ms**. |

Próba 4 ujawniła błąd w `CanBusManager::tick()`: `schedule_program()` po sukcesie
ustawia `active_=false` i `source_=none`. Dalszy kod tej samej iteracji uznaje,
że nie obsługuje już PTP, i porównuje pozycję ze starym `committed_positions_`
ścieżki JTC, tutaj z 0°. Powstaje „Przekroczony błąd śledzenia osi”, choć cel
PTP został osiągnięty. Poprawkę przetestowano tylko w wariancie stanowiskowym;
**źródła wykonawcy produkcyjnego nadal wymagają poprawki i regresji**.

Wariant próby 4 wymagał poprawienia brakującego nagłówka podczas kompilacji.
Nieudane uruchomienie przed zbudowaniem pliku nie wysłało ruchu i nie stanowi
dodatkowej próby PTP.

### Pomiary czasowe i ograniczenia wyniku

W próbie 5:

- Odstępy zapisów SYNC: średnio **20,0047 ms**, min. **19,0931 ms**,
  maks. **21,5656 ms**, odchylenie standardowe **0,2307 ms**.
- Odstępy zapytań enkodera podczas nominalnego ruchu: średnio **40,0089 ms**,
  min. **39,2985 ms**, maks. **40,6977 ms**. Maksymalny RTT enkodera 2,0973 ms.
- Maksymalny RTT ACK F5 podczas ruchu: **3,2283 ms**. Minimalny zapisany
  margines ACK do SYNC: **16,2068 ms**; to inny punkt odniesienia niż deadline
  okna ACK 8 ms. Nie naruszono okna ACK.
- Maksymalne odstępy odczytów: ENABLE **120,0875 ms**, stall **120,0332 ms**,
  prędkość **79,0122 ms**, F1 **60,2415 ms**. To nominalne okresy rozłożone na
  cykle; nie są równymi odstępami 100/50 ms. Nie wystąpiła utrata świeżości.
- Maksymalny RTT odczytu prędkości podczas ruchu: **3,9717 ms**, przy nowym,
  wydłużonym oknie bez timeoutu. Pierwszy zapis oceny RPM=0 wykonano około
  **117,7 ms po nominalnym końcu**, następnie potwierdzano stabilny koniec.
- Modelowy szczyt rezerwacji całej sesji: **25,8%**, poniżej 40%. To wynik
  jednego uczestnika, obejmujący również przygotowanie; nie dowodzi budżetu
  sześciu osi i nie jest pomiarem zajętości przewodu.

W próbie 3 znacznik odbioru jądra poprzedzał obsługę spóźnionej odpowiedzi F1
w procesie o około **0,767 ms**. Wskazuje to na udział opóźnienia obsługi RX,
a nie dowodzi, że napęd lub przewód odpowiedział po swoim terminie. Czasy
wywołań SocketCAN nie są sprzętowym pomiarem transmisji ani próbkowania.

Odczyt pozycji pokazuje **skokowy przebieg**. W próbie 4 szczyt wyniósł
**10,5469°**; przeregulowanie było dopuszczone i pozycja wróciła do pasma.
W próbie 5 największa różnica kolejnych odpowiedzi enkodera to **1,56006°
w 41,0923 ms**, co odpowiada estymacie około **6,33 RPM**. Nie potwierdza to
utrzymania fizycznej prędkości ≤1 RPM, mimo ograniczenia rozkazów do 1 RPM.
Potrzebna jest osobna weryfikacja dynamiki napędu i chwili aktualizacji danych
enkodera. **Zaliczenie końca nie oznacza zaliczenia płynności.**

![Pozycja wału i odchylenie zapisów SYNC w próbach 2–5](assets/ptp-limits-20261022.svg)

Wykres pokazuje nadane SYNC. Odrzucony termin ze spóźnieniem 1,782 ms w próbie 2
nie ma punktu, ponieważ ramki nie wysłano.

### Odbiór STOP i interpretacja odpowiedzi

Przed serią wykonano dodatkowe sprawdzenia przygotowania i kasowania celu,
bez pełnych przejazdów PTP. F6 przy wyłączonym/nieprzygotowanym napędzie zwracało
`00`, a po przygotowaniu CAN 1 i włączeniu `4A=1` zwróciło **`05`**. Producent
opisuje `05` jako przyjęcie danych do synchronizacji, nie potwierdzenie postoju
([instrukcja MKS V1.0.9, §10.1 i §12.3](https://github.com/makerbase-motor/MKS-SERVO42D-57D/blob/master/User%20Manual/V1.0.9/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.9.pdf)).

Pierwsze kontrole odrzucono z powodu wymagania ACK F6=1 oraz starego progu
bezruchu 5 impulsów (0,1099°). Następnie zastosowano pasmo 0,3° z bieżącego
zlecenia, zachowując zapis surowej odpowiedzi `05` jako odbioru synchronicznego.
Po kontrolnym SYNC odczytano po 100 ms F1=1 i RPM=0; przez sekundę pozycja
zmieniła się maksymalnie o 6 impulsów (0,13184°), bez wykonania celu +10°.
To ograniczony odbiór jednego wału w tym paśmie, a nie dowód zerowego ruchu.

Produkcyjna procedura STOP uznaje obecnie tylko `01` za dodatnią odpowiedź F6,
więc `05` oznacza w jej statystykach niepotwierdzony STOP. Nie należy utożsamiać
tego z timeoutem ani automatycznie zaliczać `05` jako zatrzymania. W próbach
1–3 jej komunikacja kończyła się w około 4–5 ms; po STOP nie było dalszego SYNC.
W próbie 4 błąd wystąpił już po wyłączeniu `active_`, więc sprzątanie wykonała
również zewnętrzna procedura F7 →F6(0) →ENABLE=0. Po każdej próbie odpowiedzi
ENABLE=0 wszystkich obecnych napędów były potwierdzane.

### Narzędzie, ślady i stan końcowy

- Narzędzia: `scripts/build_ptp_limits.py`, `scripts/ptp_limits_hardware.cpp`,
  `scripts/ptp_limits_session.py`. Budowanie tworzy kopie z jawnymi zmianami
  w `/tmp`; nie podmienia pakietów ani konfiguracji ROS.
- Bazowa biblioteka z `install/can-schedule-ros` jest wcześniejszym Release -O1.
  Nadpisane metody stanowiskowe i narzędzie skompilowano GCC 11 z **-O0**.
  Wyniki dotyczą tego procesu testowego, nie deterministyczności usługi ROS.
  Nie wykonywano porównania wydajności ani pomiaru CPU procesu.
- Lokalne ślady: `log/hardware-ptp/limits-20261022/`, katalogi `trial1`, `trial2`,
  `trial3`, `trial4-run`, `trial5`; podsumowanie `analysis.json`, kopie zmienionych
  metod i `build.json` z argumentami oraz SHA-256. Ślady nie utraciły zdarzeń.
  Każda sesja ma 38 oczekiwanych timeoutów skanu odfiltrowanych adresów
  (start i bootstrap); dodatkowy timeout aktywnego PTP wystąpił tylko w próbie 3.
- Sprawdzenia narzędzia: kompilacja wariantów, `py_compile`, trzy istniejące
  testy sekwencji STOP na atrapach i kontrola odmowy uruchomienia bez flag zgody.
  Nie powtarzano wcześniejszych 128 testów ROS ani buildów panelu.
- `can0`: ERROR-ACTIVE, bez wzrostu liczników bus-error, error-warning,
  error-passive i BUS-OFF w tej serii. Nie wykonywano iniekcji USB/BUS-OFF,
  utraty ACK ani próby `native_ptp`, dwóch lub czterech uczestników.
- Panel odtworzono przez zamknięcie dotychczasowego launch i jego istniejący
  automatyczny restart systemd. Końcowy odczyt API: świeży, bez busy/fault,
  CAN 1/2/3/5 z **ENABLE=0**. Po odpuszczeniu momentu CAN 1 wskazywał około
  9,338°; wynik 9,910° pochodzi sprzed STOP/wyłączenia momentu.
- Rewizja konfiguracji i pliki wykonania pozostały bez zmian. Kalibracja CAN 1
  została prawidłowo unieważniona i utrwalona wskutek zleconego zerowania;
  powrót do ruchu przegubu wymaga ponownego potwierdzenia odniesienia.
  Ulotne dopuszczenie testu nie ustawia flag odbioru w aplikacji produkcyjnej.

Następny krok: poprawić i objąć regresją przejście PTP →stan spoczynkowy oraz
rozróżnić w diagnostyce odbiór F6=05 od zatrzymania. Osobno zbadać skokowy
przebieg pozycji. Dopiero wtedy rozważać przeniesienie przetestowanych nastaw
2,5 ms / 19 ms do wykonawcy i odbiór kolejnych osi; pojedyncza udana próba
nie wyznacza maksymalnych opóźnień ani bezpiecznych nastaw całego robota.
