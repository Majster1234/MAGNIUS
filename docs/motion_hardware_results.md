# Odbiór sprzętowy PTP — 2026-10-21

Wynik: **przerwany na jednym silniku; nie zaliczono odbioru PTP**.
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
