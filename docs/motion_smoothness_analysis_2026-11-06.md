# Płynność JOG TCP i PTP/LIN — analiza 06.11.2026

Zakres: analiza źródeł i testy offline. Nie wykonano ruchu robota, nadawania na
`can0`, zmiany parametrów napędów ani restartu usług. Kod produkcyjny i konfiguracja
pozostały bez zmian. Testy opisują generowanie komend i harmonogram; nie identyfikują
jeszcze fizycznej dynamiki silników.

## Stan odczytany i istotne różnice wykonawców

Zapisany `/home/mk/.local/state/robot/configuration/active.json`, rewizja
`d46e996f38a24eabaadb146cb03a58c2`, zawiera
`motion.ptp_lin_dispatch_mode=synchronized`, `motion.position_integral=true`
i przełożenia J1–J6 równe 10. To odczyt pliku, bez potwierdzenia parametrów
bieżącego procesu i mikrokroku rzeczywistych napędów.

- PTP/LIN `synchronized`: wspólna faza i horyzont, przyszły punkt odczytany
  z trajektorii, przygotowanie F5 z ACK oraz wyzwolenie SYNC. Sprzężenie zwrotne
  odnosi się do referencji w chwili pomiaru. Źródło:
  `mks_servo_can/src/synchronized_program.cpp:210–244`.
- JOG TCP: kolejka q/v/a generowana do 200 ms naprzód, ale cel F5 pochodzi
  z ekstrapolacji `q+v*h+0.5*a*h²`, ze skracaniem horyzontu przy hamowaniu.
  Korekcja wieku pomiaru również używa ekstrapolacji. Źródła:
  `mks_servo_can/src/tcp_jog.cpp:137–162`, `streaming_motion.cpp:42–55`.
- JOG TCP wysyła F5 tylko w pierwszych 2 ms slotu 10 ms. Dokładny termin
  zwracany przez `next_deadline()` obsługuje obecnie tylko przygotowany SYNC,
  a pozostałe przebudzenia IO wynikają z pętli 1 ms. Źródła:
  `tcp_jog.cpp:127–130`, `synchronized_program.cpp:37–40`, `bus_runtime.cpp:47–55`.
- Odczyty JOG TCP mają termin przed następnym slotem; brak odpowiedzi jest
  wygaszany przed planowaniem F5. Blokada `pending_` chroni przed transmisją
  hosta i napędu o tym samym CAN ID. Nie ma podstaw do jej prostego usunięcia.
  Źródło: `bus_manager.cpp:435–438, 630–643, 837–851`.
- Generator JOG TCP ogranicza jerk drogi skalarnej. Przyspieszenia przegubów
  wylicza różnicą prędkości z 10 ms. Planer PTP/LIN sprawdza prędkość,
  przyspieszenie i tor TCP, lecz nie egzekwuje jawnego limitu jerku przegubów.
  Interpolacja kwintyczna zapewnia zgodność q/v/a na węzłach; sama nie ogranicza
  wartości jerku. Źródła: `robot_kinematics/src/tcp_jog.cpp:34, 59`,
  `robot_motion_planner/src/planner.cpp:251–289`.

## Test 1: spóźnione wykonanie pętli IO

Skompilowano bieżące źródła `mks_servo_can` w osobnym katalogu tymczasowym,
z istniejącym emulatorem `test/streaming_fixture.hpp`. Zegar wirtualny, tick 1 ms,
sześć osi, okres komend 10 ms. Obserwacja po rozruchu: 400 ms. Pominięto wywołania
pętli tak, aby pierwsze po terminie nastąpiło o wskazane spóźnienie. Nie symulowano
obciążenia CPU, USB, arbitrażu magistrali ani dynamiki napędu.

| Spóźnienie pojedynczego terminu | JOG TCP: największy odstęp F5 | PTP/LIN SYNC: największy odstęp SYNC |
| --- | --- | --- |
| 0 ms | 10 ms | 10 ms |
| 1 ms | 11 ms | 11 ms |
| 2 ms | **20 ms, pominięty jeden zestaw** | 12 ms |
| 3 ms | **20 ms, pominięty jeden zestaw** | 13 ms, przesunięcie osi czasu |
| 8 ms | **20 ms, pominięty jeden zestaw** | 18 ms, przesunięcie osi czasu |

We wszystkich powyższych przypadkach emulator pozostał bez awarii i bez braków ACK.
39 zamiast 40 zestawów w oknie SYNC przy spóźnieniu 8 ms wynika z przesunięcia
osi czasu poza koniec obserwacji; nie jest odrzuceniem zestawu jak w JOG TCP.
Brak odpowiedzi na odczyt prędkości w osobnym teście JOG TCP zachował 40 zestawów
i maksymalny odstęp 10 ms.

Dodatkowo, bez pomijania ticków, ustawiono syntetyczne opóźnienie każdej odpowiedzi
na 2, 3, 4, 5, 6 i 8 ms: SYNC zachował okres 10 ms i brak utraconych ACK.
Przy 10 ms wystąpiła oczekiwana awaria po trzech kolejnych brakach ACK osi
(13 brakujących potwierdzeń w całej grupie przed zatrzymaniem).
To test bramki czasowej, nie pomiar rzeczywistego RTT ani zalecenie zmiany timeoutów.

Wniosek: istnieje potwierdzona podatność harmonogramu JOG TCP na pojedyncze
spóźnienie 2 ms. Poprawa wymaga zaplanowania dokładnych terminów i obsługi
spóźnionego aktualnego zestawu przed odczytami, z zachowaniem ochrony CAN ID
i bez odtwarzania zaległych próbek. Samo dokładniejsze usypianie nie usunie
spóźnień powodowanych przez system operacyjny.

## Test 2: przybliżony i dokładny przyszły cel

Przypadek syntetyczny: monotoniczna droga jednego przegubu 0 → 0,1 rad,
prędkość 0,05 rad/s, przyspieszenie 0,5 rad/s², jerk 2 rad/s³. Są to dane
testu matematycznego, nie nowe limity robota ani propozycja próby fizycznej.
Użyto rzeczywistego `robot_kinematics::JerkProfile`, `streaming_setpoint`,
przełożenia testowego 10, horyzontu 85 ms i 233 próbek co 10 ms.

Porównano istniejące wywołanie API z wariantem podającym opcjonalny dokładny cel
`profile.sample(min(t+0.085, duration)).position`. Korekcja całkująca i sprzężenie
zwrotne były wyłączone, aby odizolować sposób wyznaczania celu.

| Wariant przyspieszenia w próbce | Maks. różnica celu | RMS różnicy | Cofnięcia celu przybliżonego / dokładnego |
| --- | --- | --- | --- |
| Analityczne | 0,011454° | 0,004936° | 2 / 0 |
| Różnica prędkości z 10 ms, jak w generatorze JOG | 0,013475° | 0,005714° | 3 / 0 |

W obu przypadkach dwa cofnięcia przetrwały kodowanie celu na impulsy F5.
Największy krok wstecz przed kwantyzacją wynosił odpowiednio 0,003362°
i 0,003535°. Dokładny cel zachował monotoniczność.

Wniosek: podawanie celu z kolejki trajektorii JOG TCP usuwa małe, sztuczne
cofnięcia w tym przypadku. Skala nie uzasadnia przypisywania temu całego
obserwowanego szarpania. Wdrożenie wymaga także ograniczenia horyzontu przez
koniec kolejki, najbliższy nawrót i STOP oraz sprawdzenia toru TCP między węzłami.
Istniejący PTP/LIN SYNC już korzysta z dokładnego przyszłego punktu.

## Test 3: wolne prędkości

Dla mikrokroków 16 i 128 wykonano po 100 wywołań generatora komend dla prędkości
wału 0,2, 0,5, 0,9, 1,2, 1,5 i 1,9 RPM. `min_holding_rpm=1`, brak sprzężenia,
stałe żądanie; wyniki przeliczono z jednostek protokołu na RPM wału.

| Żądane RPM wału | Średnia komenda, mikrokrok 16 | Średnia komenda, mikrokrok 128 |
| --- | --- | --- |
| 0,2 | 1 | 1 |
| 0,5 | 1 | 1 |
| 0,9 | 1 | 1 |
| 1,2 | 1,2 | 1,2 |
| 1,5 | 1,5 | 1,5 |
| 1,9 | 1,9 | 1,9 |

Kod ogranicza komendę od dołu do 1 RPM wału i zeruje akumulator ditheringu
poniżej tej wartości (`streaming_motion.cpp:67`). Mikrokrok 128 zmniejsza
jednostkę prędkości do 0,125 RPM według przelicznika programu, ale nie obniża
tego minimum. Dla przełożenia 10 jest to komenda 0,6°/s przegubu.
To ograniczenie prędkości komendy pozycjonera, nie pomiar rzeczywistej prędkości:
napęd może dojechać do krótkiego celu i czekać.

[Wcześniejsze wyniki sprzętowe z 04.11](benchmark_results.md#mikrokrok-j1-16--128--04112026)
dokumentują poprawę uchybów po mikrokroku 128 oraz pozostające przestoje
przy około 2–3 RPM. Samo ograniczenie poniżej 1 RPM nie wyjaśnia tych przestojów.
Wcześniejszy pomiar nie rozstrzyga osobno udziału firmware, regulatora i mechaniki.

## Zalecana kolejność dalszej pracy

1. **Harmonogram JOG TCP:** dokładny następny termin dla IO, licznik pominiętych
   slotów oraz kontrolowana obsługa spóźnionego zestawu. Ponowić tę samą macierz
   opóźnień przed/po; osobno zmierzyć rzeczywiste terminy F5 i SYNC podczas ruchu.
2. **Spójne próbkowanie JOG TCP:** cel z przyszłego punktu istniejącej kolejki,
   uchyb względem referencji z chwili pomiaru, ograniczenia horyzontu wspólne dla
   osi. Zweryfikować start, hamowanie, nawrót, krótkie impulsy i ucięcie kolejki.
3. **Identyfikacja napędu na pojedynczej osi:** porównać stałą prędkość F6
   i odpowiadający jej strumień F5, w obu kierunkach, w tym w rejonie
   wcześniej obserwowanych 2–3 RPM. Zacząć od J1; J4 zbadać osobno.
   Rejestrować cele i RPM komend, pozycję 0x31, prędkość 0x32, błąd kąta 0x39,
   wiek pomiarów i terminy transmisji. Odczyty diagnostyczne muszą mieścić się
   w istniejącym budżecie CAN, bez dodatkowego konkurującego nadajnika.
   Porównać odsetek postojów w oknach 100 ms, rozrzut prędkości po jednakowej
   filtracji, uchyb i kierunek. Różniczkowanie surowego enkodera co 10 ms jest
   zbyt podatne na kwantyzację, aby samo rozstrzygało płynność.
4. **Dopiero po pomiarze:** dobór lookahead, FF, Kp, całkowania i parametrów
   napędu po jednym; osobno rozważyć reprezentację prędkości poniżej 1 RPM.
   Nie zastępować automatycznie małej prędkości komendą F5=0 RPM.
5. **PTP/LIN:** dodać ocenę jerku po końcowej zmianie parametryzacji czasu;
   ewentualne wygładzanie musi zachować sprawdzoną drogę TCP, prędkości,
   przyspieszenia i limity mechaniczne. Limitów jerku osi nie określono na
   podstawie obecnych pomiarów.

Przed każdą próbą fizyczną trzeba odczytać aktualną pozycję, kalibrację,
mikrokrok, parametry i limity. Następnie opisać konkretną oś, kierunek,
prędkość, czas/zakres i warunek zatrzymania oraz uzyskać wyraźną zgodę operatora
zgodnie z `AGENTS.md:16`. Nierozstrzygnięta anomalia F5 osi J5 z kontekstu
05.11 wymaga osobnego sprawdzenia przed włączeniem jej do prób skoordynowanych.

## Artefakty lokalne i odtwarzanie

Katalog roboczy diagnostyki: `/tmp/robot-smoothness-20261106-nves_70i/`.
Zawiera `probe.cpp`, `CMakeLists.txt`, `results.txt` i oddzielny build Release.
Katalog jest tymczasowy; jego retencja zależy od systemu. Probe używa emulatora
`Port`, nie konstruuje transportu SocketCAN ani `BusRuntime`.

```bash
cmake -S /tmp/robot-smoothness-20261106-nves_70i \
  -B /tmp/robot-smoothness-20261106-nves_70i/build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/robot-smoothness-20261106-nves_70i/build --target probe -j2
/tmp/robot-smoothness-20261106-nves_70i/build/probe
```

Uruchomienie zakończone kodem 0; awaria dla syntetycznego opóźnienia odpowiedzi
10 ms jest celowo zaobserwowanym zachowaniem badanego sterownika.
Nie uruchamiano pełnych testów ROS, benchmarku CPU ani usług robota.

SHA-256 analizowanych źródeł `mks_servo_can/src/`:

```text
b602dc3ee18798506edfa666afc9f61d1c3f9a68245d27c0bba257a18f6e1b02 streaming_motion.cpp
a9d982365dc7c08ef3c0ad0f8f0de9726a6f1d24494363eb2a7ab9cebe493beb tcp_jog.cpp
a88ce01cb5a93f1be048cde19713f17f62c2b4f17c09ba2cb6c922d3e5051dd7 synchronized_program.cpp
0c6f6a7c6e4632feec9602d4ebb26ddb738ae3448a21e2863304e22578abe800 bus_runtime.cpp
```
