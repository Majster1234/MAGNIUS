# Próba 1 — skoordynowany LIN TCP, 06.11.2026

**Stan: zatwierdzona przez operatora; przerwana na starcie przez dodatkowy nadzór.**
Wysłano jedno żądanie wykonania LIN, zarejestrowano jeden SYNC. Pełny odcinek
nie został wykonany. Potwierdzono postój i ENABLE=0 wszystkich sześciu osi.
Operator potwierdził reset zasilania J5 po awariach F5 z 05.11; ten krótki
przebieg nie potwierdza jeszcze prawidłowego śledzenia F5 przez J5.

## Wynik próby sprzętowej — 19:14:20 CET

Zewnętrzny obserwator zlecił STOP z komunikatem „Brak świeżej pozycji/kalibracji
J1”. Przyczyną było przekroczenie progu wieku pozycji 150 ms w trakcie
uzbrajania SYNC; nie stwierdzono utraty kalibracji. Obserwator używał pozycji
ze snapshotów ROS/HTTP, publikowanych co 50 ms.

| Pomiar | Wynik |
| --- | --- |
| Żądania wykonania / SYNC w zapisie | 1 / 1 |
| Pierwszy SYNC od żądania wykonania | 162,296 ms |
| Przerwy surowych pozycji J1 / J2 / J3 | 141,350 / 171,552 / 139,341 ms |
| Przerwy surowych pozycji J4 / J5 / J6 | 169,571 / 140,211 / 171,483 ms |
| Błędne sumy kontrolne / ramki błędu CAN | 0 / 0 |
| Zapis pasywny | 4015 ramek, bez błędu odbiornika |
| Obserwacja wspólnego czasu po pierwszym SYNC | tylko 3,419 ms |
| Największe odchylenie wskazane przez monitor od pierwotnej prostej | 0,527 mm; obejmuje przygotowanie i załączanie |
| Wynik oceny płynności / dokładności jazdy TCP | **Brak wystarczających danych** |

Po STOP przez 1,469 s odczytano 15 snapshotów: wszystkie osie wyłączone,
stan świeży, brak aktywnego ruchu i awarii, `recovery_state=idle_disabled`.
Zmiana wskazania w tym oknie: J1/J5 po 1 impulsie enkodera przegubu
(0,002197°), pozostałe osie bez zmiany. Końcowe kąty J1–J6:
`[0,028564; 0,004395; 0,072510; −0,065918; 0; 0,002197]°`.

Kod wyjaśnia przerwę: `synchronized_query_window()` zwraca `false` podczas
`arming`, a `schedule_feedback()` respektuje tę bramkę. Uzbrajanie wykonuje
9 kolejnych transakcji na każdą oś. Pierwsze odczyty 31 wróciły około 144–146 ms
od żądania wykonania, przed SYNC; opóźniony snapshot ROS/HTTP nadal mógł zawierać
poprzednią pozycję. Limit świeżości istniejącego wykonawcy wynosi 250 ms,
natomiast dodatkowy monitor użył 150 ms także w fazie uzbrajania.

Źródła: `mks_servo_can/src/synchronized_program.cpp:61–69`,
`bus_manager.cpp:687–696`, `program_bus.cpp:246–292`.
Jest to niezgodność dodatkowego nadzoru z fazowaniem startu, a nie dowód
nieprawidłowej komunikacji CAN ani ocena dynamiki silników.

### Przygotowania poprzedzające jedyne żądanie wykonania

Trzy przygotowania zakończono przed wysłaniem `execute`: kontrola porównująca
plan z pierwotnym podglądem odrzucała kolejno przesunięcie startu J6 o 0,015381°,
przeliczony czas 3,77 s zamiast 4,02 s oraz odległość startu TCP 0,257088 mm
przy wewnętrznej tolerancji 0,25 mm. Były to zbyt ciasne warunki porównania
w skrypcie, a nie awarie napędów. Po każdym wysłano STOP i potwierdzono wyłączenie.
Nie wykonywano automatycznego ruchu powrotnego.

W ostatnim przygotowaniu sprawdzono rzeczywiste granice zatwierdzonej karty:
5 mm/s, 10 s całej próby, zakresy osi ±0,5°, prostopadłe odchylenie ≤3 mm
i rozszerzenie odcinka o 1 mm. Cel pozostał pierwotny. Po ustaleniu ENABLE
przygotowany program miał czas 4,08 s i start XYZ
`[440,01879; 0,20253; 449,45850] mm`, wobec czego jego odcinek do zatwierdzonego
celu miał 14,52553 mm. Maksymalna obliczona prędkość: 4,57124 mm/s, odchylenie
referencji od pierwotnej prostej: 0,39536 mm. Nie należy utożsamiać tej nowej
referencji z pierwotnym podglądem 4,02 s / 14,14214 mm.

### Dalsze przygotowanie nadzoru, bez następnego ruchu

Przygotowano obliczeniowy moduł korzystający z surowych pozycji CAN i interpolacji
sześciu osi do wspólnego czasu, bez ekstrapolacji. Kandydat stosuje limit 250 ms
przed pierwszym SYNC i 150 ms po pierwszym SYNC. W odtworzeniu **od żądania
wykonania**, obejmującym 18 ocen co 10 ms, nie wywołał przedwczesnego STOP;
maksymalny wiek surowej pozycji w fazie uzbrajania wyniósł 166,071 ms.
Testy wstrzykniętej utraty świeżości potwierdziły odrzucenie 151 ms w ruchu
oraz 251 ms w uzbrajaniu.

To tylko sprawdzenie modułu obliczeń offline. Nie zintegrowano go z aktywnym
nadzorem i nie wykonano kolejnej próby. Pełne odtworzenie obejmujące ENABLE
ujawniło również krótki skok położenia przekraczający próg rzutu −1 mm;
zagadnienie nadzoru załączania wymaga osobnego rozstrzygnięcia. Nie wolno
deklarować gotowości całej nowej procedury na podstawie samego odtworzenia startu.

Trwałe artefakty: `log/tcp-lin-20261106-191420/`: `trial/` zawiera faktycznie
uruchomiony nadzór, surowy CAN, rzeczywisty plan, wynik, potwierdzenie postoju
i prototyp obliczeń nadzoru. `preparation_1/2/3/` zachowują wcześniejsze
przygotowania; `manifest.json` rozróżnia je od jedynego żądania wykonania.
Przed kolejną próbą wymagane są gotowa karta, sprawdzenie procedury startu
i nowa zgoda operatora. Kod produkcyjny, PID i konfiguracja nie były zmieniane.

## Zatwierdzona karta ruchu — wartości pierwotnego podglądu

- Jedna próba LIN przez produkcyjnego wykonawcę PTP/LIN, tryb `synchronized`, 100 Hz.
- Załączenie i przygotowanie J1–J6; wszystkie sześć osi uczestniczy w wykonaniu.
- Ruch TCP w układzie bazowym: **−10 mm X, +10 mm Z**, Y i orientacja narzędzia
  zachowane w granicach sprawdzonej interpolacji. Długość odcinka 14,14214 mm.
- Start XYZ z aktualnych pomiarów: **[440,00958; 0,05061; 449,98389] mm**.
- Koniec XYZ z FK rozwiązania IK: **[430,00951; 0,05084; 459,98383] mm**.
- Orientacja startowa RPY: około **[−0,05713; +0,00659; +0,00659]°**.
- Limit prędkości TCP **5 mm/s**, przyspieszenia **10 mm/s²**, obrotu **2°/s**.
  Obliczona szczytowa prędkość przesuwu 4,53750 mm/s.
- Czas trajektorii **4,02 s**; termin całej próby od żądania wykonania **10 s**,
  obejmujący uzbrojenie, dolot i potwierdzenie końca.
- Koniec próby: potwierdzony postój z podtrzymaniem ENABLE. Ruch powrotny
  i powtórzenie wymagają osobnej karty i zgody.

| Oś | Start [°] | Koniec [°] | Zmiana [°] |
| --- | --- | --- | --- |
| J1 | +0,006592 | +0,006898 | +0,000306 |
| J2 | +0,002197 | −2,250533 | **−2,252730** |
| J3 | −0,002197 | +0,303485 | **+0,305682** |
| J4 | −0,057129 | −0,011211 | +0,045918 |
| J5 | +0,006592 | +1,953637 | **+1,947045** |
| J6 | 0,000000 | −0,045924 | −0,045924 |

Główne ruchy wykonują J2, J3 i J5. Udział J1 jest mniejszy od jednego impulsu
enkodera przegubu przy przełożeniu 10; nie jest to próba sześciu dużych ruchów.

## Zatrzymanie i warunki startu

Istniejące zabezpieczenia C++ pozostają aktywne. Dodatkowy obserwator zleci
globalny STOP przez `/api/v1/motion/stop`, gdy wystąpi:

- odchylenie poprzeczne TCP od prostej **>3 mm**;
- wyjście rzutu TCP poza odcinek rozszerzony o **1 mm** z każdej strony;
- wyjście dowolnego kąta poza obliczony zakres danej osi rozszerzony o **0,5°**;
- brak aktualnej pozycji/kalibracji, wiek pozycji w statusie ≥150 ms, awaria,
  zmiana rewizji, ramka błędu CAN lub utrata danych obserwatora;
- przekroczenie **10 s** od żądania wykonania albo 90 s uzbrojonego nadzoru.

Obserwator sprawdza status nominalnie co 50 ms. Progi zlecają STOP; do ich
przekroczenia dochodzi czas wykrycia, transmisji i hamowania. Zatrzymanie
awaryjne operatora musi być dostępne podczas próby.

Przed załączeniem i wykonaniem trzeba ponownie potwierdzić sześć aktualnych
pomiarów, kalibrację, limity, zgodność modelu i rewizji oraz brak innego ruchu.
Jeżeli pozycja startowa albo wynik ponownego przygotowania zmieni zakres
zatwierdzanej karty, wymagane jest ponowne sprawdzenie przed wykonaniem.
Po przygotowaniu należy sprawdzić, że rzeczywisty program jest LIN/SYNC
z kompletem sześciu uczestników. Obecne zezwolenia na niepełny zestaw nie
upoważniają do pominięcia osi w tej próbie.

## Rejestracja i metryki TCP

Zapis obejmuje surowe ramki CAN i ich znaczniki czasu jądra, statusy osi oraz
referencję q/v/a rzeczywiście przygotowanego programu. Pasywny odbiornik nie
nadaje ramek CAN. Zapis produkcyjny CAN/ROS jest aktywny.

Po próbie pozycje 0x31 sześciu osi zostaną przeliczone przez rzeczywiste
przełożenia, kierunki i offsety. Pomiary należy interpolować do wspólnego czasu
wyłącznie tam, gdzie każda oś ma próbki po obu stronach; bez ekstrapolacji
i bez łączenia różnych generacji/segmentów. Następnie FK z bieżącego URDF da:

1. maksymalne, RMS i p95 odchylenia TCP prostopadle do prostej;
2. błąd wzdłuż drogi względem referencji w tej samej chwili;
3. rozrzut prędkości i postoje w oknach 100 ms, z jednakową filtracją;
4. odchyłkę orientacji oraz uchyby i czasowe różnice śledzenia osi;
5. odstępy F5/SYNC, braki ACK, przesunięcia czasu, wiek i kompletność pomiarów.

TCP wyliczony z enkoderów i modelu nie obejmuje ugięć, luzów ani błędów geometrii
za enkoderami. Wynik należy opisywać jako TCP z FK; niezależny pomiar fizycznej
końcówki wymaga zewnętrznego czujnika.

## Wykonane przygotowanie bez ruchu

- Odczytano bieżący model i sześć pozycji, bez wartości zastępczych.
- Planer C++ obliczył osobno podgląd i fizyczny LIN. Oba mają czas 4,02 s;
  fizyczny plan ma 18 węzłów q/v/a. Nie przekazano go jeszcze do wykonania.
- Niezależnie przeliczono FK i kwintyczną interpolację co 1 ms (4021 próbek).
  FK startu zgadza się z API C++ do 1e−8 mm. Wszystkie kąty mieszczą się
  w zapisanych limitach. Maksymalna odchyłka prostej referencji:
  **0,001506 mm**, zmiana orientacji **0,000683°**. To wyniki obliczeń.
- Przygotowano obserwator i sprawdzono rozróżnienie odchyłki 2 mm / 4 mm
  przy progu 3 mm. Jego domyślne uruchomienie wykonuje tylko odczyt.
- Pasywny odbiornik zweryfikowano przez 1 s: 566 ramek, po 25–26 odpowiedzi
  pozycji na oś, bez błędu odbiornika. W obserwacji 3 s wszystkie sześć kątów
  pozostało identycznych, ENABLE=0, brak awarii.
- Odczytano parametry: mikrokrok 128 wszystkich osi; PID vFOC 220/100/270/320.
  Prąd J1–J4 i J6: 1600 mA, J5: 200 mA. Nie zmieniano tych ustawień.

Rewizja konfiguracji: `d46e996f38a24eabaadb146cb03a58c2`.
Model: `15fe34425c8a1df257acbc98b1058bd5b2b9840c0be108700d842f1711661e8f`.
Podgląd API: `39633d02de40f3598bcc3b98317601c9d27d773839dfdb6a447fce1ee2c803fa`.
Fizyczny plan obliczeniowy: `8069899e275720f9578eb0946de8cf8e4898024d0a8cf85a10f2fd7a74ab6a24`.

Artefakty lokalne: `/tmp/tcp-lin-test-20261106-_azs9i6b/`: `model.urdf`,
`physical_plan.json`, `preview_plan.json`, `validated_summary.json`,
`reference_1ms.json`, `validate_plan.py`, `kinematics.py`, `guard.py`.
Po akceptacji nadzór uruchamia się z `guard.py --arm-approved`; wykonanie
programu jest osobną operacją produkcyjnego API. Sam obserwator nie załącza
napędów ani nie rozpoczyna ruchu. Retencja katalogu `/tmp` zależy od systemu.
