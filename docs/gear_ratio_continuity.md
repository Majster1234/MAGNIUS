# Zachowanie kąta po zmianie przełożenia — 25.10.2026

Zmiana przełożenia zachowuje bieżący kąt skonfigurowanego i skalibrowanego
przegubu. Właściciel CAN przelicza offset zera na podstawie świeżej pozycji wału.
Nie przestawia licznika MKS i nie wysyła komend ruchu ani zerowania.

## Przeliczenie i warunki

```text
shaft_rad = encoder_ticks × 2π / 16384
q_old = direction_old × shaft_rad / ratio_old + offset_old
offset_new = q_old − direction_new × shaft_rad / ratio_new
```

Przykład: wał 90°, przełożenie 1:1 i offset 0° dają J1=90°. Zmiana na 3:1
ustawia offset 60°; wał i J1 pozostają na 90°. Następny 1° przegubu wymaga 3° wału.
Zachowywany jest stan sprzed następnej zmiany konfiguracji, a nie historyczna
pozycja sprzed wcześniej wykonanego ruchu. Przełożenie musi odpowiadać mechanice.

Na tej samej osi zmianę przełożenia i zmianę offsetu/kierunku albo potwierdzenie
odniesienia wykonuje się w osobnych zapisach. Zapis dostępnej osi wymaga ENABLE=0,
pozycji młodszej niż 60 ms i stabilności przez 100 ms w granicach 3 impulsów.
PREPARE zapamiętuje pozycję i segment odniesienia. COMMIT ponownie sprawdza
postój, segment i różnicę najwyżej 3 impulsów względem PREPARE. Utrata warunków
przerywa zapis. Zachowany kąt musi mieścić się w nowych limitach; granice muszą
być przedstawialne w F5 int24, a prędkość w zakresie MKS. Kąta nie obcina się
do limitów. Zapis unieważnia przygotowane programy i wymaga nowego przygotowania.

## Brak pomiaru i potwierdzenie odniesienia

Dla niedostępnej osi zapisuje się żądane przełożenie i offset, ale kalibracja
pozostaje nieważna. Stary pomiar nie zastępuje świeżego. Powrót komunikacji,
restart i zapis innych parametrów nie zdejmują blokady. Znane ENABLE=1 nadal
powoduje odrzucenie zmiany, również przy utracie pozycji.

Panel pokazuje osie wymagające odniesienia. Operator sprawdza fizyczną pozycję
i zapisany offset, zaznacza konkretne osie oraz wybiera „Potwierdź odniesienie
wybranych osi”. Operacja dotyczy aktywnego dokumentu, wymaga świeżego pomiaru,
postoju i ENABLE=0; nie stosuje niezapisanych edycji szkicu. Sam ponowny zapis
tych samych wartości nie potwierdza kalibracji. Nieważna obecna oś nadal blokuje
ruch także przy `allow_missing_motion`.

HTTP `POST /api/v1/configuration/apply` przyjmuje opcjonalne
`confirm_calibration_axes: [1, ..., 6]` — unikalne numery wybranych osi,
domyślnie pustą listę. Status konfiguracji udostępnia `calibration_invalid[6]`.
Żądanie ROS `ConfigureRobot` ma maskę `confirm_calibration[6]`, a odpowiedź
`resolved_configuration`: rzeczywiście przyjęte osie i flagi kalibracji.
Zmiana typu ROS wymaga spójnego przebudowania klientów i serwera.

Koordynator zapisuje rozstrzygnięty dokument i flagi do dziennika przed COMMIT,
następnie do aktywnego pliku i `calibration.json`. Rollback odtwarza poprzedni
profil i flagi; uzgodnienie po utracie odpowiedzi zachowuje wyliczony offset.
BOOTSTRAP używa zapisanych wartości, bez kolejnego przeliczania. Snapshot JSON
zachowuje wersję 1. Panel przyjmuje wyliczony offset bez utraty edycji wykonanych
po wysłaniu żądania. Eksport po zastosowaniu zawiera ten offset.

## Wspólny odczyt i sprawdzenia

Pozycja manualna, sprzężenie PTP/LIN i `CanSystem` korzystają z tej samej
konwersji impulsów. Atomowy stan dla `ros2_control` zawiera przełożenie, kierunek,
offset, adres, ważność kalibracji i generację konfiguracji. Szybki odczyt używa
tego profilu zamiast kopii z uruchomienia. Zmiana generacji kasuje historię
prędkości i komend; stara generacja w komendzie JTC jest odrzucana.

25.10.2026: zbudowano 11 pakietów i frontend. Zaliczone: **130 testów C++,
89 Python/ROS, 5 TypeScript i 2 scenariusze Chromium**, bez błędów.
To celowo wybrane zestawy, nie pełny test wszystkich pakietów.

Sprawdzenia obejmują niezerowe i ujemne liczniki, oba kierunki, offsety,
wielokrotne przełożenia 1/3/2,5/0,5/12,5, odwrotne przeliczenie celu, pierwszy
F5 bez skoku licznika i kolejny ruch zgodny z nowym przełożeniem, błędy limitów,
utratę pomiaru/ENABLE/odniesienia przed COMMIT, rollback i restart. Sprawdzono
selektywne potwierdzanie, trwałość blokady, rzeczywistą usługę ROS C++ z emulatorem,
koordynator z ROS/RSP i plikami, szybki odczyt pluginu, odrzucenie starych komend,
PTP/LIN/STOP oraz wcześniejszą poprawkę dokładności F5. Testy panelu sprawdzają
wyliczony offset, równoczesne edycje, potwierdzenie oraz dotychczasowe snapshoty.

Początkowe kompilacje wykryły brak include/helpera czasu w nowych testach;
poprawiono je. Pierwsze uruchomienie testu API wymagało zachowania ROS PYTHONPATH,
a Chromium uprawnień do procesów potomnych. Końcowe wskazane zestawy przeszły.
Pomiary i ograniczenia: [benchmark_results.md](benchmark_results.md).

## Wdrożenie

Build: `build/gear-ratio-continuity-ros`, instalacja: `install/gear-ratio-continuity-ros`.
Obejmuje też wcześniejszą poprawkę dokładności F5. Punkt
`install/streaming-smooth-ros/local_setup.bash` wskazuje nowy overlay.
Panel znajduje się w jego pakiecie `robot_web_bridge`; włączy się wraz z nowym
backendem przy restarcie. Zachowano poprzednie hashowane zasoby dla otwartych kart.
Kopie punktu startowego i panelu: `log/gear-ratio-continuity-20261025/deployment`.
Wyniki testów, porównań i hashe: `log/gear-ratio-continuity-20261025`.

Usługi nie restartowano. Przed/po MainPID=37208, NRestarts=0, czas aktywacji
i stan active bez zmian. Konfiguracja can0 identyczna: 1 Mbit/s, TX1000.
Nie otwierano fizycznego SocketCAN, nie wysyłano do napędów ENABLE, zerowania
ani ruchu i nie zmieniano aktywnego dokumentu robota. Nowy kod oczekuje na restart
operatora w postoju:

```bash
sudo systemctl restart robot-web-status.service
```

Po restarcie odświeżyć panel. Próba sprzętowa pozostaje osobnym uzgodnieniem osi,
kierunku, prędkości, zakresu i zatrzymania; testy emulatorów nie są jej odbiorem.
