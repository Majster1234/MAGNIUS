# Diagnostyka — trace pozycji wałów

Moduł **Diagnostyka** (`#diagnostics`) jest dostępny z menu i kafelka przeglądu.
Pokazuje równocześnie J1–J6 / CAN 1–6, w stopniach rzeczywistego wału. Przełożenie,
offset przegubu i ENABLE nie zmieniają zapisywanej pozycji. Nie jest to sygnał
zadany ani estymata ruchu. Dokładność prezentacji 0,01° nie jest deklaracją
dokładności mechanicznej enkodera.

## Obsługa wykresów

- Pinezka **„Przypnij wykres J…”** przy wybranej osi odłącza pojedynczy
  wykres do pływającego okna. Pozostałe pięć osi zostaje w Diagnostyce.
  Przypięcie innej osi zastępuje tę w oknie, a poprzednia wraca do modułu.
  Jednocześnie przypięty jest jeden wykres; poza Diagnostyką widoczny jest tylko on.
  Przypięcie działa do zamknięcia okna lub odświeżenia strony.
- Okno pozostaje na ekranie przy zmianie modułu. Korzysta z tego samego połączenia
  trace i bufora historii. Zakres czasu i kursor pozostają wspólne z Diagnostyką;
  każda oś zachowuje własną skalę Y. Zmiana rozmiaru okna powiększa też sam wykres.
- Pasek tytułu służy do przesuwania okna, prawy dolny narożnik do zmiany rozmiaru.
  Ikony u góry pozwalają wrócić do Diagnostyki, maksymalizować/przywrócić rozmiar
  i zamknąć okno. Zamknięcie pozostawia wybrany moduł; wykres wraca do Diagnostyki,
  a rejestrator nadal działa. Położenie i rozmiar są zapamiętywane lokalnie.
  Uchwyty obsługują też strzałki klawiatury (10 px, z Shift 50 px).
- Sterowanie czasem i podglądem pozostaje u góry pływającego okna podczas
  przewijania zawartości. Przeciąganie samego wykresu nadal zmienia czas,
  niezależnie od położenia okna.
- Przeciąganie dowolnego wykresu lewym przyciskiem myszy przesuwa wspólny czas
  wszystkich sześciu osi i wstrzymuje przewijanie na żywo. Przeciągnięcie w prawo
  odsłania starsze próbki, w lewo — późniejsze. Sam klik nie zatrzymuje podglądu.
- Pozioma rolka myszy nad wykresem przesuwa wspólny czas bez widocznego suwaka:
  w lewo do starszych próbek, w prawo do późniejszych. Przewijanie wstrzymuje
  podgląd na żywo, a przycisk „Na żywo” przywraca bieżące 10 s. Działa także
  w przypiętym wykresie po przejściu do innego modułu. Ruch odpowiada pikselom
  wykresu (jedna jego szerokość to jedno okno czasu); zdarzenia w liniach i stronach
  są przeliczane na tę samą skalę. Rolka pionowa, gesty z przewagą kierunku
  pionowego oraz Ctrl/Meta + rolka zachowują natywne działanie przeglądarki.
- Początkowe okno obejmuje ostatnie 10 s. Pole czasu i ikony lup ± zmieniają jego
  szerokość, strzałki przesuwają je o połowę. Ikona dopasowania obejmuje całą
  zachowaną historię. Ikona transmisji „Na żywo” resetuje przesunięcie i szerokość
  okna do bieżących 10 s; zachowuje skale Y. Przyciski mają podpowiedzi i etykiety
  dostępne dla czytników ekranu, a aktywne „Na żywo” jest podświetlone.
- Historia jest pobierana także poza widocznym zakresem: po jednym sąsiednim
  przedziale z każdej strony. Każdy przedział ma szerokość bieżącego okna czasu.
  Przesuwanie korzysta z gotowych próbek i doczytuje kolejne w trakcie ruchu myszy,
  bez czekania na jej puszczenie. Bufor przeglądarki mieści do ośmiu przedziałów,
  po maksymalnie 2000 punktów na oś w każdym; działają najwyżej dwa pobrania naraz.
  Zmiana kierunku ponownie wykorzystuje zachowany bufor. Zmiana szerokości okna
  lub ponowne połączenie unieważnia wcześniejsze pobrania.
  Bardzo szybki skok poza bufor może nadal wymagać odpowiedzi HTTP — wtedy wykres
  pokazuje „Wczytywanie historii…”. Nie łączy linii przez niepobrany fragment.
- „Wstrzymaj podgląd” zatrzymuje okno, **zapis trwa dalej**. Dostępny jest wybór
  daty i czasu końca zakresu. Zamknięcie przeglądarki nie zatrzymuje rejestratora.
- Każda oś ma Auto Y i niezależne minimum/maksimum. Te ustawienia oraz szerokość
  okna pozostają w `localStorage` przeglądarki.
- Kursor jest wspólny dla sześciu wykresów. Pokazuje dostępny pomiar, bez
  interpolowania pozycji. Przy długich zakresach kursor doczytuje mały wycinek
  oryginalnych danych. Brak pomiaru oznacza „—”.
- Linie łączą sąsiednie pomiary bez wygładzania. Zmiana segmentu, utrata próbki
  albo odstęp ponad 80 ms przerywa linię. Min/max w koszykach redukuje historię
  i zachowuje krótkie szczyty; koszyk z luką jest traktowany zachowawczo jako przerwa.
- Świeżość pozycji wygasa po 250 ms. Brak paczek źródła przez 500 ms oznacza
  utratę źródła; GUI wygasza też stan po utracie połączenia z backendem.
  Są to progi prezentacji, **nie nowe zabezpieczenia ani limity PTP**.

## Przepływ i zapis

Manager CAN przekazuje każdy poprawnie przyjęty pomiar `31H` CAN 1–6, także
potwierdzenie „zeruj wał”, do kolejki SPSC o pojemności 4096. Zachowuje surowe
ticki i indywidualny czas CLOCK_MONOTONIC oraz timestamp odbioru SocketCAN UTC
(awaryjnie CLOCK_REALTIME). Nie zapisuje dysku ani nie publikuje ROS w IO.
Nie dodaje zapytań CAN. Dane sterowania zachowują dotychczasową precyzję.

`DiagnosticsNode` opróżnia kolejkę co 50 ms, maksymalnie 256 próbek na paczkę,
publikując `/robot/diagnostics/shaft_trace` z QoS best effort / depth 20. Ta sama
implementacja działa w samodzielnej diagnostyce oraz wewnątrz `robot_hardware`.
Puste paczki są heartbeat. Przepełnienie źródła nie blokuje IO: nowa próbka
jest odrzucana, licznik rośnie, a sekwencje pozwalają wykryć lukę.

Backend uruchamia `ShaftTraceStore` zawsze, bez `--record`. Osobny wątek pisze
przez kolejkę 4096 i wykonuje `fsync` co sekundę oraz przy poprawnym zamknięciu.
Przy błędzie dysku nadal działają odbiór i podgląd bieżący. Błąd osi pozostaje
widoczny do restartu rejestratora; nie ma automatycznego kasowania pliku ani
ponawiania zapisu, które mogłoby ukryć awarię. Nagłe odcięcie zasilania może
utracić nieopróżnione dane; CRC wykrywa niepełne/uszkodzone rekordy.

Domyślny katalog to `$XDG_STATE_HOME/robot/shaft-trace`, a przy pustej lub
nieustawionej zmiennej `~/.local/state/robot/shaft-trace`. Parametr ROS bridge
`shaft_trace_directory` pozwala wskazać inny katalog bez zmian GUI.
Pliki `axis-1.bin` … `axis-6.bin` powstają dla osi, które dostarczyły pomiar.
Druga instancja nie może jednocześnie pisać do tego samego pliku (`flock`).

Każdy plik ma **20 000 000 bajtów łącznie**, bez kolejnych plików po restarcie:

| Element | Rozmiar / znaczenie |
| --- | --- |
| Obszar nagłówka | 4096 B, dwa naprzemienne nagłówki z CRC32 |
| Rekord | 56 B, ordinal, sekwencja źródła, segment, monotonic_ns, utc_ns, pozycja, flagi, CRC32 |
| Pojemność | 357 069 próbek na oś; przy 25 Hz około 3 h 58 min |

Plik jest cykliczny i może być rzadki na dysku: logiczny rozmiar od początku
obejmuje cały limit. Najstarsze próbki są nadpisywane. CRC i ordinal chronią
przed pokazaniem starego slotu jako nowego pomiaru. Uszkodzone rekordy są
pomijane z przerwą, a nie zastępowane zerem. Nieprawidłowy rozmiar lub oba
uszkodzone nagłówki zgłaszają błąd, zachowując istniejący plik do analizy.

Pozycja jest zaokrąglana **raz**, w backendzie, z 16384 ticków/obrót do całkowitych
setnych stopnia: połówki od zera, także dla wartości ujemnych. Zapis, API, wykres
i CSV używają tej samej liczby. Zerowanie, reset sesji CAN i restart managera
nadają nowe segmenty; restart bridge oznacza lukę przy wznowieniu odbioru.

Liczniki GUI rozróżniają odrzucenia kolejki źródła, luki sekwencji odbiornika
i utraty kolejki/dysku. **Nie należy ich sumować**: ta sama utrata źródła może
zostać wykryta również jako luka sekwencji. Status pokazuje zakres każdej osi,
rozmiar, błąd i świeżość. Osie bez pomiarów zachowują puste panele.

## API

Wszystkie odczyty podlegają istniejącej polityce LAN/Host/Origin panelu.
Identyfikatory 64-bitowe w JSON są ciągami dziesiętnymi; pozycja to
`position_cdeg` (całkowite setne stopnia), czas wykresu `time_ms` jest UTC.

| Odczyt | Wynik |
| --- | --- |
| `GET /api/v1/diagnostics/trace` lub `/status` | Status sześciu osi, rejestratora i kolejek |
| `GET /api/v1/diagnostics/trace/history` | Domyślnie min/max całego wybranego zakresu |
| `WS /api/v1/diagnostics/trace/ws` | Nowe próbki w kolejności odbioru i status co 50 ms |
| `GET /api/v1/diagnostics/trace/axes/{axis}/export.csv` | Surowe zachowane próbki jednej osi, bez redukcji |

Historia: opcjonalne `start_ms`, `end_ms`, `axis=1..6`, `max_points=2..2000`.
Każda oś odpowiedzi zawiera najwyżej 2000 punktów. Tryb `reduction=none` daje
strony surowych pomiarów: powtarzaj zapytanie dla wybranej osi z
`after=next_cursor`, dopóki `has_more=false`. Kursory odnoszą się do ordinali
pliku; podczas rotacji najstarsze dane mogą już zostać nadpisane. Domyślny
min/max obejmuje cały zakres w pojedynczej, ograniczonej odpowiedzi.

CSV przyjmuje ten sam zakres czasu albo całą historię przy braku parametrów.
Kolumny: `axis,sequence,segment,monotonic_ns,utc_ns,position_degrees,gap`.
Pozycja ma zawsze dwa miejsca po kropce. Eksport używa stabilnej kopii binarnej
jednej osi w RAM (do 20 MB), strumieniuje wiersze bez plików tymczasowych i
nie blokuje writer na czas pobierania przez klienta. Równoczesny kolejny
eksport może otrzymać HTTP 429. Przerwy i segmenty pozostają w danych CSV.

## Sprawdzenia

Testy pamięciowe i API obejmują 150 próbek/s, duplikaty, połowy ujemne, segmenty,
rotację pełnego pliku 20 MB ponad dwa razy, restart, równoległy zapis/eksport,
CRC, blokadę drugiego writera, symulowany ENOSPC i przepełnienie kolejki.
Test ROS sprawdza kolejność paczek także po zatrzymaniu executora przy nadal
pracującym IO. Testy Chromium używają atrap i nie wydają poleceń sprzętowych.
Wyniki i koszt CPU: [benchmark_results.md](benchmark_results.md).

2026-10-23: po zmianie przypinania na pojedynczą oś pomyślnie wykonano
build panelu, cztery testy geometrii okna oraz
`node web/test/diagnostics.browser.cjs --pinning-only`.
Scenariusz sprawdza jedną oś w oknie i pięć w module, zastępowanie przypiętej osi,
brak duplikatów i ponownego WS, zachowanie skal/wspólnego czasu, geometrię,
powrót po zamknięciu i oś bez danych. Pozostałe testy danych, cache i backendu
nie są ponawiane dla tej zmiany. Wcześniejsze wyniki pełnego trace i prefetch
pozostają w dokumentacji benchmarków.

2026-10-23: build i osobny scenariusz poziomej rolki przeszły.
`node web/test/diagnostics.browser.cjs --wheel-only` sprawdza: oba kierunki, wspólny czas,
pauza/live, jednostki zdarzeń, szybkie serie bez utraty przesunięcia, przewijanie
pionowe, przypięta oś i brak suwaka. Ten scenariusz nie uruchamia pozostałych
regresji Diagnostyki.

Pasywny odbiór, przy działającej aplikacji i zbudowanych nowych interfejsach:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/local_setup.bash
ROS_LOCALHOST_ONLY=1 python3 scripts/observe_shaft_trace.py --seconds 10 --output log/shaft-passive.json
```

Narzędzie tylko słucha `can0` i tematu ROS, porównując timestamp, oś i surowe
ticki. Nie ma funkcji nadawania CAN. Nie jest testem ruchu, płynności ani odbiorem
STOP/PTP. Flagi wykonania i ustawienia próby 5 pozostają odłożone.

Stan 2026-10-23: kod i build są przygotowane. Działająca usługa nadal używa
starszych binariów; jej restart został odrzucony przez automatyczną kontrolę
uprawnień i wymaga zgody użytkownika. Do tego czasu nowe API nie jest aktywne.
Pasywny odbiór pełnej ścieżki sprzęt → trace → CSV pozostaje do wykonania.
