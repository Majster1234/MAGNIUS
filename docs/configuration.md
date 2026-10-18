# Strona konfiguracji

W menu panelu wybierz **Konfiguracja** albo otwórz `/#configuration`.
Widok desktop korzysta ze wspólnego ciemnego motywu i nagłówka u góry.

**Ustawienia systemowe** pozostają puste. Poprzednie pola sieci, CAN i logów
usunięto. Nowe ustawienia będą dodawane na konkretne polecenie użytkownika.

**Parametry robota** przedstawia zwarta tabela danych maszynowych.
Wiersze oznaczają parametry, a kolumny J1–J6 odpowiadają CAN 1–6.
Suwak nad tabelą przełącza dwa widoki:

- **Geometria:** translacja X/Y/Z i rotacja X (Roll), Y (Pitch), Z (Yaw)
  względem poprzedniego członu;
- **Przełożenia i ograniczenia:** przełożenie silnik/przegub, kierunek,
  offset zera, dolna i górna pozycja graniczna, maksymalna prędkość i przyspieszenie.

Suwak działa także z klawiatury: strzałki oraz Home/End. Przełączenie nie usuwa
danych drugiego widoku. Tabela z suwakiem i opisem zajmuje najwyżej połowę
dostępnej wysokości pod nagłówkiem konfiguracji; przy mniejszej wysokości ma
własne przewijanie i nieruchomy wiersz nagłówków. Sześć osi mieści się w poziomie.

Pod tabelą są domyślnie zwinięte sekcje **Dane robota** oraz **Narzędzie i TCP**.
Zachowują wcześniejsze pola nazw układów odniesienia, masy, TCP i środka masy.

Parametry mechaniczne używają SI: m, rad, rad/s, rad/s² i kg. Przełożenie jest
liczbą obrotów silnika na obrót przegubu. Limity przegubów są odrębne od limitów
wału ±3600° w ręcznym JOG. Nieznane wartości pozostają puste, także przełożenia,
kierunki i geometria. Zero wpisane przez użytkownika jest wartością, a puste
pole oznacza brak danych. Konwencja geometrii wymaga ustalenia przed jej zastosowaniem w modelu.

## Zakres szkieletu

Pola można edytować. Szkic jest automatycznie zachowywany w `sessionStorage`
tej karty przeglądarki, oddzielnie dla osi, danych ogólnych i narzędzia. Przetrwa
nawigację oraz odświeżenie; nie jest współdzielonym plikiem konfiguracji robota.
Po zamknięciu karty nie należy polegać na jego zachowaniu. Blokada pamięci
przeglądarki jest sygnalizowana; uszkodzony zapis daje pusty szkic.
Liczba pól jest stała, tekst ograniczony do 256 znaków; brak historii zmian.
Wcześniejsze szkice parametrów robota zachowują zgodność z tabelą; usunięte
klucze ustawień systemowych są pomijane przy odczycie i kolejnym zapisie szkicu.

Ten etap nie udostępnia API zapisu ani zastosowania parametrów. Wpisanie danych
nie zmienia sieci, CAN, napędów, aktywnych limitów, URDF ani parametrów ROS.
Nie ma przycisku pozorującego wdrożenie. Przed podłączeniem zapisu trzeba ustalić
konwencję geometrii, walidację kompletności i zakresów, wersjonowanie konfiguracji,
konwersję do plików robota oraz warunki bezpiecznego zastosowania ustawień.

## Sprawdzenia

2026-10-18: TypeScript, produkcyjny build i 12 istniejących testów Vitest OK.
Test Chromium konfiguracji sprawdza pustą zakładkę systemu, parametry w wierszach,
sześć osi w kolumnach, suwak widoków, niezależność wartości, rozróżnienie zera
i braku danych, zwinięte pola narzędzia, zgodność wcześniejszego szkicu,
nawigację i odświeżenie, uszkodzony zapis oraz brak żądań zapisujących do backendu.
Przy 1280×800, 1366×768 i 1920×1080 wysokości panelu z tabelą wyniosły 259 px
(geometria) i 287 px (ograniczenia), każdorazowo poniżej połowy wolnej wysokości,
bez poziomego przewijania tabeli. Test należy do `npm run test:browser` w CI.
Dotychczasowy test przeglądarki ręcznego sterowania też przeszedł.

Oddzielnie otwarto stronę w działającym panelu LAN: pusta zakładka systemu,
oba widoki tabeli z suwakiem i ich wysokość, bez błędów JavaScript i żądań zapisu.
Nie wykonywano testów ruchu, zapisu konfiguracji serwera ani ponownego builda ROS.
