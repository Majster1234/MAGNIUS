# Repozytorium Git i publikacja na GitHubie

Katalog główny `Projekty` jest lokalnym repozytorium Git z gałęzią `main`.
Obejmuje dokumentację, pliki sprzętowe, skrypty i źródła czterech pakietów ROS.
Zbudowany workspace pozostaje lokalnie i nie jest częścią historii.

## GitHub Desktop

1. Uruchom GitHub Desktop i zaloguj się na swoje konto.
2. Wybierz **File → Add Local Repository** i wskaż folder `Projekty`.
3. Wybierz **Publish repository**.
4. Podaj nazwę repozytorium, np. `robot-6dof-ros2`, opis i wybraną widoczność.
5. Zatwierdź publikację przyciskiem **Publish repository**.

Dodajesz istniejący folder; nie twórz kolejnego repozytorium wewnątrz `ros2_ws`.
Dodanie lokalnego repozytorium do programu opisuje
[instrukcja GitHub Desktop](https://docs.github.com/en/desktop/adding-and-cloning-repositories/adding-a-repository-from-your-local-computer-to-github-desktop).

## Publikacja z terminala

Alternatywnie utwórz na GitHubie puste repozytorium bez dodatkowego README,
`.gitignore` i licencji, ponieważ pliki projektu są już przygotowane lokalnie.
Następnie, z katalogu głównego projektu, użyj poniższych poleceń po zastąpieniu
`TWOJ_LOGIN` i `NAZWA_REPO` rzeczywistymi wartościami:

```bash
git remote add origin https://github.com/TWOJ_LOGIN/NAZWA_REPO.git
git push -u origin main
```

Te czynności wymagają zalogowania lub skonfigurowanego uwierzytelniania GitHub.
Samo przygotowanie folderu nie tworzy zdalnego repozytorium ani nie wysyła kodu.
Po publikacji przez GitHub Desktop zdalne repozytorium jest już skonfigurowane;
nie dodawaj drugi raz `origin`.

## Pliki pomocnicze

- `.gitignore` pomija wyniki budowania, lokalne ustawienia, nagrania i pliki
  uwierzytelniające; przykładowe konfiguracje `.example.yaml` pozostają w repozytorium.
- `.gitattributes` ustala końce linii LF i sposób traktowania zasobów binarnych.
- `.editorconfig` ustala kodowanie UTF-8 i podstawowe formatowanie.
- `CONTRIBUTING.md` opisuje sposób pracy i lokalne sprawdzanie projektu.
- `.github` zawiera szablony zgłoszeń, pull requestów oraz workflow CI.

## Automatyczne sprawdzanie

Po wysłaniu kodu na `main`, otwarciu pull requestu lub ręcznym uruchomieniu
workflow GitHub Actions sprawdzi składnię Bash, XML i YAML oraz zbuduje workspace
i uruchomi dostępne testy. Konfiguracja używa Ubuntu 22.04 i kontenera
`ros:humble-ros-base-jammy` z
[oficjalnej listy obrazów ROS](https://github.com/docker-library/official-images/blob/master/library/ros).

CI nie wymaga fizycznego robota ani sesji graficznej. Obecne pakiety są szkieletami,
więc budowanie sprawdza pakiety i instalację zasobów; nie testuje sterowania osiami.
Wynik zdalnego workflow będzie widoczny w zakładce **Actions** po publikacji.

## Licencja i metadane

Zgodnie z decyzją właściciela wybór licencji jest odłożony i nie ma pliku
`LICENSE`. Metadane opiekuna oraz pole `TODO` licencji w `package.xml`
pozostają do uzupełnienia przed wydaniem pakietów ROS.
