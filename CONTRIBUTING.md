# Praca nad projektem

Projekt używa ROS 2 Humble na Ubuntu 22.04. Przygotowanie narzędzi opisuje
[instrukcja środowiska](docs/setup.md), a podział pakietów
[architektura](docs/architecture.md).

## Zmiany

Twórz gałąź dla danego zadania, np. `feature/robot-model` lub `fix/can-timeout`.
W opisie commitu podaj konkretną zmianę. Pull request powinien opisywać efekt,
sposób sprawdzenia i dane sprzętowe istotne dla zmiany.

Kod i zasoby ROS umieszczaj w odpowiednim pakiecie `ros2_ws/src`.
Schematy i pliki CAD należą do `hardware`, a dokumentacja do `docs`.
Zachowuj nazwy osi `joint_1`–`joint_6`, jednostki SI i zgodność z API Humble.

Dodawaj zależności do `package.xml` wraz z implementacją, która z nich korzysta.
Nie zastępuj nieznanych parametrów napędów fikcyjnymi wartościami.

## Sprawdzenie przed commitem

Z katalogu głównego repozytorium:

```bash
bash -n start_ros2.sh scripts/ros2_terminal.bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

Testy komunikacji powinny korzystać z ramek referencyjnych i sprawdzać błędy
protokołu. Testy wymagające fizycznego robota opisuj oddzielnie od testów
uruchamianych automatycznie. Obecny szkielet nie zawiera testów wykonywalnych
ani sterownika napędów.

## Zawartość commitów

Dodawaj kod źródłowy, konfiguracje przykładowe i dokumentację. Wyniki budowania,
lokalne ustawienia narzędzi, pliki uwierzytelniające i nagrania ROS są pomijane
przez `.gitignore`. Polecenie `git diff --cached` pozwala obejrzeć zawartość
przygotowanego commitu.

Wybór licencji został odłożony. Repozytorium nie zawiera pliku `LICENSE`,
a metadane opiekuna i licencji w manifestach ROS nadal wymagają uzupełnienia.
