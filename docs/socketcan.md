# Integracja U2C v2.1 z Linuksem

## Sterownik i prędkość

Adapter podłączony do tego komputera został wykryty jako `can0`, ze sterownikiem
jądra `gs_usb`. Sterownik jest już częścią zainstalowanego kernela Ubuntu;
nie wymaga instalowania zewnętrznego modułu ani kompilacji. Firmware adaptera
udostępnia interfejs zgodny z `gs_usb`; nie wykonywano jego aktualizacji.

Użytkownik przestawił napędy na **1000000 bit/s**; host przełączono i sprawdzono
24.10.2026 ([raport komunikacji](can_1m_results.md)). Ta wartość musi być
jednakowa dla wszystkich urządzeń fizycznej magistrali. Konfiguracja dotyczy
Classical CAN; nie włącza CAN FD ani automatycznego resetowania bus-off.

Podział warstw opisuje [docs/can_protocol.md](can_protocol.md), a systemowe API
[dokumentacja SocketCAN jądra Linux](https://docs.kernel.org/networking/can.html).

## Instalacja konfiguracji

Z katalogu głównego repozytorium, z podłączonym U2C:

```bash
./scripts/install_socketcan.sh --bitrate 1000000 --txqueuelen 1000 --dry-run
sudo ./scripts/install_socketcan.sh --bitrate 1000000 --txqueuelen 1000
```

Skrypt instaluje pakiet `can-utils`, jednostkę `/etc/systemd/system/robot-can@.service`
i plik `/etc/robot-can/can0.conf` z `BITRATE=1000000` i `TX_QUEUE_LEN=1000`. Dotychczasowe wersje tych
plików są zachowywane z przyrostkiem `.bak-...` przy ponownej instalacji.
Zmiana plików w repozytorium nie aktualizuje automatycznie kopii w `/etc`.

Usługa `robot-can@can0.service` jest włączana jako zależność urządzenia
`sys-subsystem-net-devices-can0.device`. Ustawia bitrate i podnosi interfejs po
pojawieniu się urządzenia, również przy starcie komputera. Odłączenie USB zatrzymuje
usługę; ponowne podłączenie jako `can0` uruchamia ją ponownie.
Mechanizm opiera się na [jednostkach urządzeń systemd](https://www.freedesktop.org/software/systemd/man/latest/systemd.device.html).

Usługa sprawdza sterownik `gs_usb` przed konfiguracją. Nie włącza
`systemd-networkd` i nie zmienia konfiguracji Wi-Fi ani Ethernetu w NetworkManagerze.
Nie trzeba otwierać terminali ROS, aby system skonfigurował CAN.

Nazwa `can0` odpowiada obecnemu pojedynczemu adapterowi. Przy dodaniu kolejnych
adapterów CAN należy osobno ustalić trwałe nazwy na podstawie numerów seryjnych;
ta konfiguracja nie przypisuje nazwy do numeru seryjnego konkretnego egzemplarza.

## Sprawdzenie i obsługa

```bash
ip -details -statistics link show can0
systemctl status robot-can@can0.service --no-pager
journalctl -u robot-can@can0.service -b --no-pager
candump can0
```

W `ip` sprawdź flagę `UP`, `bitrate 1000000` i stan kontrolera CAN, np.
`ERROR-ACTIVE`. Ogólny stan linku `UNKNOWN` jest normalny dla CAN. `candump`
czeka na ramki; brak wydruku nie potwierdza ani nie wyklucza działania napędów.
Przerwij podgląd przez `Ctrl+C`. Narzędzia pochodzą z
[projektu linux-can/can-utils](https://github.com/linux-can/can-utils).

Po uruchomieniu interfejsu aplikacja może otworzyć socket CAN i używać nazwy
`can0`. Konfiguracja linku wymaga administratora, ale typowy odczyt SocketCAN
nie wymaga uruchamiania całej aplikacji ROS jako root.

Do ręcznego zatrzymania lub ponownego uruchomienia konfiguracji:

```bash
sudo systemctl stop robot-can@can0.service
sudo systemctl start robot-can@can0.service
```

Do wyłączenia automatycznej konfiguracji i zatrzymania interfejsu:

```bash
sudo systemctl disable --now robot-can@can0.service
```

Skrypt nie wysyła komend do napędów. Testy protokołu MKS i ruchu osi pozostają
osobnym etapem projektu.

## Profil strumieniowania sześciu osi

Szablony repozytorium wskazują teraz BITRATE=1000000 i TX_QUEUE_LEN=1000.
Pętla 100 Hz wysyła F5 dla sześciu osi oraz sześć zapytań31, a napędy zwracają
pozycje. Wraz z diagnostyką to modelowo26,58% przy 1M,53,16% przy 500k.
Przy wymaganej puli robota 40% pełne sześć osi wymaga1 Mbit/s. Rezerwa przyszłego
modułu20 punktów jest osobna. Szczegóły: [wykonanie PTP](motion_execution.md).

Fizyczny interfejs ma potwierdzone 1000000 bit/s, TX=1000; cztery napędy
odpowiadają bez błędów w teście odczytów. Samo uruchomienie
procesu ani edycja YAML nie zmienia ustawień linku. SocketCAN odczytuje rzeczywisty
bitrate przez Netlink, a manager odrzuca niezgodny profil przed wysłaniem komend.

Jednostka ustawia txqueuelen przed podniesieniem interfejsu. Większa kolejka
łagodzi chwilowe ENOBUFS, nie powiększa przepustowości i nie gwarantuje latencji.
Biblioteka odbiera ramki osobnym wątkiem poll 1 ms, z atomowym cache pozycji31.
CanRSP=Disable wyłącza odpowiedzi ruchu; odczyty i konfiguracja nadal odpowiadają.
Wymagany tryb CAN: **SR_vFOC**, bez SYNC 4B (4A=0). Przy zmianie instalacji
nie należy uruchamiać dwóch właścicieli poleceń CAN. Restart wymaga osobnej zgody.

Historyczny odczyt przed wdrożeniem 24.10.2026: `can0` 500000 bit/s, `txqueuelen` 10; PID usługi
22808 i licznik restartów 2 bez zmiany. Szablon 1000000/1000 nie jest
ówczesną konfiguracją. Stan po wdrożeniu poniżej.

Pierwsze wdrożenie 24.10.2026: nowa jednostka jest zainstalowana, TX=1000,
bitrate nadal 500000. Sterowniki/host nie były przełączane na 1 Mbit/s. Wdrożenie
wykonał użytkownik przez scripts/deploy_streaming100.sh; końcowa usługa jest
aktywna. [Wyniki prób 1/2/4 osi](motion_hardware_results.md).

Późniejszą migrację hosta i napędów na 1 Mbit/s opisuje
[raport komunikacji](can_1m_results.md); nie zmienia ona historycznych prób ruchu.
