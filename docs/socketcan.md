# Integracja U2C v2.1 z Linuksem

## Sterownik i prędkość

Adapter podłączony do tego komputera został wykryty jako `can0`, ze sterownikiem
jądra `gs_usb`. Sterownik jest już częścią zainstalowanego kernela Ubuntu;
nie wymaga instalowania zewnętrznego modułu ani kompilacji. Firmware adaptera
udostępnia interfejs zgodny z `gs_usb`; nie wykonywano jego aktualizacji.

Użytkownik potwierdził prędkość napędów **500000 bit/s**. Ta wartość musi być
jednakowa dla wszystkich urządzeń fizycznej magistrali. Konfiguracja dotyczy
Classical CAN; nie włącza CAN FD ani automatycznego resetowania bus-off.

Podział warstw opisuje [docs/can_protocol.md](can_protocol.md), a systemowe API
[dokumentacja SocketCAN jądra Linux](https://docs.kernel.org/networking/can.html).

## Instalacja konfiguracji

Z katalogu głównego repozytorium, z podłączonym U2C:

```bash
./scripts/install_socketcan.sh --bitrate 500000 --dry-run
sudo ./scripts/install_socketcan.sh --bitrate 500000
```

Skrypt instaluje pakiet `can-utils`, jednostkę `/etc/systemd/system/robot-can@.service`
i plik `/etc/robot-can/can0.conf` z `BITRATE=500000`. Dotychczasowe wersje tych
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

W `ip` sprawdź flagę `UP`, `bitrate 500000` i stan kontrolera CAN, np.
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
