# Kopie zapasowe nastaw sterowników MKS (07.11.2026)

Żadna nastawa sterownika MKS SERVO42D nie może być zmieniona bez planu awaryjnego przywracania.
Mechanizm ma trzy kontenery, przypisane do adresów CAN: **J1 = adres 1, …, J6 = adres 6**.
Przywracanie zapisuje do każdego adresu wyłącznie jego własne nastawy.

| Kontener | Plik (`~/.local/state/robot/drive-backups/`) | Zasady |
| --- | --- | --- |
| **base** — nastawy podstawowe | `base.json` | Tworzony **jeden raz** z odczytu napędów. Plik tylko do odczytu (0400), zapis bez nadpisywania (`link`), suma SHA-256. API nie pozwala go zmienić; nową bazę można utworzyć tylko po ręcznym usunięciu pliku przez operatora. |
| **pre_test** — stan przed testami | `pre_test.json`, poprzednie w `archive/` | Nowa kopia archiwizuje poprzednią. Zapis nastaw testowych wymaga, by bieżący stan napędów był identyczny z tą kopią. |
| **test** — nastawy testowe | `test.json` | Tylko dozwolone nastawy z walidacją zakresów. |

Dziennik każdej operacji i każdego zapisu (przed/po): `audit.jsonl`.

## Zakres kopii (katalog `mks_servo_can/drive_parameters.cpp`, instrukcja CAN v1.0.9)

Odczyt uniwersalny `00h kod [sub]`; napęd zwraca parametr w formacie zapisu albo `FFh FFh` (brak odczytu w firmware —
nastawa trafia na listę `unsupported`).

- **Przywracane** (zapis tylko różnic, kolejność): tryb 82h, mikrokrok 84h, prąd roboczy 83h, prąd trzymania 9Bh,
  PID vFOC 96h/00–01, PID CLOSE 97h/00–01, ochrona przeciążeniowa 88h, ochrona błędu pozycji 9Dh, próg osiągnięcia
  pozycji 95h, heartbeat 98h, poziom EN 85h, kierunek 86h, wygaszanie ekranu 87h, interpolacja 89h, blokada klawiszy 8Fh,
  grupowy ID CAN 8Dh, tryb IN_1 9Fh, ID użytkownika 42h.
- **W kopii i porównaniu, bez zapisu** (blokada w C++): bitrate 8Ah, brak weryfikacji CRC 0Fh (zerwałyby komunikację),
  tryb odpowiedzi 8Ch i SYNC 4Ah (zarządza nimi sterowanie), bazowanie/krańcówki 90h/94h/9Ah/9Eh (9Ah może ustawić
  punkt zero, 90h wymaga bazowania), autostart FFh, dzielnik impulsów 99h (57D).
- **Poza katalogiem:** adres CAN 8Bh, kalibracja enkodera 80h, przywrócenie fabryczne 3Fh, restart 41h.

## Zasady bezpieczeństwa (egzekwowane w C++ `CanBusManager::submit_parameter_job`)

- Odczyt i zapis tylko bez aktywnego sterowania, awarii/odzyskiwania, skanu, operacji ręcznej i zapisu konfiguracji.
- **Zapis tylko przy potwierdzonym ENABLE=0 danej osi.** Wyjątek (decyzja operatora 07.11.2026): PID 96h/97h także przy
  ENABLE=1, bez aktywnego sterowania i operacji ręcznej — strojenie bez cykli wyłączania osi obciążonych grawitacją.
- Kolejne kandydaty testowe po jednej kopii `pre_test`: zgodność z `pre_test` sprawdzana poza nastawami objętymi bieżącym
  kontenerem `test` (inna różnica blokuje zapis).
- Każdy zapis kończy się odczytem kontrolnym; różnica = błąd. Zapis unieważnia zapamiętane w sterowniku odczyty
  (tryb, mikrokrok, prąd, PID, heartbeat) i przygotowany program PTP/LIN.
- Nieudany zapis nastaw testowych → automatyczny powrót dotkniętych osi do `pre_test`.

## Procedura testu nastaw

1. `base` istnieje (jednorazowo `capture_base`).
2. Wyłącz ENABLE osi, których dotyczy test.
3. `capture_pre_test` → `PUT test` → `apply_test` (odmowa, gdy stan ≠ `pre_test`).
4. Test (osobna karta i zgoda operatora).
5. Powrót: `restore` z `pre_test` (lub `base`) → wynik zawiera `remaining_differences` (puste = zgodne).

## API (`robot_web_bridge`)

```bash
B=http://192.168.1.69:8080/api/v1/drive-backup
curl -s $B                                                     # kontenery + stan operacji
curl -s -X POST $B/operations -d '{"action":"capture_base"}'   # jednorazowo
curl -s -X POST $B/operations -d '{"action":"capture_pre_test"}'
curl -s -X PUT  $B/test -d '{"axes":{"2":{"work_current_ma":2000,"vfoc_kp_ki":[300,120]}}}'
curl -s -X POST $B/operations -d '{"action":"apply_test"}'
curl -s -X POST $B/operations -d '{"action":"verify","kind":"base"}'
curl -s -X POST $B/operations -d '{"action":"restore","source":"pre_test","axes":[2]}'
```

Operacje są asynchroniczne (202); wynik w `operation` z `GET`. Nastawy testowe: `work_mode` (4/5),
`work_current_ma` (1–3000), `hold_current_ratio` (0–8), `microstep` (16/32/64/128), `subdivision_interpolation`,
`stall_protection` (0/1), `vfoc_kp_ki`/`vfoc_kd_kv`/`close_kp_ki`/`close_kd_kv` ([a, b] 0–1024),
`position_error_protection` ([enable, czas×15 ms, błędy]), `position_reached_threshold` ([enable, próg]).

## Strojenie i weryfikacja PID (`scripts/mks_pid_tuning.py`)

Po zmianie mechaniki, jedna oś (adres CAN = numer osi), tryb SR_vFOC. Domyślnie tylko preflight i karta testu;
ruch wyłącznie z `--execute-approved` po zgodzie operatora.

```bash
python3 scripts/mks_pid_tuning.py tune   --axis 4                     # karta + preflight
python3 scripts/mks_pid_tuning.py tune   --axis 4 --execute-approved  # strojenie (po zgodzie)
python3 scripts/mks_pid_tuning.py verify --axis 4 --execute-approved  # weryfikacja zmiany z kontenera test
```

- Kopia `pre_test` automatycznie przed pierwszym zapisem; zapisy przez `apply_test`; przy przerwaniu, odrzuceniu
  lub braku poprawy automatyczny powrót do `pre_test` (odczyt kontrolny).
- Ocena = programy PTP tylko badanej osi: ±2° i ±8° od startu z powrotem (8 ruchów), 10°/s, 50°/s², obserwacja postoju
  1,5 s; pomiar ze śladu wału. Wskaźnik na ruch: 1·(czas ustalenia/0,5 s) + 2·(przeregulowanie/ε) + 3·(oscylacja p-p/ε)
  + 3·(uchyb ustalony/ε) + 0,5·(RMS śledzenia/ε), ε = 0,05° przegubu. Twarde odrzucenie: oscylacja p-p > 0,15°,
  przeregulowanie > 0,5°, brak ustalenia, awaria/nieudany program (w tym blokada przygotowania przez brak postoju).
- Przeszukiwanie: współrzędne Kp → Ki → Kd → Kv, mnożnik 1,4 (potem √), granice 0,5–2× `base`, maks. 20 kandydatów;
  poprawa przyjmowana powyżej połowy marginesu szumu (margines = max(5 %, 2× różnica dwóch linii bazowych)).
- Weryfikacja: przeplatana przed/po ×2; nastawa zostaje tylko przy koszcie < bazowy·(1 − margines) i braku twardych
  naruszeń. Wyniki: `log/pid-<tryb>-<data>-J<n>/` (`events.jsonl`, `report.json`).

### Tryb izolowany (domyślny, 07.11.2026)

Strojona oś jest jedyną z ENABLE=1; narzędzie wyłącza pozostałe osie (pojedynczo, przy potwierdzonym postoju) i włącza
strojoną po utworzeniu kopii `pre_test`. Program PTP jest przygotowywany z `participating_axes` = [oś]
(`POST /api/v1/motion/prepare`, pole opcjonalne → `PrepareMotionProgram.participation_mask` → `PreparedMotionProgram::
participation_mask`): osie spoza maski nie są komendowane, nie są sprawdzane przy przygotowaniu, starcie, dolocie i SYNC,
a `tcp_verified=false`. Mostek dopuszcza maskę tylko dla PTP, gdy cel osi spoza maski jest równy pozycji startowej planu.
Inne osie nie wchodzą do oceny; jedyne zabezpieczenie: przesunięcie wyłączonej osi > 3° przerywa test.
`--no-isolated` przywraca ruch programu wszystkimi osiami.

Potwierdzenie postoju po dolocie (|RPM| ≤ 1, F1=1) ma limit `kSettleTimeout` = 3 s — po nim awaria z opisem osi
(RPM, F1) zamiast bezterminowego oczekiwania (07.11.2026: napęd w F1=3 zawiesił program na ≥ 29 s). W strojeniu taka
awaria jest odrzuceniem kandydata: odzyskiwanie, ponowne ENABLE strojonej osi, powrót do najlepszego PID.
