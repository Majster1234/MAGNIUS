---
name: tcp-motion-quality
description: Standardowy test jakości ruchu robota w trybie planowanego ruchu TCP (12 ruchów PTP/LIN, 20/400 mm, 5/15/40 mm/s) — uruchomienie, warunki, odczyt wyników. Używaj, gdy operator prosi o test/porównanie jakości ruchu, dobór acc F5 lub innych nastaw wykonania, albo o weryfikację płynności PTP/LIN na sprzęcie.
---

# Test jakości ruchu TCP

Pełny opis: `docs/tcp_motion_quality_procedure.md`. Skrypt: `scripts/tcp_motion_quality.py`.

## Zgoda na ruch (sprawdź najpierw)

- Dopóki operator nie zgłosi, że maszyna jest **sprzęgnięta mechanicznie**, obowiązuje stała zgoda
  (08.11.2026): test można uruchomić bez osobnej karty, bo każdy przebieg zaczyna się od zerowania MKS.
  Przed startem napisz w jednym zdaniu, co uruchamiasz (wartości acc, liczba przebiegów, czas ~5 min/przebieg).
- Po sprzęgnięciu zgoda wygasa: wymagana karta ruchu i wyraźna zgoda (AGENTS.md §2).

## Uruchomienie

```bash
cd ~/Desktop/Projekty
python3 scripts/tcp_motion_quality.py                                # suchy przebieg (IK + plan 12 ruchów), bez ruchu
python3 scripts/tcp_motion_quality.py --execute                      # jeden test przy bieżącym acc
python3 scripts/tcp_motion_quality.py --execute --acc 241 100 50     # seria; każdy przebieg od zerowania MKS
python3 scripts/tcp_motion_quality.py --execute --acc exec           # acc z parametrów wykonania (pole puste)
```

Serie nastaw regulatora wykonania (poza konfiguracją robota, przywracane po serii):
`--execute --baseline --tune lookahead_ms=120,160 --tune ff=1.1,1.2 --tune kp=1.5,4` (klucze: kp, ff,
lookahead_ms, ki, ki_limit_deg). Ręcznie: `POST /api/v1/motion/execution-tuning` (`{"reset": true}` = plik).

Długą serię uruchamiaj w tle (`run_in_background`) z przekierowaniem do pliku w scratchpadzie i śledź
zdarzenia `run_start|zeroed|move|move_failed|aborted|PRZERWANO`. Jeden przebieg trwa ok. 5 min.
Skrypt kończy z ENABLE=0. `--acc` zapisuje `motion.f5_acceleration_code` w konfiguracji ROS i ta wartość
zostaje po teście — na koniec przywróć wartość wskazaną przez operatora.

## Warunki wstępne

- Usługa `robot-web-status` z nagrywaniem (`--record`, pliki `log/manual-can/can-*.jsonl`), tryb PTP/LIN
  `synchronized` (analiza czyta `sync_feedback`), API pod `http://192.168.1.69:8080` (zmienna `ROBOT_API`).
- Brak awarii; konfiguracja niezablokowana. Po przerwanym teście skrypt sam potwierdza odniesienie.
- Po zmianach w C++ upewnij się, że usługa została zrestartowana (restart z `sudo` wykonuje operator).

## Wyniki

`log/tcp_quality/<czas>/summary.md` (tabela), `results.json` (pełne metryki, uchyb osi), `events.jsonl`.
Wskaźniki: tętnienie prędkości TCP [mm/s, % szczytu] (główna miara szarpania; zawiera szum różniczkowania
enkodera — porównuj nastawy między sobą, nie wartości bezwzględne), zatrzymania [%], odchyłka drogi TCP
max/RMS, błąd końcowy, czas planowany/rzeczywisty, korekty dolotu. Ruch niezaliczony (np. brak postoju
po dolocie) jest zapisywany i test idzie dalej; >3 niezaliczone lub awaria sprzętowa przerywa serię.
Wnioski dopisz do `docs/benchmark_results.md` (sekcja z datą, wartości acc, średnie, katalog wyników).

## Znane zjawiska

- Zerowanie MKS wymaga ENABLE=1 (oś przygotowana), potwierdzenie odniesienia — ENABLE=0; sterownik C++
  kończy sekwencje ENABLE z opóźnieniem, skrypt ponawia żądania (409 / „potwierdzony postój”).
- J5 (prąd 200 mA) po minimalnym przestrzale potrafi utknąć przy 0,125 RPM (F1=4 przez 3 s →
  „Brak potwierdzenia postoju po dolocie”).
- Awaria „SYNC: brak budżetu całego zestawu” była błędem rozliczania pasma (rezerwa `mandatory_cycle_bits`
  i niewyrównane okno budżetu → `sync_batch_window`, 08.11.2026); jeśli wraca, sprawdź, czy usługa działa
  na nowym buildzie (`/proc/<pid>/maps` procesu `can_diagnostics`), i przeanalizuj bity w oknie przed `bus_fault`.
- „Plan przekracza limit podglądu” — plan ma ~1 KiB na 10 ms ruchu; od 08.11.2026 limit mostu to 16 MiB
  (PTP 400 mm przy 5 mm/s ≈ 8,2 MiB). Przy błędzie z „8 MiB” usługa działa na starym kodzie mostu.
- Ucieczka osi (napęd jedzie wbrew F5, dotąd J2 przy LIN 400 mm 40 mm/s, ~1 na 12 wykonań): od 08.11.2026 nadzór
  SYNC przerywa ruch przy 4° lub ruchu przeciwnym do referencji i podaje w komunikacie błąd kąta 39h — zapisz go
  w `docs/motion_hardware_results.md`. Testowe nadpisanie acc (`--fixed acc=0`, `--tune acc=…`) działa poza konfiguracją.
