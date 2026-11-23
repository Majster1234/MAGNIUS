"""Pomocnik izolowanych benchmarków; nie jest częścią aplikacji ROS."""

import argparse
import json
import time


def measure(prepare, iterations):
    """prepare() tworzy obiekt z cycle(index) i verify(); oba poza pomiarem."""
    state = prepare()
    cpu_start = time.process_time_ns()
    wall_start = time.perf_counter_ns()
    for index in range(iterations):
        state.cycle(index)
    wall_ns = time.perf_counter_ns() - wall_start
    cpu_ns = time.process_time_ns() - cpu_start
    # Weryfikacja musi odczytać wynik pracy; nie używamy assert (python -O).
    if state.verify() is not True:
        raise ValueError('Scenariusz zwrócił nieprawidłowy wynik')
    return {'wall_ns': wall_ns, 'cpu_ns': cpu_ns}


def execute(prepare, iterations=0, samples=7, warmup_seconds=1.0,
            min_series_ms=200.0):
    if iterations < 0 or samples < 1 or warmup_seconds < 0 or min_series_ms <= 0:
        raise ValueError('Nieprawidłowe parametry pomiaru')
    count = iterations or 1
    target = int(min_series_ms * 1_000_000)
    warmup_end = time.perf_counter_ns() + int(warmup_seconds * 1_000_000_000)
    while True:
        trial = measure(prepare, count)
        if not iterations and trial['wall_ns'] < target:
            count *= min(10, max(2, (target + max(1, trial['wall_ns']) - 1)
                                // max(1, trial['wall_ns'])))
            if count > 1_000_000_000:
                raise ValueError('Przekroczono limit liczby cykli')
            continue
        if time.perf_counter_ns() >= warmup_end:
            break
    return {'iterations': count,
            'samples': [measure(prepare, count) for _ in range(samples)]}


def main(prepare):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=0)
    args = parser.parse_args()
    print(json.dumps(execute(prepare, iterations=args.iterations)))
