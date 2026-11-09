#!/usr/bin/env python3
"""Analiza pasywnego zapisu `candump -L -e can0` z prób JOG TCP.

Liczy dla osi J1–J6 (CAN ID 1–6): odstępy kolejnych F5 w seriach ruchu, skoki RPM F5,
opóźnienie i brak odpowiedzi pozycji 0x31, F5 wysłane przy oczekującym odczycie tej osi
oraz ramki błędów magistrali. Wynik: JSON na stdout.
"""
import json
import statistics
import sys


def parse(path):
    for line in open(path, encoding="ascii", errors="replace"):
        parts = line.split()
        if len(parts) < 3 or "#" not in parts[2]:
            continue
        stamp = float(parts[0].strip("()"))
        ident, data = parts[2].split("#", 1)
        yield stamp, int(ident, 16), bytes.fromhex(data.split("#")[-1] if data.startswith("R") else data)


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    return values[min(len(values) - 1, int(round(q * (len(values) - 1))))]


def summary(values, digits=3):
    if not values:
        return None
    return {"n": len(values), "mean": round(statistics.fmean(values), digits),
            "stdev": round(statistics.pstdev(values), digits), "p99": round(percentile(values, .99), digits),
            "max": round(max(values), digits), "min": round(min(values), digits)}


def main(path):
    errors = 0
    axes = {a: {"f5": [], "pending": None, "latency": [], "missing": 0, "f5_while_pending": 0} for a in range(1, 7)}
    for stamp, ident, data in parse(path):
        if ident & 0x20000000:
            errors += 1
            continue
        axis = axes.get(ident)
        if axis is None or not data:
            continue
        if data[0] == 0xF5 and len(data) == 8:
            if axis["pending"] is not None:
                axis["f5_while_pending"] += 1
            axis["f5"].append((stamp, ((data[1] & 0x0F) << 8) | data[2]))
        elif data[0] == 0x31 and len(data) == 2:
            if axis["pending"] is not None:
                axis["missing"] += 1
            axis["pending"] = stamp
        elif data[0] == 0x31 and len(data) == 8 and axis["pending"] is not None:
            axis["latency"].append((stamp - axis["pending"]) * 1000)
            axis["pending"] = None
    result = {"error_frames": errors, "axes": {}}
    for number, axis in axes.items():
        intervals, jumps, segments = [], [], 0
        previous = None
        for stamp, rpm in axis["f5"]:
            if previous is not None and stamp - previous[0] < .1:
                intervals.append((stamp - previous[0]) * 1000)
                if rpm > 1 and previous[1] > 1:
                    jumps.append(max(rpm, previous[1]) / min(rpm, previous[1]))
            else:
                segments += 1
            previous = (stamp, rpm)
        result["axes"][f"J{number}"] = {
            "f5_frames": len(axis["f5"]), "motion_segments": segments,
            "f5_interval_ms": summary(intervals),
            "f5_interval_outside_8_12_ms": sum(1 for v in intervals if v < 8 or v > 12),
            "rpm_ratio_jumps_over_1_5": sum(1 for v in jumps if v > 1.5),
            "rpm_ratio_jump_max": round(max(jumps), 3) if jumps else None,
            "position_reply_ms": summary(axis["latency"]),
            "position_reply_missing": axis["missing"],
            "f5_while_position_pending": axis["f5_while_pending"],
        }
    json.dump(result, sys.stdout, indent=2, ensure_ascii=False)
    print()


if __name__ == "__main__":
    main(sys.argv[1])
