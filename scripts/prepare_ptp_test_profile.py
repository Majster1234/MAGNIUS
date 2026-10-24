#!/usr/bin/env python3
"""Zatwierdza bazę ostatnich prób rozsprzęglonych wałów przez normalne API konfiguracji.

Nie wysyła komend ruchu, ENABLE ani zera. Zachowuje geometrię i kierunki;
po potwierdzonym zerowaniu ustawia offsety tych osi na zero. Prędkości planera
domyślnie ogranicza do 1 RPM wału. Jawny argument pozwala przywrócić zatwierdzony
limit do 80 stopni/s. Zachowuje holding 1 RPM i kod acc=1 napędów.
"""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import time
import urllib.request
import yaml

ROOT = Path(__file__).resolve().parents[1]
AXES = (1, 2, 3, 5)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--address', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--zero-proof', type=Path)
    parser.add_argument('--joint-velocity-deg-s', type=float,
                        help='Jawna zmiana limitu przegubów, do zatwierdzonych 80 stopni/s')
    args = parser.parse_args()
    if args.joint_velocity_deg_s is not None and not (
            math.isfinite(args.joint_velocity_deg_s) and 0 < args.joint_velocity_deg_s <= 80):
        parser.error('Limit przegubu musi należeć do (0, 80] stopni/s')
    args.output.mkdir(parents=True, exist_ok=True)
    base = f'http://{args.address}:8080/api/v1/'
    def call(path, body=None):
        request = urllib.request.Request(base+path, data=None if body is None else json.dumps(body).encode(),
                                         headers={'Content-Type': 'application/json'}, method='GET' if body is None else 'POST')
        with urllib.request.urlopen(request, timeout=6) as response:
            return json.load(response)
    zero_proof = json.loads(args.zero_proof.read_text()) if args.zero_proof else None
    if zero_proof:
        assert zero_proof.get('success') is True and zero_proof['bitrate']==1000000
        assert all(abs(zero_proof['axes'][str(a)]['after_ticks'])<=3 for a in AXES)
    snapshot = call('configuration')
    assert snapshot['operation']['phase'] in ('idle', 'done') and snapshot['operation']['success'] is True
    active = snapshot['active']; document = deepcopy(active['document'])
    (args.output/'configuration-before.json').write_text(json.dumps(snapshot, ensure_ascii=False, indent=2))
    samples = []
    for _ in range(5):
        state = call('manual/status')
        assert state['fresh'] and not state['busy'] and not state['fault'], 'Brak świeżego postoju'
        assert state['configuration_revision'] == active['revision'], 'Zmieniła się rewizja konfiguracji'
        for axis in state['axes']:
            if axis['available']:
                assert axis['enable_known'] and not axis['enabled'], 'Przed odblokowaniem wymagane ENABLE=0'
        samples.append(state); time.sleep(.1)
    for axis in AXES:
        actual = [s['axes'][axis-1] for s in samples]
        assert all(a['available'] and a['position_known'] for a in actual), f'Brak J{axis}'
        positions = [a['position_degrees'] for a in actual]
        assert max(positions)-min(positions) <= .15, f'J{axis}: brak stabilności pozycji'
        if zero_proof:
            assert abs(positions[-1])<=.2, f'J{axis}: pozycja oddaliła się od potwierdzonego zera'
            document['values'][f'joint.{axis}.zero_offset']=0.0
        elif not actual[-1]['calibration_valid']:
            assert args.joint_velocity_deg_s is None, f'J{axis}: zmiana prędkości wymaga ważnej kalibracji'
            baseline = json.loads((ROOT/'log/streaming100-deployment/final-runtime.json').read_text())['manual']
            previous = json.loads((ROOT/'log/streaming100-deployment/preflight.json').read_text())['configuration']['active']['document']['values']
            assert abs(positions[-1]-baseline['axes'][axis-1]['position_degrees']) <= 1, f'J{axis}: położenie inne od ostatniej zweryfikowanej bazy'
            for field in ('gear_ratio', 'direction', 'zero_offset'):
                key = f'joint.{axis}.{field}'
                assert document['values'][key] == previous[key], f'J{axis}: zmienił się {field}; wymagana osobna kalibracja'
    # Limit dotyczy również referencji planera, nie tylko pola RPM w ramce.
    max_rpm = 1
    for axis in range(1, 7):
        key = f'joint.{axis}.'; gear = document['values'][key+'gear_ratio']
        assert isinstance(gear, (int, float)) and math.isfinite(gear) and gear > 0
        document['values'][key+'velocity'] = (args.joint_velocity_deg_s if args.joint_velocity_deg_s is not None
                                             else min(document['values'][key+'velocity'], 6.0/gear))
        max_rpm = max(max_rpm, math.ceil(document['values'][key+'velocity']*gear/6.0))
    assert max_rpm <= 3000, 'Limit prędkości wymaga więcej niż 3000 RPM wału'
    call('configuration/validate', {'document': document})
    call('configuration/apply', {'document': document, 'expected_revision': active['revision']})
    deadline = time.monotonic()+12
    while time.monotonic() < deadline:
        result = call('configuration')
        if result['operation']['phase'] == 'done' and result['operation']['success'] is True:
            break
        if result['operation']['phase'] in ('failed', 'uncertain'):
            raise RuntimeError(result['operation']['message'])
        time.sleep(.2)
    else:
        raise RuntimeError('Nie potwierdzono transakcji konfiguracji')
    revision = result['active']['revision']
    assert result['active']['document']['values'] == document['values']
    deadline = time.monotonic()+5
    while time.monotonic() < deadline:
        state = call('manual/status')
        if state['fresh'] and state['configuration_revision'] == revision and all(state['axes'][a-1]['calibration_valid'] for a in AXES):
            break
        time.sleep(.1)
    else:
        raise RuntimeError('Runtime nie potwierdził przyjętej bazy stanowiska')
    profile = ROOT/'config/local/execution.production.yaml'
    old_profile = profile.read_text(); (args.output/'execution-before.yaml').write_text(old_profile)
    settings = yaml.safe_load(old_profile); values = settings['/**']['ros__parameters']
    values.update(execution_enabled=True, execution_configuration_revision=revision,
                  execution_min_holding_rpm=1, execution_max_rpm=max_rpm, execution_acceleration_codes=[1]*6)
    profile.write_text('# PTP stanowiska rozsprzęglonego: limity przegubów + acc1, bez automatycznego ruchu.\n'+yaml.safe_dump(settings, sort_keys=False, allow_unicode=True))
    (args.output/'configuration-after.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    (args.output/'manual-after.json').write_text(json.dumps(state, ensure_ascii=False, indent=2))
    print(json.dumps({'revision': revision, 'execution_enabled_after_restart': True,
                      'joint_velocity_limit_deg_s': args.joint_velocity_deg_s,
                      'shaft_limit_rpm': max_rpm, 'acc_code': 1,
                      'basis': 'potwierdzone zero enkoderów' if zero_proof else 'bieżąca baza konfiguracji'}, ensure_ascii=False))

if __name__ == '__main__':
    main()
