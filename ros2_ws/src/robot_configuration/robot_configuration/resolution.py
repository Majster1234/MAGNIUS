"""Profil rozstrzygnięty przez właściciela CAN, bez przeliczania z opóźnionego ROS."""
from copy import deepcopy
import math
from .schema import CRITICAL_FAULT_POLICY, profiles, validate


def confirmation_axes(value):
    if not isinstance(value, list) or any(type(a) is not int or not 1 <= a <= 6 for a in value) or len(set(value)) != len(value):
        raise ValueError('confirm_calibration_axes: wymagane unikalne numery osi 1–6')
    return [a in value for a in range(1, 7)]


def resolve(previous, candidate, response):
    resolved = response.get('resolved_configuration')
    if not isinstance(resolved, dict) or resolved.get('revision') != candidate['revision']:
        raise RuntimeError('Brak zgodnego profilu rozstrzygniętego przez CAN')
    policy = resolved.get('disable_enable_on_critical_fault', True)
    if type(policy) is not bool or policy != candidate['document']['values'].get(CRITICAL_FAULT_POLICY, True):
        raise RuntimeError('CAN nie potwierdził reguły wyłączenia ENABLE przy awarii krytycznej')
    axes, invalid = resolved.get('axes'), resolved.get('calibration_invalid')
    if not isinstance(axes, list) or len(axes) != 6 or not isinstance(invalid, list) or len(invalid) != 6 or any(type(v) is not bool for v in invalid):
        raise RuntimeError('Niekompletny profil lub stan kalibracji CAN')
    result = deepcopy(candidate)
    for i, (old, requested, actual) in enumerate(zip(profiles(previous['document']), profiles(candidate['document']), axes)):
        for key, value in requested.items():
            got = actual.get(key)
            if key == 'zero_offset_rad' and requested['configured']:
                if type(got) not in (int, float) or not math.isfinite(got):
                    raise RuntimeError('Nieprawidłowy offset rozstrzygnięty przez CAN')
                if not math.isclose(got, value, rel_tol=0, abs_tol=1e-12):
                    if not old['configured'] or old['gear_ratio'] == requested['gear_ratio'] or old['zero_offset_rad'] != value or old['direction'] != requested['direction']:
                        raise RuntimeError('CAN zmienił offset poza zmianą przełożenia')
                    result['document']['values'][f'joint.{i+1}.zero_offset'] = math.degrees(got)
            elif got != value:
                raise RuntimeError(f'CAN zmienił nieoczekiwany parametr J{i+1}: {key}')
    result['document'] = validate(result['document'])
    return result, invalid
