"""Jednostki pliku/panelu: mm, stopnie, kg. Dopiero interfejs ROS używa SI."""
from copy import deepcopy
from datetime import datetime, timezone
import math
import re
import xml.etree.ElementTree as ET

VERSION = 1
UNITS = 'mm-deg-kg'
MAX_BYTES = 65536
CONTROL = ('gear_ratio', 'direction', 'zero_offset', 'lower', 'upper', 'velocity', 'acceleration')
ORIGIN = ('origin_x', 'origin_y', 'origin_z', 'origin_roll', 'origin_pitch', 'origin_yaw')
TCP = ('tcp_x', 'tcp_y', 'tcp_z', 'tcp_roll', 'tcp_pitch', 'tcp_yaw')
TCP_MOTION = ('max_linear_velocity', 'max_angular_velocity', 'linear_acceleration', 'angular_acceleration')
TEXT = ('robot.name', 'robot.base_frame', 'robot.flange_frame', 'tool.name', 'tool.frame')
CRITICAL_FAULT_POLICY = 'safety.disable_enable_on_critical_fault'
DISPATCH_MODE = 'motion.ptp_lin_dispatch_mode'
KEYS = (*TEXT, 'tool.mass', 'tool.com_x', 'tool.com_y', 'tool.com_z',
        *(f'tool.{key}' for key in TCP),
        *(f'joint.{axis}.{key}' for axis in range(1, 7) for key in (*ORIGIN, *CONTROL)),
        *(f'tcp.{key}' for key in TCP_MOTION), CRITICAL_FAULT_POLICY, DISPATCH_MODE)
DEMO = ((0, 0, 100), (0, 0, 100), (0, 0, 250), (220, 0, 0), (80, 0, 0), (60, 0, 0))
AXES = ((0, 0, 1), (0, 1, 0), (0, 1, 0), (1, 0, 0), (0, 1, 0), (1, 0, 0))


def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def empty_document():
    return {'schema_version': VERSION, 'units': UNITS, 'name': '', 'created_at': now_iso(),
            'values': {**dict.fromkeys(KEYS), CRITICAL_FAULT_POLICY: True, DISPATCH_MODE: 'streamed'}}


def validate(document):
    if not isinstance(document, dict) or set(document) - {'schema_version', 'units', 'name', 'created_at', 'values'}:
        raise ValueError('Nieprawidłowa struktura snapshotu')
    if type(document.get('schema_version')) is not int or document['schema_version'] != VERSION or document.get('units') != UNITS:
        raise ValueError('Nieobsługiwana wersja lub jednostki snapshotu; wymagane mm-deg-kg, wersja 1')
    values = document.get('values')
    required = set(KEYS) - {f'tcp.{key}' for key in TCP_MOTION} - {CRITICAL_FAULT_POLICY, DISPATCH_MODE}
    if not isinstance(values, dict) or set(values) - set(KEYS) or required - set(values):
        raise ValueError('Snapshot musi zawierać komplet znanych pól; nieznane dane zapisuj jako null')
    result = deepcopy(document)
    result['values'].setdefault(CRITICAL_FAULT_POLICY, True)
    result['values'].setdefault(DISPATCH_MODE, 'streamed')
    for key in TCP_MOTION:
        result['values'].setdefault(f'tcp.{key}', None)
    values = result['values']
    for key, value in values.items():
        if key == DISPATCH_MODE:
            if value not in ('streamed', 'synchronized'):
                raise ValueError(f'{key}: wymagane streamed albo synchronized')
            continue
        if key == CRITICAL_FAULT_POLICY:
            if type(value) is not bool:
                raise ValueError(f'{key}: wymagane true albo false')
            continue
        if value is None:
            continue
        if key in TEXT:
            if not isinstance(value, str) or len(value) > 256:
                raise ValueError(f'{key}: wymagany tekst do 256 znaków')
            result['values'][key] = value.strip() or None
        else:
            if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 1e9:
                raise ValueError(f'{key}: wymagana skończona liczba w zakresie ±1e9')
            field = key.rsplit('.', 1)[-1]
            if field in ('gear_ratio', 'velocity', 'acceleration') and value <= 0:
                raise ValueError(f'{key}: wartość musi być dodatnia')
            if key.startswith('tcp.') and value <= 0:
                raise ValueError(f'{key}: wartość musi być dodatnia')
            if field == 'direction' and value not in (-1, 1):
                raise ValueError(f'{key}: kierunek musi wynosić +1 albo -1')
            if key == 'tool.mass' and value < 0:
                raise ValueError('Masa nie może być ujemna')
    for axis in range(1, 7):
        lower, upper = (values[f'joint.{axis}.{key}'] for key in ('lower', 'upper'))
        if lower is not None and upper is not None and lower >= upper:
            raise ValueError(f'J{axis}: dolny limit musi być mniejszy od górnego')
    for velocity, acceleration in (('max_linear_velocity', 'linear_acceleration'), ('max_angular_velocity', 'angular_acceleration')):
        v, a = values[f'tcp.{velocity}'], values[f'tcp.{acceleration}']
        if v is not None and a is not None and v / a > 10:
            raise ValueError('Profil TCP wymaga hamowania dłuższego niż 10 s')
    name = document.get('name', '')
    if not isinstance(name, str) or len(name) > 80:
        raise ValueError('Nazwa snapshotu: do 80 znaków')
    stamp = document.get('created_at', now_iso())
    try:
        if not isinstance(stamp, str) or len(stamp) > 40 or datetime.fromisoformat(stamp.replace('Z', '+00:00')).tzinfo is None:
            raise ValueError()
    except (ValueError, TypeError):
        raise ValueError('Nieprawidłowa data snapshotu') from None
    result.update(name=name.strip(), created_at=stamp)
    names = [values.get('robot.base_frame') or 'base_link',
             *[f'link_{i}' for i in range(1, 6)],
             values.get('robot.flange_frame') or 'link_6', values.get('tool.frame') or 'tool0']
    if len(set(names)) != 8 or any(not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name) for name in names):
        raise ValueError('Nazwy układów muszą być unikalne: litery, cyfry i podkreślenia, bez początkowej cyfry')
    return result


def profiles(document):
    values = document['values']
    result = []
    for axis in range(1, 7):
        data = {key: values[f'joint.{axis}.{key}'] for key in CONTROL}
        complete = all(value is not None for value in data.values())
        result.append({'axis': axis, 'configured': complete, 'gear_ratio': float(data['gear_ratio'] or 0),
                       'direction': int(data['direction'] or 0),
                       **{target: math.radians(data[source] or 0) for source, target in (
                           ('zero_offset', 'zero_offset_rad'), ('lower', 'lower_rad'), ('upper', 'upper_rad'),
                           ('velocity', 'velocity_rad_s'), ('acceleration', 'acceleration_rad_s2'))}})
    return result


def changed_axes(previous, candidate):
    a, b = previous['values'], candidate['values']
    common = any(a.get(key) != b.get(key) for key in KEYS if key not in (CRITICAL_FAULT_POLICY, DISPATCH_MODE) and not key.startswith(('joint.', 'tcp.')))
    return [common or any(a[f'joint.{axis}.{key}'] != b[f'joint.{axis}.{key}'] for key in (*ORIGIN, *CONTROL))
            for axis in range(1, 7)]


def validate_transition(previous, candidate):
    for old, new in zip(profiles(previous), profiles(candidate)):
        if old['configured'] and not new['configured']:
            raise ValueError(f"J{old['axis']}: nie można usunąć wymaganego parametru wcześniej skonfigurowanej osi")


def build_urdf(document, revision):
    values = document['values']
    complete = all(values[f'joint.{axis}.{key}'] is not None for axis in range(1, 7) for key in ORIGIN)
    root = ET.Element('robot', name=values['robot.name'] or 'configured_preview',
                      configuration_revision=revision, model_kind='configured' if complete else 'partial-demo',
                      tcp_configured='true' if complete and all(values[f'tool.{key}'] is not None for key in TCP) else 'false')
    names = [values['robot.base_frame'] or 'base_link', *[f'link_{i}' for i in range(1, 6)],
             values['robot.flange_frame'] or 'link_6', values['tool.frame'] or 'tool0']
    translations, rotations = [], []
    for axis in range(1, 7):
        prefix = f'joint.{axis}.'
        translations.append([(values[prefix + ORIGIN[i]] if values[prefix + ORIGIN[i]] is not None else DEMO[axis - 1][i]) / 1000 for i in range(3)])
        rotations.append([math.radians(values[prefix + key] or 0) for key in ORIGIN[3:]])
    tcp = [(values[f'tool.{TCP[i]}'] if values[f'tool.{TCP[i]}'] is not None else (80 if i == 0 else 0)) / 1000 for i in range(3)]

    def vec(items):
        return ' '.join(f'{item:.15g}' for item in items)

    def visual(link, geometry, xyz=(0, 0, 0), rpy=(0, 0, 0)):
        element = ET.SubElement(link, 'visual')
        ET.SubElement(element, 'origin', xyz=vec(xyz), rpy=vec(rpy))
        shape, dimensions = geometry
        ET.SubElement(ET.SubElement(element, 'geometry'), shape, **dimensions)
        ET.SubElement(ET.SubElement(element, 'material', name='body'), 'color', rgba='0.43 0.51 0.57 1')

    base = ET.SubElement(root, 'link', name=names[0])
    visual(base, ('cylinder', {'radius': '0.09', 'length': '0.1'}), xyz=(0, 0, .05))
    for i in range(6):
        link = ET.SubElement(root, 'link', name=names[i + 1])
        orientation = (0, 0, 0) if AXES[i][2] else ((math.pi / 2, 0, 0) if AXES[i][1] else (0, math.pi / 2, 0))
        visual(link, ('cylinder', {'radius': '0.035', 'length': '0.07'}), rpy=orientation)
        end = translations[i + 1] if i < 5 else tcp
        length = math.sqrt(sum(x * x for x in end))
        if length > 1e-9:
            visual(link, ('box', {'size': vec((.045, .045, length))}), xyz=[x / 2 for x in end],
                   rpy=(0, math.acos(max(-1, min(1, end[2] / length))), math.atan2(end[1], end[0])))
        joint = ET.SubElement(root, 'joint', name=f'joint_{i + 1}', type='continuous')
        ET.SubElement(joint, 'parent', link=names[i]); ET.SubElement(joint, 'child', link=names[i + 1])
        ET.SubElement(joint, 'origin', xyz=vec(translations[i]), rpy=vec(rotations[i]))
        ET.SubElement(joint, 'axis', xyz=vec(AXES[i]))
    ET.SubElement(root, 'link', name=names[7])
    joint = ET.SubElement(root, 'joint', name='tool_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link=names[6]); ET.SubElement(joint, 'child', link=names[7])
    ET.SubElement(joint, 'origin', xyz=vec(tcp), rpy=vec([math.radians(values[f'tool.{key}'] or 0) for key in TCP[3:]]))
    return ET.tostring(root, encoding='unicode')
