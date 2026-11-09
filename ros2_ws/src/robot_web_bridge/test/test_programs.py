"""Programy ruchowe: magazyn plików, walidacja, konflikty rewizji i API."""
import asyncio
import json
import pytest
from robot_web_bridge.app import create_app
from robot_web_bridge.programs import ProgramError, ProgramConflict
from robot_web_bridge.status import StatusStore
from test_configuration_api import request

# Testy magazynu używają własnej biblioteki (z pętlą i licznikiem); domyślna zawiera tylko move_ptp/move_lin.
TEST_COMMANDS = {'schema_version': 1, 'categories': [{'id': 'ruch', 'label': 'Ruch'}], 'commands': [
    {'id': 'move_ptp', 'category': 'ruch', 'label': 'jedź PTP do {position} z prędkością {speed}', 'params': [{'name': 'position', 'kind': 'position'}, {'name': 'speed', 'kind': 'number'}]},
    {'id': 'move_lin', 'category': 'ruch', 'label': 'jedź LIN do {position} z prędkością {speed}', 'params': [{'name': 'position', 'kind': 'position'}, {'name': 'speed', 'kind': 'number'}]},
    {'id': 'for', 'category': 'logika', 'label': 'powtórz {counter} od {from} do {to}', 'slots': [{'name': 'body'}],
     'params': [{'name': 'counter', 'kind': 'variable'}, {'name': 'from', 'kind': 'number'}, {'name': 'to', 'kind': 'number'}]},
    {'id': 'increment', 'category': 'zmienne', 'label': 'zwiększ {variable} o {by}', 'params': [{'name': 'variable', 'kind': 'variable'}, {'name': 'by', 'kind': 'number'}]},
    {'id': 'wait', 'category': 'czas', 'label': 'czekaj {seconds} s', 'params': [{'name': 'seconds', 'kind': 'number'}]}]}


def ProgramStore(directory):   # noqa: N802 — magazyn z biblioteką testową
    from robot_web_bridge.programs import ProgramStore as Store
    import pathlib as _p
    library = _p.Path(directory) / 'library'
    library.mkdir(parents=True, exist_ok=True)
    (library / 'commands.json').write_text(json.dumps(TEST_COMMANDS))
    return Store(directory)


def program(**extra):
    return {'name': 'Paletyzacja 3×', 'variables': [{'name': 'licznik', 'type': 'INT', 'init': 0, 'note': 'detale'},
            {'name': 'v_lin', 'type': 'REAL', 'init': 50, 'unit': 'mm/s'}, {'name': 'i', 'type': 'INT', 'init': 0}],
            'positions': [{'name': 'Home', 'joints_deg': [0, 0, 0, 0, 45, 0]}, {'name': 'Odbiór', 'joints_deg': [30, 25, 15, 0, 40, 30]}],
            'blocks': [{'id': 'b1', 'command': 'move_ptp', 'params': {'position': {'position': 'Home'}, 'speed': 30}},
                       {'id': 'b2', 'command': 'for', 'params': {'counter': {'var': 'i'}, 'from': 1, 'to': 3}, 'body': [
                           {'id': 'b3', 'command': 'move_lin', 'params': {'position': {'position': 'Odbiór'}, 'speed': {'var': 'v_lin'}}},
                           {'id': 'b4', 'command': 'increment', 'params': {'variable': {'var': 'licznik'}, 'by': 1}}]}], **extra}


def test_store_creates_library_folders_and_default_motion_commands(tmp_path):
    from robot_web_bridge.programs import ProgramStore as Store
    store = Store(tmp_path)
    assert (tmp_path / 'library/commands.json').exists() and (tmp_path / 'library/variables.json').exists()
    assert (tmp_path / 'library/positions.json').exists() and (tmp_path / 'programs').is_dir()
    snapshot = store.library_snapshot()
    assert [c['id'] for c in snapshot['commands']['commands']] == ['move_ptp', 'move_lin'] and snapshot['warning'] is None
    assert store.create({'name': 'Pusty'})['program']['blocks'] == []
    with pytest.raises(ProgramError):
        store.create({'name': 'Z blokiem', 'blocks': [{'id': 'a', 'command': 'for'}]})
    custom = ProgramStore(tmp_path / 'custom')
    assert {c['id'] for c in custom.library_snapshot()['commands']['commands']} >= {'move_ptp', 'for'}


def test_create_save_conflict_and_delete_to_trash(tmp_path):
    store = ProgramStore(tmp_path)
    created = store.create(program())['program']
    assert created['id'] == 'paletyzacja_3' and created['revision'] and created['created_at']
    assert store.create(program())['program']['id'] == 'paletyzacja_3_2'
    changed = dict(created, name='Paletyzacja 4×')
    saved = store.save(created['id'], changed, created['revision'])['program']
    assert saved['name'] == 'Paletyzacja 4×' and saved['revision'] != created['revision']
    with pytest.raises(ProgramConflict):
        store.save(created['id'], changed, created['revision'])
    assert store.save(created['id'], saved, saved['revision']).get('unchanged')
    assert [p['blocks'] for p in store.list() if p['id'] == created['id']] == [4]
    store.delete(created['id'])
    assert not (tmp_path / 'programs/paletyzacja_3.json').exists() and list((tmp_path / 'programs/.trash').glob('paletyzacja_3-*.json'))


def test_validation_rejects_bad_structure_and_reports_missing_references(tmp_path):
    store = ProgramStore(tmp_path)
    bad = [dict(program(), variables=[{'name': '1x', 'type': 'INT', 'init': 0}]),
           dict(program(), variables=[{'name': 'a', 'type': 'INT', 'init': 1.5}]),
           dict(program(), positions=[{'name': 'P', 'joints_deg': [0, 0, 0]}]),
           dict(program(), blocks=[{'id': 'x', 'command': 'nieznany'}]),
           dict(program(), blocks=[{'id': 'x', 'command': 'wait', 'params': {'obce': 1}}]),
           dict(program(), blocks=[{'id': 'x', 'command': 'wait'}, {'id': 'x', 'command': 'wait'}]),
           dict(program(), variables=[{'name': 'a', 'type': 'INT', 'init': 0}, {'name': 'a', 'type': 'REAL', 'init': 0}])]
    for document in bad:
        with pytest.raises(ProgramError):
            store.create(document)
    created = store.create(dict(program(), positions=[]))
    assert created['problems'] == ['Brak pozycji „Home”', 'Brak pozycji „Odbiór”']


def test_library_lists_are_validated_and_revisioned(tmp_path):
    store = ProgramStore(tmp_path)
    first = store.library_snapshot()['positions']
    saved = store.set_library('positions', [{'name': 'Home', 'joints_deg': [0, 0, 0, 0, 45, 0]}], first['revision'])
    with pytest.raises(ProgramConflict):
        store.set_library('positions', [], first['revision'])
    with pytest.raises(ProgramError):
        store.set_library('variables', [{'name': 'x', 'type': 'XYZ', 'init': 0}])
    assert store.library_snapshot()['positions']['revision'] == saved['revision']


def test_api_round_trip(tmp_path):
    store = ProgramStore(tmp_path / 'programs')
    app = create_app(StatusStore('192.168.1.69', 8080, 'enp2s0', '192.168.1.0/24'), tmp_path, program_store=store)
    run = lambda *a: asyncio.run(request(app, *a))
    code, data = run('POST', '/api/v1/programs', {'program': program()}); assert code == 201
    identifier, rev = data['program']['id'], data['program']['revision']
    code, data = run('GET', '/api/v1/programs'); assert code == 200 and data['items'][0]['id'] == identifier
    code, data = run('PUT', f'/api/v1/programs/{identifier}', {'program': dict(program(), name='Nowa'), 'expected_revision': rev}); assert code == 200
    code, _ = run('PUT', f'/api/v1/programs/{identifier}', {'program': program(), 'expected_revision': rev}); assert code == 409
    code, _ = run('PUT', f'/api/v1/programs/{identifier}', {'program': dict(program(), blocks=[{'id': 'q', 'command': '??'}]), 'expected_revision': data['program']['revision']}); assert code == 422
    code, _ = run('GET', '/api/v1/programs/../etc'); assert code == 404
    code, data = run('GET', '/api/v1/program-library'); assert code == 200 and data['commands']['commands']
    code, _ = run('PUT', '/api/v1/program-library/variables', {'items': [{'name': 'g', 'type': 'BOOL', 'init': True}]}); assert code == 200
    code, _ = run('DELETE', f'/api/v1/programs/{identifier}'); assert code == 200
    code, _ = run('GET', f'/api/v1/programs/{identifier}'); assert code == 404


def test_operator_approval_binds_revision_and_expires_on_change(tmp_path):
    store = ProgramStore(tmp_path)
    created = store.create(program())['program']
    assert not store.approved(created)
    with pytest.raises(ProgramConflict):
        store.approve(created['id'], 'zla-rewizja')
    approved = store.approve(created['id'], created['revision'], '192.168.1.85')['program']
    assert store.approved(approved) and approved['revision'] == created['revision'] and approved['approval']['client'] == '192.168.1.85'
    sneaky = dict(approved, approval={'revision': 'x'}, name='Zmieniony')   # klient nie może ustawić zatwierdzenia zapisem
    saved = store.save(created['id'], sneaky, approved['revision'])['program']
    assert not store.approved(saved) and saved['approval']['revision'] == created['revision']
    assert [p['approved'] for p in store.list()] == [False]
    broken = store.create(dict(program(), name='Bez pozycji', positions=[]))['program']
    with pytest.raises(ProgramError):
        store.approve(broken['id'], broken['revision'])
    assert store.check(program())['problems'] == []
