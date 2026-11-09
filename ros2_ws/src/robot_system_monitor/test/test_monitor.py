import subprocess
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from robot_system_monitor import monitor, probes


def link(state='ERROR-ACTIVE', up=True):
    return [{'ifname': 'can0', 'flags': ['UP'] if up else [],
             'linkinfo': {'info_kind': 'can', 'info_data': {
                 'state': state, 'bittiming': {'bitrate': 500000}},
                 'info_xstats': {'bus_error': 2}},
             'stats64': {'rx': {'packets': 4, 'errors': 0},
                         'tx': {'packets': 0, 'errors': 0}}}]


@pytest.mark.parametrize('links,level', [([], 2), (link(up=False), 2),
                                       (link('BUS-OFF'), 2), (link('ERROR-PASSIVE'), 1),
                                       (link(), 0)])
def test_can_states(links, level):
    assert probes.parse_can_link(links)[0] == level


def test_active_can_is_not_drive_confirmation():
    level, message, values = probes.parse_can_link(link())
    assert level == 0
    assert 'niepotwierdzona' in message
    assert values['tx_packets'] == 0
    assert values['bus_errors'] == 2


def test_missing_device_differs_from_permission_error(monkeypatch):
    monkeypatch.setattr(probes.subprocess, 'run', lambda *a, **kw:
                        SimpleNamespace(returncode=1, stderr='Device "can0" does not exist.'))
    assert probes.can_status('can0')[2]['present'] is False
    monkeypatch.setattr(probes.subprocess, 'run', lambda *a, **kw:
                        SimpleNamespace(returncode=1, stderr='Operation not permitted'))
    with pytest.raises(RuntimeError, match='permitted'):
        probes.can_status('can0')


def test_unavailable_controllers():
    node = SimpleNamespace(pending=None, controller_result=None,
                           controllers=SimpleNamespace(service_is_ready=lambda: False))
    level, message, values = monitor.SystemMonitor.controller_status(node)
    assert level == 1
    assert 'nieuruchomione' in message
    assert values['configured'] is False


def test_controller_request_timeout(monkeypatch):
    removed = []
    pending = SimpleNamespace(done=lambda: False, cancel=lambda: removed.append('cancel'))
    node = SimpleNamespace(pending=pending, pending_since=0, controller_result=[],
                           controller_received=0, controllers=SimpleNamespace(
                               remove_pending_request=lambda f: removed.append(f),
                               service_is_ready=lambda: True,
                               call_async=lambda req: pending))
    monkeypatch.setattr(monitor.time, 'monotonic', lambda: 3.1)
    assert monitor.SystemMonitor.controller_status(node)[0] == 1
    assert removed == [pending, 'cancel']


def test_probe_failure_does_not_stop_other_measurements(monkeypatch):
    def failure():
        raise OSError('Brak dostępu')
    monkeypatch.setattr(monitor, 'host_status', failure)
    monkeypatch.setattr(monitor, 'can_status', lambda interface: (0, 'CAN', {'tx_packets': 0}))
    published = []
    node = SimpleNamespace(
        controller_status=lambda: (1, 'Brak kontrolerów', {}),
        get_parameter=lambda name: SimpleNamespace(value='can0'),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
        publisher=SimpleNamespace(publish=published.append))
    monitor.SystemMonitor.publish_status(node)
    states = {item.name: item.level[0] for item in published[0].status}
    assert states == {'host': 2, 'can': 0, 'controllers': 1, 'drives': 1}


def test_can_timeout_is_bounded(monkeypatch):
    def timeout(*args, **kwargs):
        assert kwargs['timeout'] == 0.75
        assert args[0] == ['ip', '-json', '-details', '-statistics', 'link', 'show', 'dev', 'can0']
        raise subprocess.TimeoutExpired(args[0], kwargs['timeout'])
    monkeypatch.setattr(probes.subprocess, 'run', timeout)
    with pytest.raises(subprocess.TimeoutExpired):
        probes.can_status('can0')
