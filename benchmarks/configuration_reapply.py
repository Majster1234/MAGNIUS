"""Ponowny zapis tego samego profilu; pliki tymczasowe, atrapa ROS, bez CAN."""
import sys
import tempfile
from copy import deepcopy
from pathlib import Path
from benchmarks.harness import main

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/robot_configuration'))
from robot_configuration.coordinator import Coordinator
from robot_configuration.schema import empty_document
from robot_configuration.storage import Storage


class Transport:
    def __init__(self):
        self.revision, self.stage, self.locked, self.operation_id = 'initial', 'boot', True, ''

    def control(self, phase, operation_id, state, changed, expected=''):
        if phase in ('bootstrap', 'prepare'):
            self.locked, self.operation_id, self.stage = True, operation_id, 'prepared'
        if phase in ('bootstrap', 'commit'):
            self.revision, self.stage = state['revision'], 'committed'
        if phase == 'finish':
            self.locked, self.stage = False, 'finished'
        return dict(revision=self.revision, stage=self.stage, locked=self.locked, operation_id=self.operation_id)

    def model(self, xml):
        self.xml = xml


class Cycle:
    def __init__(self):
        self.directory = tempfile.TemporaryDirectory()
        self.storage, self.transport = Storage(self.directory.name), Transport()
        self.coordinator = Coordinator(self.storage, self.transport)
        self.coordinator.bootstrap()
        document = empty_document()
        for axis in range(1, 7):
            for key, value in dict(gear_ratio=3, direction=1, zero_offset=0, lower=-180,
                                   upper=180, velocity=80, acceleration=100).items():
                document['values'][f'joint.{axis}.{key}'] = value
        self.coordinator.apply(*self.coordinator.begin(document, 'initial'))
        self.document = deepcopy(document)
        self.document['created_at'] = '2026-10-25T00:00:00.000Z'

    def cycle(self, index):
        c = self.coordinator
        c.apply(*c.begin(self.document, c.active['revision']))

    def verify(self):
        c = self.coordinator
        valid = (c.operation['phase'] == 'done' and c.operation['success'] is True
                 and not c.running and not self.transport.locked
                 and c.active['revision'] == self.transport.revision == self.storage.active()['revision']
                 and c.active['document']['values'] == self.document['values'])
        self.directory.cleanup()
        return valid


if __name__ == '__main__':
    main(Cycle)
