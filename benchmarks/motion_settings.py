import sys
import tempfile
from pathlib import Path
from benchmarks.harness import main
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/robot_configuration'))
from robot_configuration.storage import Storage
class Cycle:
    def __init__(self):
        self.directory=tempfile.TemporaryDirectory()
        self.storage=Storage(self.directory.name)
        self.result=None
    def cycle(self,index):
        self.result=self.storage.system_settings()
        self.storage.set_system_settings(False)
    def verify(self):
        return self.result['ignore_missing_drivers'] is False
if __name__=='__main__': main(Cycle)
