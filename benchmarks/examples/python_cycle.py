"""Przykład techniczny: stały koszt obliczeń, bez ROS i sprzętu."""

from benchmarks.harness import main


class Cycle:
    def __init__(self):
        self.value = 0
        self.count = 0

    def cycle(self, index):
        self.value = (self.value + index + 1) & 0xffffffff
        self.count += 1

    def verify(self):
        return self.value == (self.count * (self.count + 1) // 2) & 0xffffffff


if __name__ == '__main__':
    main(Cycle)
