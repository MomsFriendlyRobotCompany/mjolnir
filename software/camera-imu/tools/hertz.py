import time

class Hertz:
    def __init__(self, format, rollover):
        self.rollover = rollover
        self.format = format
        self.reset()

    def increment(self):
        self.count += 1

        if (self.count % self.rollover) == 0:
            now = time.monotonic()
            hz = self.count/(now - self.start_hz)
            self.count = 0
            self.start_hz = now
            print(self.format.format(hz))

    def reset(self):
        self.start_hz = time.monotonic()
        self.count = 0
