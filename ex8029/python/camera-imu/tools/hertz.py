import time

class Hertz:
    __slots__ = ["rollover", "format", "start", "count"]

    def __init__(self, format, rollover):
        """
        format: string format to use when printing
        rollover: when to print string
        """
        self.rollover = rollover
        self.format = format
        self.reset()

    def increment(self):
        """
        Update count and if count == rollover, then print
        hertz using the format. Also at rollover, it resets
        """
        self.count += 1

        if (self.count % self.rollover) == 0:
            now = time.monotonic()
            hz = self.count/(now - self.start)
            self.count = 0
            self.start = now
            print(self.format.format(hz))

    def reset(self):
        """
        Resets count to 0 and start to current time
        """
        self.start = time.monotonic()
        self.count = 0
