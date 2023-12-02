class PhaseFreqConv:
    def __init__(self, sampling_period):
        self._prev = None
        self._sp = sampling_period

    def feed(self, current):
        prev = self._prev
        self._prev = current
        if prev is None:
            return None
        else:
            return current - prev - self._sp

