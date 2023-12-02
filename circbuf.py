import numpy as np

class CircularBuffer:
    def __init__(self, rows, cols, defval=np.nan):
        self._buf = np.full([rows, cols], defval, dtype=float)

    def append(self, *values):
        self._buf[0] = np.array(values)
        self._buf = np.roll(self._buf, -1, axis=0)

    def get(self):
        return self._buf

