import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from argparse import ArgumentParser
import numpy as np
from mqttclient import MQTTClient
from filters import Bessel

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

class CircularBuffer:
    def __init__(self, rows, cols, defval=np.nan):
        self._buf = np.full([rows, cols], defval, dtype=float)

    def append(self, *values):
        self._buf[0] = np.array(values)
        self._buf = np.roll(self._buf, -1, axis=0)

    def get(self):
        return self._buf

def main():
    phase_diff = PhaseFreqConv(args.period)
    filt1 = Bessel(3, 0.02)
    buf = CircularBuffer(args.bufsize, 2)
    mqtt = MQTTClient(
        host=args.host,
        topics=['meas/chA']
        )

    # enable interactive mode
    plt.ion()

    while True:
        for phase in (float(strvalue) for topic, strvalue in mqtt.get() if topic == 'meas/chA'):
            freq = phase_diff.feed(phase)

            if freq is not None:
                # frequency lowpass filtering
                out1 = np.array(filt1.feed(freq))

                # append sample to circular buffer
                buf.append(freq, out1[0])

                # plot results
                plt.plot(buf.get(), label=['in', 'bessel 3rd'])
                plt.grid()
                plt.legend()
                plt.draw()
                plt.pause(0.1)
                plt.clf()

            phase_prev = phase

if __name__ == '__main__':
    p = ArgumentParser()
    p.add_argument('--bufsize', type=int, default=500, help='display buffer length')
    p.add_argument('--period', type=int, default=1, help='sample period')
    p.add_argument('host')
    args = p.parse_args()
    main()
