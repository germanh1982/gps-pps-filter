import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from argparse import ArgumentParser
import numpy as np
from mqttclient import MQTTClient
from circbuf import CircularBuffer
from pfconv import PhaseFreqConv

def main():
    phase_diff = PhaseFreqConv(args.period)
    phase_diff_corr = PhaseFreqConv(args.period)
    buf = CircularBuffer(args.bufsize, 3)
    mqtt = MQTTClient(host=args.host, topics=['meas/chA', 'qerr'])

    # enable interactive mode
    plt.ion()

    qerr = 0

    while True:
        for topic, value in mqtt.get():
            if topic == 'qerr':
                qerr = float(value) * 1e-12 # convert from ps to s
                print(f"qerr = {qerr}")

            elif topic == 'meas/chA':
                phase = float(value)
                freq = phase_diff.feed(phase)
                freq_corr = phase_diff_corr.feed(phase + qerr)

                print(f"freq={freq} freq_corr={freq_corr}")

                if freq is not None:
                    # frequency lowpass filtering
                    #out1 = np.array(filt1.feed(freq))

                    # append sample to circular buffer
                    buf.append(freq, qerr, freq_corr)

                    # plot results
                    plt.plot(buf.get(), label=['freq', 'qerr', 'corrected'])
                    plt.grid()
                    plt.legend()
                    plt.draw()
                    plt.pause(1)
                    plt.clf()

if __name__ == '__main__':
    p = ArgumentParser()
    p.add_argument('--bufsize', type=int, default=500, help='display buffer length')
    p.add_argument('--period', type=int, default=1, help='sample period')
    p.add_argument('host')
    args = p.parse_args()
    main()
