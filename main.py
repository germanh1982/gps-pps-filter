import queue
from paho.mqtt.client import Client
import logging
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from argparse import ArgumentParser
import numpy as np
from circbuf import CircularBuffer
from pfconv import PhaseFreqConv
from kalman import Kalman

def initialize_mqtt(qm):
    # set up mqtt consumer
    def on_message(client, userdata, msg):
        qm.put((msg.topic, msg.payload))

    client = Client()
    client.connect(args.host)
    client.on_message = on_message
    client.loop_start()
    client.subscribe([
        (f'meas/{args.channel}', 0),
        ('qerr', 0)
    ])

def main():
    # set up data processing blocks
    phase_diff = PhaseFreqConv(args.period)
    phase_diff_corr = PhaseFreqConv(args.period)
    buf = CircularBuffer(args.bufsize, 2)
    filt = Kalman(args.startx, args.startv, meas_noise=args.measnoise, proc_noise_var=args.procnoise)

    qerr = [0] # PPS quantization error: keep value in mutable type to avoid nonlocal declaration in drawing callback()

    # configure plot
    fig = plt.figure()
    ax1 = fig.add_subplot(111)

    # queue to synchronize data from mqtt into drawing callback
    qm = queue.Queue()

    initialize_mqtt(qm)

    # line labels
    labels = [
        'freq_corr',
        'filt_freq',
    ]

    def animate(frame):
        # drawing callback

        while True:
            try:
                topic, value = qm.get_nowait()

            except queue.Empty:
                break

            if topic == 'qerr':
                # value is phase quantization error
                qerr[0] = float(value) * 1e-12 # convert from ps to s

            elif topic == f'meas/{args.channel}':
                # value is PPS phase
                phase = float(value)
                freq = phase_diff.feed(phase)
                freq_corr = phase_diff_corr.feed(phase + qerr[0])

                if freq_corr is not None:
                    filt_freq, filt_drift = filt.feed(freq_corr)
                    log.debug(f"freq={freq_corr} qerr={qerr[0]} filt_freq={filt_freq}")

                if freq is not None and filt_freq is not None:
                    # append sample to circular buffer
                    buf.append(freq_corr, filt_freq)

                    # plot results
                    ax1.clear()
                    ax1.plot(buf.get(), label=labels)
                    ax1.grid()
                    ax1.legend()

    ani = FuncAnimation(fig, animate, interval=args.period * 1000, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        print(f"Last state: --startx={filt.state[0]} --startv={filt.state[1]}")

if __name__ == '__main__':
    p = ArgumentParser()
    p.add_argument('--bufsize', type=int, default=500, help='display buffer length')
    p.add_argument('--period', type=int, default=1, help='sample period')
    p.add_argument('--measnoise', type=float, default=1e4, help='Measurement noise')
    p.add_argument('--procnoise', type=float, default=0, help='Process noise')
    p.add_argument('--channel', choices=['chA', 'chB', 'chC', 'chD'], default='chA', help='TICC sampling channel')
    p.add_argument('--startx', type=float, default=0, help="Initial filter state for x")
    p.add_argument('--startv', type=float, default=0, help="Initial filter state for v")
    p.add_argument('host')
    args = p.parse_args()

    logging.basicConfig()
    log = logging.getLogger('gpsfilter')
    log.setLevel(logging.DEBUG)

    main()
