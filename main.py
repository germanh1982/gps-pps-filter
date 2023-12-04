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
import multiprocessing
from serial import Serial
from pyubx2 import UBXReader
from time import sleep

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
    ])

def gpsreader(qerr_val):
    # serial port setup
    stream = Serial(args.gpsport, args.gpsspeed, timeout=2)
    ubr = UBXReader(stream)

    # main loop
    while True:
        raw_data, parsed_data = ubr.read()
        if parsed_data.identity == 'TIM-TP':
            log.debug(parsed_data)
            sleep(1)
            qerr_val.value = parsed_data.qErr


def main():
    # set up gps receiver
    qerr_val = multiprocessing.Value('d')
    gpsr = multiprocessing.Process(target=gpsreader, args=(qerr_val,))
    gpsr.start()

    # set up data processing blocks
    phase_diff = PhaseFreqConv(args.period)
    phase_diff_corr = PhaseFreqConv(args.period)
    buf = CircularBuffer(args.bufsize, 4)
    filt = Kalman(args.startx, args.startv, meas_noise=args.measnoise, proc_noise_var=args.procnoise)

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
        'qerr',
        'freq',
    ]

    def animate(frame):
        # drawing callback

        while True:
            try:
                topic, value = qm.get_nowait()
                qerr = qerr_val.value * 1e-12

            except queue.Empty:
                break

            if topic == f'meas/{args.channel}':
                # value is PPS phase
                phase = float(value)
                freq = phase_diff.feed(phase)

                if qerr_val.value is None:
                    log.warning(f"No quantization error value available.")
                    continue

                freq_corr = phase_diff_corr.feed(phase + qerr)

                if freq_corr is not None:
                    filt_freq, filt_drift = filt.feed(freq_corr)
                    log.debug(f"freq={freq_corr} qerr={qerr} filt_freq={filt_freq}")

                if freq is not None and filt_freq is not None:
                    # append sample to circular buffer
                    buf.append(freq_corr, filt_freq, qerr, freq)

                    # plot results
                    ax1.clear()
                    ax1.plot(buf.get(), label=labels)
                    ax1.grid()
                    ax1.legend()

    ani = FuncAnimation(fig, animate, interval=args.period * 1000, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        gpsr.terminate()
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
    p.add_argument('--gpsport', default='/dev/ttyACM0', help="U-Blox GPS receiver port.")
    p.add_argument('--gpsspeed', type=int, default=9600, help="GPS baud rate.")
    p.add_argument('host')
    args = p.parse_args()

    logging.basicConfig()
    log = logging.getLogger('gpsfilter')
    log.setLevel(logging.DEBUG)

    main()
