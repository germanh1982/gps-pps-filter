#!./bin/python3
from argparse import ArgumentParser
from serial import Serial
from pyubx2 import UBXReader
import paho.mqtt.client as mqttclient
from time import sleep

def main():
    # MQTT client setup
    client = mqttclient.Client()
    client.connect(args.host)
    client.loop_start()

    # serial port setup
    stream = Serial(args.port, args.speed, timeout=args.timeout)
    ubr = UBXReader(stream)

    # main loop
    while True:
        (raw_data, parsed_data) = ubr.read()
        if parsed_data.identity == 'TIM-TP':
            client.publish(args.topic, parsed_data.qErr)
            print(parsed_data)

        sleep(0.001)

if __name__ == '__main__':
    p = ArgumentParser()
    p.add_argument('--timeout', type=int, default=3)
    p.add_argument('--speed', type=int, default=9600)
    p.add_argument('--topic', default='qerr')
    p.add_argument('port')
    p.add_argument('host') # 192.168.0.149
    args = p.parse_args()

    try:
        main()
    except KeyboardInterrupt:
        pass
