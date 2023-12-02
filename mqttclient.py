from queue import Queue, Empty
import paho.mqtt.client as mqttclient

class MQTTClient:
    def on_message(self, client, userdata, message):
        self._incoming.put(message)

    def on_connect(self, client, userdata, flags, rc):
        for topic in self._topics:
            client.subscribe(topic)

    def __init__(self, host, topics=[]):
        self._topics = topics
        self._incoming = Queue()

        self._conn = mqttclient.Client()
        self._conn.connect(host)
        self._conn.on_connect = self.on_connect
        self._conn.on_message = self.on_message
        self._conn.loop_start()

    def get(self):
        while True:
            try:
                m = self._incoming.get_nowait()
            except Empty:
                pass
            else:
                yield (m.topic, m.payload)
