from time import time


class SocketEmit:

    prev_time = None
    topic = None
    interval = 0.1

    def __init__(self, socketio, topic, interval=0.1):
        self.socketio = socketio
        self.prev_time = time()
        self.interval = interval
        self.topic = topic

    def emit(self, event, data):
        if time() - self.prev_time > self.interval:
            self.socketio.emit(event, data, namespace=self.topic)
            self.prev_time = time()

    def getCallback(self, event):
        def callback(data):
            print("Emit Data : ", data)
            self.emit(event, data)

        return callback
