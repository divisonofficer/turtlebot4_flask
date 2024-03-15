from time import time


class ManualTopicLogger:
    __slots__ = ["topic", "interval", "time", "logs", "count"]

    def __init__(self, topic):
        self.topic = topic
        self.interval = None
        self.logs = []
        self.count = 0
        self.time = time()

    def log(self, data):
        self.count += 1
        # self.logs.append(data)
        self.interval = time() - self.time
        self.time = time()

        # if len(self.logs) > 10:
        #    self.logs.pop(0)

        return self.interval

    def get_dict(self):
        return {
            "topic": self.topic,
            "interval": self.interval,
            "count": self.count,
            "logs": self.logs,
            "time": self.time,
        }


class ManualTopicManager:
    topics = {}

    def __init__(
        self,
        socket,
        socket_namespace: str,
        controller,
    ):
        self.socket = socket
        self.controller = controller
        self.socket_namespace = socket_namespace

    def get_callback(self, topic_name):
        def callback(data):
            data = self.controller.rospy.ros2_message_to_dictionary(data)
            data["interval"] = self.topics[topic_name].log(data)
            self.socket.emit(topic_name, data, namespace=self.socket_namespace)

        return callback

    def register_topic(self, topic_name, topic_type):
        if topic_name in self.topics:
            return None

        if not self.controller.manual_topic_subscription(
            topic_name,
            topic_type,
            self.get_callback(topic_name),
        ):
            print("Failed to register topic", topic_name, topic_type)
            return None
        print("Registered topic", topic_name, topic_type)
        self.topics[topic_name] = ManualTopicLogger(topic_name)
        return True

    def delete_topic(self, topic_name):
        if topic_name not in self.topics:
            return 400
        del self.topics[topic_name]
        self.controller.ros_unsubscribe_topic(topic_name)
        return 200
