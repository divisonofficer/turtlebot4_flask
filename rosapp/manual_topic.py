from ros_executor import RosExecutor
from controller import Controller


class ManualTopicManager:
    topics = {}

    def __init__(self, socket, controller: Controller, executor: RosExecutor):
        self.socket = socket
        self.executor = executor
        self.controller = controller

    def get_callback(self, topic_name):
        def callback(data):
            data = self.controller.rospy.ros2_message_to_dictionary(data)
            self.socket.emit(topic_name, data, namespace="/manual")

        return callback

    def register_topic(self, topic_name, topic_type):
        if topic_name in self.topics:
            return None

        if not self.controller.manual_topic_subscription(
            topic_name,
            topic_type,
            lambda data: self.socket.emit(
                topic_name,
                self.controller.rospy.ros2_message_to_dictionary(data),
                namespace="/manual",
            ),
        ):
            print("Failed to register topic", topic_name, topic_type)
            return None
        print("Registered topic", topic_name, topic_type)
        self.topics[topic_name] = True
        return True

    def delete_topic(self, topic_name):
        if topic_name not in self.topics:
            return
        del self.topics[topic_name]
        self.controller.ros_unsubscribe_topic(topic_name)
        return
