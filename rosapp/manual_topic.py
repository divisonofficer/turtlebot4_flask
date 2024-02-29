from ros_call import ros2_message_to_dictionary


class ManualTopicManager:
    topics = {}

    def __init__(self, socket, controller, executor):
        self.socket = socket
        self.executor = executor
        self.controller = controller

    def get_callback(self, topic_name):
        def callback(data):
            data = ros2_message_to_dictionary(data)
            self.socket.emit(topic_name, data, namespace="/manual")

        return callback

    def register_topic(self, topic_name, topic_type):
        if topic_name in self.topics:
            return None

        node = self.controller.manual_topic_subscription(
            topic_name, topic_type, self.get_callback(topic_name)
        )
        self.topics[topic_name] = node
        self.executor.executor_thread([node])

        return node

    def delete_topic(self, topic_name):
        if topic_name not in self.topics:
            return
        node = self.topics[topic_name]
        self.executor.remove_node_runtime(node)
        node.destroy_subscription()
        del self.topics[topic_name]
        return
