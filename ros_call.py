import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool, Trigger
from sensor_msgs.msg import Image
import threading
from rclpy.executors import MultiThreadedExecutor

service_type_map = {
    "std_srvs/srv/Empty": Empty,
    "std_srvs/srv/SetBool": SetBool,
    "std_srvs/srv/Trigger": Trigger,
    "sensor_msgs/msg/Image": Image,
}

ROBOT_NAMESPACE = ""  # "cgbot1/"


def service_type_resolution(service_type):
    # if service_type is a string, resolve it to the service type
    if not isinstance(service_type, str):
        return service_type
    if "/" not in service_type:
        raise ValueError(f"Invalid service type: {service_type}")
    if service_type not in service_type_map:
        raise ValueError(f"Service type not found: {service_type}")

    return service_type_map[service_type]


class SimpleSubscriber(Node):
    def __init__(self, topic_type, topic_name, callback):
        super().__init__("simple_subscriber")
        self.subscription = self.create_subscription(
            topic_type,  # Replace String with your actual message type
            topic_name,
            callback,
            10,
        )
        self.subscription  # prevent unused variable warning


def executor_thread(nodes: list[Node]):
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    def spin():
        try:
            executor.spin()
        finally:
            executor.shutdown()
            for node in nodes:
                node.destroy_node()
            rclpy.shutdown()
        # Spin in a separate thread

    thread = threading.Thread(target=spin)
    thread.start()


def subscribe_topic(topic_name, topic_type, callback):
    topic_type = service_type_resolution(topic_type)
    return SimpleSubscriber(topic_type, ROBOT_NAMESPACE + topic_name, callback)


def call_ros2_service(service_name, service_type, request_data=None):
    node = rclpy.create_node("flask_ros2_service_client")
    service_type = service_type_resolution(service_type)
    client = node.create_client(service_type, ROBOT_NAMESPACE + service_name)
    while not client.wait_for_service(timeout_sec=1.0):
        print("Service not available, waiting again...")
    req = service_type.Request()

    # Populate request data here based on the service definition
    # For example, for a simple on/off service:
    if request_data:
        req.data = request_data

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result()
