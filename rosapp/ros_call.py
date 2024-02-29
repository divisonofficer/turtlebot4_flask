import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool, Trigger
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from rclpy.executors import MultiThreadedExecutor
from service_type_map import service_type_map
from array import array

ROBOT_NAMESPACE = ""  # "cgbot1/"


import numpy as np


def ros2_message_to_dictionary(message):
    """
    Convert a ROS2 message to a Python dictionary, making it serializable to JSON.
    This function handles nested messages, arrays of messages, NumPy arrays,
    Python array.array, and simple data types.
    """
    if hasattr(message, "get_fields_and_field_types"):  # It's a ROS2 message
        output = {}
        for field in message.get_fields_and_field_types():
            value = getattr(message, field)
            output[field] = ros2_message_to_dictionary(
                value
            )  # Recursive conversion for nested types
        return output
    elif isinstance(message, list) or isinstance(message, np.ndarray):
        # Convert each element in the list or NumPy array, which might be a ROS2 message or a basic data type
        return [ros2_message_to_dictionary(element) for element in message]
    elif isinstance(message, array):  # Check if it's an instance of array.array
        # Convert array.array to list
        return message.tolist()
    else:
        # For basic data types that are directly serializable (int, float, string, etc.)
        return message


# Example usage remains the same


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
