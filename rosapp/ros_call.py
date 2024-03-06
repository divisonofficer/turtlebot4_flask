import rclpy
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSLivelinessPolicy,
)

from rclpy.node import Node
from service_type_map import service_type_map, service_type_qos_map
from array import array
import subprocess
import re

ROBOT_NAMESPACE = ""  # "cgbot1/"


import numpy as np
from time import time


class SimpleSubscriber(Node):

    def __init__(self):

        name = str(time())[-3:]

        super().__init__("simple_subscriber" + name)
        self.runningSubscriptions = dict()

    def add_subscription(self, topic_name, topic_type, callback, qos_profile):
        if topic_name in self.runningSubscriptions:
            return False

        print(
            f"Adding subscription to {topic_name} with type {topic_type}, qos_profile {qos_profile}"
        )
        subscription = self.create_subscription(
            topic_type,
            topic_name,
            callback,
            qos_profile=qos_profile,
        )
        self.runningSubscriptions[topic_name] = subscription
        print(self.subscriptions)
        return True

    def remove_subscription(self, topic_name):
        if topic_name not in self.runningSubscriptions:
            return False
        self.destroy_subscription(self.runningSubscriptions[topic_name])
        del self.runningSubscriptions[topic_name]
        return True


class RosPyManager:
    def __init__(self):
        rclpy.init()
        self.simple_subscriber = SimpleSubscriber()

    def ros2_message_to_dictionary(self, message):
        """
        Convert a ROS2 message to a Python dictionary, making it serializable to JSON.
        This function handles nested messages, arrays of messages, NumPy arrays,
        Python array.array, and simple data types.
        """
        if hasattr(message, "get_fields_and_field_types"):  # It's a ROS2 message
            output = {}
            for field in message.get_fields_and_field_types():
                value = getattr(message, field)
                output[field] = self.ros2_message_to_dictionary(
                    value
                )  # Recursive conversion for nested types
            return output
        elif isinstance(message, list) or isinstance(message, np.ndarray):
            # Convert each element in the list or NumPy array, which might be a ROS2 message or a basic data type
            return [self.ros2_message_to_dictionary(element) for element in message]
        elif isinstance(message, array):  # Check if it's an instance of array.array
            # Convert array.array to list
            return message.tolist()
        else:
            # For basic data types that are directly serializable (int, float, string, etc.)
            return message

    # Example usage remains the same

    def service_type_resolution(self, service_type):
        # if service_type is a string, resolve it to the service type
        if not isinstance(service_type, str):
            return service_type
        if "/" not in service_type:
            raise ValueError(f"Invalid service type: {service_type}")
        if service_type not in service_type_map:
            raise ValueError(f"Service type not found: {service_type}")

        return service_type_map[service_type]

    def subscribe_topic(self, topic_name, topic_type, callback):
        return self.simple_subscriber.add_subscription(
            topic_name,
            self.service_type_resolution(topic_type),
            callback,
            self.create_qos_profile_from_dict(self.get_qos_profile(topic_name)),
        )

    def get_qos_profile(self, topic_name):
        # Execute the command to get verbose topic info
        result = subprocess.run(
            ["ros2", "topic", "info", "-v", topic_name],
            stdout=subprocess.PIPE,
            text=True,
        )
        output = result.stdout

        # Regular expression to find the QoS profile block
        qos_profile_pattern = re.compile(
            r"QoS profile:\n"
            r"  Reliability: (\w+)\n"
            r"  History \(Depth\): (\w+)\n"
            r"  Durability: (\w+)\n"
            r"  Lifespan: (\w+)\n"
            r"  Deadline: (\w+)\n"
            r"  Liveliness: (\w+)\n"
            r"  Liveliness lease duration: (\w+)",
            re.MULTILINE,
        )

        # Search for the QoS profile in the command output
        match = qos_profile_pattern.search(output)
        if match:
            qos_profile = {
                "Reliability": match.group(1),
                "History_Depth": match.group(2),
                "Durability": match.group(3),
                "Lifespan": match.group(4),
                "Deadline": match.group(5),
                "Liveliness": match.group(6),
                "Liveliness lease duration": match.group(7),
            }
            return qos_profile
        else:
            return "QoS Profile not found."

    def create_qos_profile_from_dict(self, qos_dict):
        # Map string values to rclpy.qos enums and values
        reliability_map = {
            "RELIABLE": QoSReliabilityPolicy.RELIABLE,
            "BEST_EFFORT": QoSReliabilityPolicy.BEST_EFFORT,
        }

        durability_map = {
            "VOLATILE": QoSDurabilityPolicy.VOLATILE,
            "TRANSIENT_LOCAL": QoSDurabilityPolicy.TRANSIENT_LOCAL,
        }

        liveliness_map = {
            "AUTOMATIC": QoSLivelinessPolicy.AUTOMATIC,
            "MANUAL_BY_TOPIC": QoSLivelinessPolicy.MANUAL_BY_TOPIC,
            # Add other mappings as necessary
        }

        # Assuming default values for certain parameters
        history_policy = QoSHistoryPolicy.KEEP_LAST
        depth = (
            10  # Default depth, since 'UNKNOWN' isn't practical for programmatic use
        )

        reliability = reliability_map.get(
            qos_dict["Reliability"], QoSReliabilityPolicy.SYSTEM_DEFAULT
        )
        durability = durability_map.get(
            qos_dict["Durability"], QoSDurabilityPolicy.SYSTEM_DEFAULT
        )
        liveliness = liveliness_map.get(
            qos_dict["Liveliness"], QoSLivelinessPolicy.SYSTEM_DEFAULT
        )

        # Create the QoSProfile
        qos_profile = QoSProfile(
            history=history_policy,
            depth=depth,
            reliability=reliability,
            durability=durability,
            liveliness=liveliness,
            # You might need to handle conversion for 'Lifespan', 'Deadline', and 'Liveliness lease duration'
            # if your application specifically requires them. Here, we're using defaults.
        )

        return qos_profile

    def unsubscribe_topic(self, topic_name):
        return self.simple_subscriber.remove_subscription(topic_name)

    def call_ros2_service(self, service_name, service_type, request_data=None):
        node = rclpy.create_node("flask_ros2_service_client")
        service_type = self.service_type_resolution(service_type)
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
