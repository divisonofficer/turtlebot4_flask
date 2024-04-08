import rclpy
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSLivelinessPolicy,
)

from rclpy.node import Node
from .service_type_map import service_type_map, service_type_qos_map
from array import array
import subprocess
import re
import json
import threading

ROBOT_NAMESPACE = ""  # "cgbot1/"


import numpy as np
from time import time

from typing import Union, Optional


class SimpleSubscriber(Node):
    runningSubscriptions: dict
    runningPublishers: dict

    def __init__(self):
        super().__init__("client_subscriber")
        self.runningSubscriptions = dict()
        self.runningPublishers = dict()

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

    def add_publisher(self, topic_name, topic_type, qos_profile):
        if topic_name in self.runningPublishers:
            return self.runningPublishers[topic_name]
        print(
            f"Adding publisher to {topic_name} with type {topic_type}, qos_profile {qos_profile}"
        )
        publisher = self.create_publisher(
            topic_type,
            topic_name,
            qos_profile=qos_profile,
        )
        self.runningPublishers[topic_name] = publisher
        return publisher


class RosPyManager:
    scan_thread: Optional[threading.Thread] = None

    def __init__(self):
        rclpy.init()
        self.simple_subscriber = SimpleSubscriber()
        self.ros_viewer = Node("client_viewer")
        self.topic_nodes_dict = {}

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
            # if value is Infinity, return null
            if message == "Infinity":
                return None
            return message

    # Example usage remains the same

    def service_type_resolution(self, service_type):
        """
        From a string of service_type, resolve it to the service type
        """
        # if service_type is a string, resolve it to the service type
        if not isinstance(service_type, str):
            return service_type
        if "/" not in service_type:
            raise ValueError(f"Invalid service type: {service_type}")
        if service_type not in service_type_map:
            raise ValueError(f"Service type not found: {service_type}")

        return service_type_map[service_type]

    def subscribe_topic(self, topic_name, topic_type, callback):
        """
        Create Topic Subscription
        args:
            topic_name: str
            topic_type: str
            callback: function(ros2 message type)

        if qos profile is not provided by the system, it will be crashed.
        """
        # qos_profile = self.create_qos_profile_from_dict(
        #     self.get_qos_profile(topic_name)["Publishers"][0]["QoS profile"]
        # )

        qos_profile = self.ros_viewer.get_publishers_info_by_topic(topic_name)[
            0
        ].qos_profile

        return self.simple_subscriber.add_subscription(
            topic_name,
            self.service_type_resolution(topic_type),
            callback,
            qos_profile=qos_profile,
        )

    def publish_topic(self, topic_name, topic_type, data):
        """
        Publish data to a topic
        args:
            topic_name: str
            data: dict
        """
        topic_type = self.service_type_resolution(topic_type)
        publisher = self.simple_subscriber.add_publisher(
            topic_name,
            topic_type,
            10,
        )
        if type(data) is dict:
            data = self.ros2_dict_to_message(topic_type, data)
        publisher.publish(data)

    def ros2_dict_to_message(self, message_type, dictionary):
        message_class = self.service_type_resolution(message_type)
        message = message_class()
        for field, value in dictionary.items():
            if hasattr(message, field):
                if isinstance(value, dict):
                    setattr(
                        message,
                        field,
                        self.ros2_dict_to_message(
                            getattr(message, field).__class__, value
                        ),
                    )
                else:
                    # Convert number to float
                    if isinstance(value, int):
                        value = float(value)
                    setattr(message, field, value)
        return message

    def create_qos_profile_from_dict(self, qos_dict):
        """
        From a dictionary of QoS settings, create a QoSProfile object
        """
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

        try:
            # ROS2 서비스 호출 명령어 실행
            result = subprocess.run(
                [
                    "ros2",
                    "service",
                    "call",
                    service_name,
                    service_type,
                    json.dumps(request_data),
                ],
                stdout=subprocess.PIPE,
                text=True,
                timeout=10,  # 타임아웃 설정 (초 단위)
            )
            # 명령어 실행 결과 반환
            return result.stdout
        except subprocess.TimeoutExpired:
            # 타임아웃 발생 시 오류 메시지 반환
            return "Error: The command timed out."

        node = self.simple_subscriber
        service_type = self.service_type_resolution(service_type)
        client = node.create_client(service_type, ROBOT_NAMESPACE + service_name)
        holds = 0
        while not client.wait_for_service(timeout_sec=1.0):
            print("Service not available, waiting again...")
            holds = holds + 1
            if holds > 5:
                break
        req = service_type.Request()

        # Populate request data here based on the service definition
        # For example, for a simple on/off service:
        if request_data:
            req.data = request_data

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        return future.result()

    def get_type_json_format(self, type_name):
        """
        Get type in json format
        """
        type_class = self.service_type_resolution(type_name)
        item = type_class()
        return self.ros2_message_to_dictionary(item)

    def get_topic_list(self):
        """
        Get list of topics
        """
        result = self.ros_viewer.get_topic_names_and_types()
        self.scan_topic_node_thread([topic for topic, _ in result])
        return [
            {
                "topic": topic,
                "type": type,
                "nodes": (
                    self.topic_nodes_dict[topic]
                    if topic in self.topic_nodes_dict
                    else self.get_topic_nodes_list(topic)
                ),
            }
            for topic, type in result
        ]

    def get_node_list(self):
        """
        Get list of nodes
        """
        result = self.ros_viewer.get_node_names_and_namespaces()
        return [{"name": name, "namespace": namespace} for name, namespace in result]

    def get_services_list(self):
        """
        Get List of Service (callable), including parameter types & returns
        """
        result = self.ros_viewer.get_service_names_and_types()
        return [{"service": service, "type": type[0]} for service, type in result]

    def get_node_detail(self, node_name, namespace):
        clients = self.ros_viewer.get_client_names_and_types_by_node(
            node_name, namespace
        )
        services = self.ros_viewer.get_service_names_and_types_by_node(
            node_name, namespace
        )
        publishers = self.ros_viewer.get_publisher_names_and_types_by_node(
            node_name, namespace
        )
        subscriptions = self.ros_viewer.get_subscriber_names_and_types_by_node(
            node_name, namespace
        )

        return {
            "clients": [{"name": x[0], "type": x[1]} for x in clients],
            "services": [
                {"name": service, "type": type[0]} for service, type in services
            ],
            "publishers": [
                {"name": publisher[0], "type": publisher[1]} for publisher in publishers
            ],
            "subscriptions": [
                {"name": subscription[0], "type": subscription[1]}
                for subscription in subscriptions
            ],
        }

    def get_topic_nodes_list(self, topic_name):
        """
        Get list of nodes for each topic
        """
        publishers = self.ros_viewer.get_publishers_info_by_topic(topic_name)
        subscriptions = self.ros_viewer.get_subscriptions_info_by_topic(topic_name)

        def endpointinfo_dict(info):
            return {
                "name": info.node_name,
                "namespace": info.node_namespace,
                "topicType": info.topic_type,
                "endpointType": info.endpoint_type.name,
                "gid": info.endpoint_gid,
                "qosProfile": {
                    "reliability": info.qos_profile.reliability.name,
                    "depthHistory": info.qos_profile.history.name,
                    "durability": info.qos_profile.durability.name,
                    "lifespan": info.qos_profile.lifespan.nanoseconds,
                    "deadline": info.qos_profile.deadline.nanoseconds,
                    "liveliness": info.qos_profile.liveliness.name,
                    "livelinessLeaseDuration": info.qos_profile.liveliness_lease_duration.nanoseconds,
                },
            }

        publishers = [endpointinfo_dict(publisher) for publisher in publishers]
        subscriptions = [
            endpointinfo_dict(subscription) for subscription in subscriptions
        ]
        self.topic_nodes_dict[topic_name] = {
            "publishers": publishers,
            "subscriptions": subscriptions,
        }
        return {"publishers": publishers, "subscriptions": subscriptions}

    def scan_topic_node_thread(self, topics):
        time_begin = time()
        for topic in topics:
            self.get_topic_nodes_list(topic)
        self.scan_thread = None
        time_taken = time() - time_begin
        self.ros_viewer.get_logger().info(f"Scanning topics took {time_taken} seconds")
