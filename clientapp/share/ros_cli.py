import subprocess
from typing import Dict, List, Tuple, Union


class RosCliManager:
    TOPIC_KEY_MAP = {
        "Node name": "name",
        "Node namespace": "namespace",
        "Topic type": "topicType",
        "Endpoint type": "endpointType",
        "GID": "gid",
        "QoS profile": "qosProfile",
        "Reliability": "reliability",
        "History (Depth)": "depthHistory",
        "Durability": "durability",
        "Lifespan": "lifespan",
        "Deadline": "deadline",
        "Liveliness": "liveliness",
        "Liveliness lease duration": "livelinessLeaseDuration",
    }

    def __init__(self):
        self.property_dict = {}
        self.topic_node_dict = {}

    def get_topic_names_and_types(self):
        output = subprocess.run(
            ["ros2", "topic", "list", "-t"], stdout=subprocess.PIPE, text=True
        )
        topics_output = output.stdout.split("\n")
        topics = [topic.split(" ") for topic in topics_output if topic]
        topics_with_type = [(topic[0], [topic[1][1:-1]]) for topic in topics]
        return topics_with_type

    def get_client_names_and_types_by_node(self, node_name, node_namespace):
        output = subprocess.run(
            ["ros2", "node", "info", "/" + node_name], stdout=subprocess.PIPE, text=True
        )
        node_info = [line.strip() for line in output.stdout.split("\n") if line]

        KEY_LIST = [
            "Subscribers:",
            "Publishers:",
            "Service Servers:",
            "Service Clients:",
            "Action Servers:",
            "Action Clients:",
        ]
        KEY_TRANS_MAP = {
            "Subscribers": "subscriptions",
            "Publishers": "publishers",
            "Service Servers": "servers",
            "Service Clients": "clients",
            "Action Servers": "action_servers",
            "Action Clients": "action_clients",
        }

        output_dict: Dict[str, List[Tuple[str, List[str]]]] = {}
        current_key = ""
        for line in node_info:
            if line == node_name:
                continue
            if line in KEY_LIST:
                current_key = KEY_TRANS_MAP[line[:-1]]
                output_dict[current_key] = []
                continue
            if current_key:
                [name, type] = line.split(" ")
                name = name[0:-1]
                output_dict[current_key].append((name, [type]))

        self.property_dict[node_name] = output_dict
        return list(output_dict["clients"])

    def get_service_names_and_types_by_node(self, node_name, node_namespace):
        return list(self.property_dict[node_name]["servers"])

    def get_publisher_names_and_types_by_node(self, node_name, node_namespace):
        return list(self.property_dict[node_name]["publishers"])

    def get_subscriber_names_and_types_by_node(self, node_name, node_namespace):
        return list(self.property_dict[node_name]["subscriptions"])

    def get_node_names_and_namespaces(self):
        output = subprocess.run(
            ["ros2", "node", "list"], stdout=subprocess.PIPE, text=True
        )
        nodes = output.stdout.split("\n")
        return [(node, "") for node in nodes if node]

    def get_service_names_and_types(self):
        output = subprocess.run(
            ["ros2", "service", "list", "-t"], stdout=subprocess.PIPE, text=True
        )
        services_output = output.stdout.split("\n")
        return [
            (service.split(" ")[0], [service.split("[")[0].split("]")[0]])
            for service in services_output
            if service
        ]

    def get_publishers_info_by_topic(self, topic_name):
        output = subprocess.run(
            ["ros2", "topic", "info", "-v", topic_name],
            stdout=subprocess.PIPE,
            text=True,
        )

        output_blocks = output.stdout.split("\n\n")

        publishers = []
        subscription = []
        for block in output_blocks:
            if not "Node name" in block:
                continue
            [node_profile, qos_profile] = block.split("QoS profile:")
            node_profile_lines = [
                line.strip() for line in node_profile.split("\n") if line
            ]
            qos_profile_lines = [
                line.strip() for line in qos_profile.split("\n") if line
            ]
            node_dict: Dict[str, Union[str, Dict]] = {}
            qos_profile_dict = {}
            for line in node_profile_lines:
                [key, value] = line.split(":")
                node_dict[self.TOPIC_KEY_MAP[key.strip()]] = value.strip()
            for line in qos_profile_lines:
                [key, value] = line.split(":")
                qos_profile_dict[self.TOPIC_KEY_MAP[key.strip()]] = value.strip()
            node_dict["qosProfile"] = qos_profile_dict
            if "PUBLISHER" in node_dict["endpointType"]:
                publishers.append(node_dict)
            else:
                subscription.append(node_dict)

        self.topic_node_dict[topic_name] = {
            "publishers": publishers,
            "subscriptions": subscription,
        }
        return publishers

    def get_subscriptions_info_by_topic(self, topic_name):
        return self.topic_node_dict[topic_name]["subscriptions"]


if __name__ == "__main__":
    manager = RosCliManager()
    container_name = "turtlebot4_base_node"

    print(manager.get_client_names_and_types_by_node(container_name, ""))
    print(manager.get_service_names_and_types_by_node(container_name, ""))
    print(manager.get_publisher_names_and_types_by_node(container_name, ""))
    print(manager.get_subscriber_names_and_types_by_node(container_name, ""))
