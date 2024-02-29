import subprocess, json
from tqdm import tqdm


def run_command(command):
    """Run a command in the subprocess and return its output"""
    result = subprocess.run(command, shell=True, text=True, capture_output=True)
    return result.stdout.strip()


def get_ros2_topic_list():
    """Get the list of ROS2 topics"""
    command = "ros2 topic list"
    topics = run_command(command).split("\n")
    return topics


def get_topic_info(topic):
    """Get the type of a ROS2 topic"""
    command = f"ros2 topic info {topic}"
    info = run_command(command)
    return info


# Get list of ROS2 topics
topics = get_ros2_topic_list()
topic_info = {}

# For each topic, find the response type
for topic in tqdm(topics, desc="Getting topic info"):
    info = get_topic_info(topic)
    type = info.split("\n")[0].split(": ")[1]
    topic_info[topic] = type


# save on json
with open("ros2_topics.json", "w") as f:
    json.dump(topic_info, f, indent=4)
