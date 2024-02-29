import json


class ConfigManager:
    def __init__(self):
        # read ros2 topics from ros2_topics.json
        file = open("ros2_topics.json", "r")
        self.ros2_topics = json.load(file)


configManager = ConfigManager()
