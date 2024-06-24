from time import time
from typing import List, Any
from capture_msg_def import CaptureMessageDefinition, ScenarioHyperParameter
from capture_pb2 import CaptureMessageDefGroup, CaptureTopicTimestampLog


class CaptureMessage:
    messages_received: bool
    messages_received_second: bool
    timestamp: float
    timestamp_second: float
    image_topics: List[str]
    use_second_timestamp: dict[str, bool] = {}

    timestamp_log: CaptureTopicTimestampLog

    msg_dict: dict[str, Any] = {}

    def __init__(
        self,
        definition: CaptureMessageDefinition,
        hyperparameter: ScenarioHyperParameter,
    ):
        self.msg_dict = {}
        self.messages_received = False
        self.messages_received_second = False
        self.timestamp = time()
        self.timestamp_second = time()
        self.use_second_timestamp = {}
        self.definition = definition
        self.timestamp_log = CaptureTopicTimestampLog()
        self.hyperparamter = hyperparameter

    def get_message(self, group: CaptureMessageDefGroup):
        return [
            self.msg_dict[x.topic] if x.topic in self.msg_dict else None
            for x in group.messages
        ]

    def check_messages_received(self):
        keys = [
            [
                msg.topic
                for group in self.definition.entries()
                if group.enabled
                for msg in group.messages
                if msg.topic not in self.use_second_timestamp.keys()
            ],
            self.use_second_timestamp.keys(),
        ]
        msg_flag = [True, len(keys[1]) > 0]
        for i in range(2):
            for k in keys[i]:
                topic_def = self.definition.resolve_topic(k)
                if topic_def is None:
                    return
                if k in self.msg_dict and self.msg_dict[k]:
                    if (
                        topic_def.interpolation < 1
                        or len(self.msg_dict[k]) >= topic_def.interpolation
                    ):
                        continue
                msg_flag[i] = False
                break
        self.messages_received, self.messages_received_second = msg_flag

    def update_msg(self, topic: str, msg):
        # if "jai" in topic:
        #      print(topic, " behind :", time() - msg.header.stamp.sec, msg.header.stamp)
        topic_def = self.definition.resolve_topic(topic)
        if not topic_def:
            return
        if topic in self.msg_dict and topic_def.interpolation < 1:
            return

        topic_delay = topic_def.delay
        if (
            msg.header.stamp.sec + msg.header.stamp.nanosec / 1000000000 - topic_delay
            >= (
                self.timestamp_second
                if topic in self.use_second_timestamp
                else self.timestamp
            )
        ):
            if topic in self.msg_dict and topic_def.interpolation > 0:
                self.msg_dict[topic].append(msg)
            else:
                self.msg_dict[topic] = msg if topic_def.interpolation < 1 else [msg]

            self.timestamp_log.logs.append(
                CaptureTopicTimestampLog.TimestampLog(
                    topic=topic,
                    timestamp=msg.header.stamp.sec
                    + msg.header.stamp.nanosec / 1000000000,
                    delay_to_system=time()
                    - msg.header.stamp.sec
                    - msg.header.stamp.nanosec / 1000000000,
                )
            )

            self.check_messages_received()

    def vacate_second_msg_dict(self, group: CaptureMessageDefGroup):
        self.messages_received_second = False
        self.use_second_timestamp = {}
        for msg in group.messages:
            if msg.topic in self.msg_dict:
                del self.msg_dict[msg.topic]
            self.use_second_timestamp[msg.topic] = True
        self.timestamp_second = time()
