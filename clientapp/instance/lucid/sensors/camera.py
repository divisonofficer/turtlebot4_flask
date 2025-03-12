from dataclasses import dataclass, asdict
from typing import Literal
from .sensor import Sensor


class Camera(Sensor):
    @dataclass
    class Const(Sensor.Const):
        srcs: int = 1
        raw_format: Literal["rgb", "bayer", "gray", "depth"] = "rgb"
        raw_bits: int = 8
        width: int = 640
        height: int = 480

    @dataclass
    class State(Sensor.State):
        pass

    def __init__(self, name: str, const: Const):
        state = self.State()
        super().__init__(name, const, state)
        self.const = const

    def register_callback(self, callback):
        self.frame_callback = callback

    def start_stream(self):
        pass

    def stop_stream(self):
        pass

    def post_process_thumbnail(self, frame, frame_id: str):
        return frame

    def get_status(self):
        return asdict(self.state)

    def get_config_status(self):
        return {}
