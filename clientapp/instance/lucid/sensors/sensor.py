from dataclasses import dataclass, field
from typing import Any, Dict, List, Literal


class Sensor:
    @dataclass
    class Frame:
        timestamp: float
        data: Dict[str, Any]
        attrs: Dict[str, Any]
        file_format: Dict[str, Literal["png", "npy", "tiff"]]

    @dataclass
    class State:
        timestamp_last: float = 0
        fps: float = 0
        stream_on: bool = False
        preview_on: bool = False
        preview_last: float = 0
        preview_interval: float = 3.0

    @dataclass
    class Const:
        preview_enabled: bool = True
        preview_keys: List[str] = field(default_factory=list)

    def __init__(self, name: str, const: Const, state: State):
        self.name = name
        self.const = const
        self.state = state
