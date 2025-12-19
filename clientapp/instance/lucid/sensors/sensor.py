from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Literal


class Sensor:
    @dataclass
    class Frame:
        timestamp: float
        data: Dict[str, Any]
        attrs: Dict[str, Any]
        file_format: Dict[str, Literal["png", "npy", "tiff", "exr"]]

    @dataclass
    class State:
        timestamp_last: float = 0
        fps: float = 0
        device_status: str = (
            "disconnected"  # disconnected, connecting, connected, error
        )
        stream_on: bool = False
        preview_on: bool = False
        preview_last: float = 0
        preview_interval: float = 1.0

    @dataclass
    class Const:
        preview_enabled: bool = True
        preview_keys: List[str] = field(default_factory=list)

    class Config:
        configs: Dict[str, Any]

        def __init__(self, configs: Dict[str, Any] = {}):
            self.configs = configs

        @dataclass
        class Param:
            name: str
            value: Any
            range: List[float] = field(default_factory=list)
            type: Literal["int", "float", "bool", "enum", "str"] = "float"
            enum_list: List[str] = field(default_factory=list)
            unit: str = ""

        def update_config(self, name: str, value: Any):
            raise NotImplementedError("update_config method not implemented")

        def get_config(self, name: str):
            raise NotImplementedError("get_config method not implemented")

        def get_dict(self):
            return dict(
                sorted(
                    {
                        name: asdict(config) for name, config in self.configs.items()
                    }.items()
                )
            )

    def refresh_config(self):
        pass

    def update_config(self, name: str, value: Any):
        if name in self.config.configs:
            self.config.update_config(name, value)
        else:
            raise ValueError(f"Config {name} not found in {self.name}")

    def launch_device(self):
        raise NotImplementedError("launch_device method not implemented")

    def validate_device(self):
        raise NotImplementedError("validate_device method not implemented")

    def trigger_device(self):
        raise NotImplementedError("trigger_device method not implemented")

    def __init__(
        self, name: str, const: Const, state: State, config: Config = Config()
    ):
        self.name = name
        self.const = const
        self.state = state
        self.config = config
