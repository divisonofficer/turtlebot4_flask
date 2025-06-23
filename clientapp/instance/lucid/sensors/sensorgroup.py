from dataclasses import dataclass
import threading
from typing import List, Optional
from sensors.sensor import Sensor
import time


class SensorGroup:
    @dataclass
    class State:
        trigger_loop_on: bool = False
        trigger_loop_fps: int = 0

    class Entity:
        def __init__(self, sensor: Sensor, trigger_sync: bool = False):
            self.name = sensor.name
            self.sensor = sensor
            self.trigger_sync = trigger_sync

    def __init__(self, name: str, sensors: List[Entity], trigger_loop_fps: int = 0):
        self.name = name
        self.sensors = sensors
        self.state = self.State()
        self.state.trigger_loop_fps = trigger_loop_fps
        self.trigger_loop: Optional[threading.Thread] = None
        self.trigger_loop_stop = threading.Event()

    def trigger_sync(self):
        for entity in self.sensors:
            if entity.trigger_sync:
                entity.sensor.trigger_device()

    def trigger_loop_func(self):
        while not self.trigger_loop_stop.is_set():
            self.trigger_sync()
            if self.state.trigger_loop_fps > 0:
                time.sleep(1 / self.state.trigger_loop_fps)

            if self.state.trigger_loop_fps == 0 or self.trigger_loop_stop.is_set():
                self.trigger_loop_stop.clear()
                break
        self.trigger_loop = None

    def start_trigger_loop(self):
        if self.trigger_loop is not None:
            return
        self.trigger_loop_stop.clear()
        self.trigger_loop = threading.Thread(target=self.trigger_loop_func)
        self.trigger_loop.start()

    def stop_trigger_loop(self):
        if self.trigger_loop is not None:
            self.trigger_loop_stop.set()
            self.trigger_loop.join()
            self.trigger_loop = None
        self.state.trigger_loop_on = False
