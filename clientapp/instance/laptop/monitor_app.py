import subprocess
import re
import time
import threading
from datetime import datetime

from flask import Flask
from flask_socketio import SocketIO

# Global list to hold battery information


class BatteryMonitor:
    battery_data: list[dict]

    def __init__(self, socketio):
        self.socketio = socketio
        self.battery_data = []

    def read_battery_file(self, file_path):
        """Utility function to read a file from the battery info directory."""
        try:
            # Execute the command to read the file
            result = subprocess.run(
                ["cat", file_path], capture_output=True, text=True, check=True
            )
            return (
                result.stdout.strip()
            )  # Return the content, stripped of leading/trailing whitespace
        except subprocess.CalledProcessError:
            print(f"Failed to read {file_path}")
            return None

    def get_battery_info(self):
        base_path = "/sys/class/power_supply/BAT0"

        # Files to read from the battery info directory
        files_to_read = {
            "SoC": f"{base_path}/capacity",
            "Energy Now (Wh)": f"{base_path}/energy_now",
            "Energy Full (Wh)": f"{base_path}/energy_full",
            "State": f"{base_path}/status",
            "Measurement Time": "timestamp",  # This is not directly available, using current time instead
        }

        battery_info = {}

        for key, file_path in files_to_read.items():
            if key == "Measurement Time":
                # Since there's no direct measurement time file, use the current time instead
                battery_info[key] = time.time()
            else:
                battery_info[key] = self.read_battery_file(file_path)

        # Calculate energy in Wh if possible
        if battery_info["Energy Now (Wh)"] and battery_info["Energy Full (Wh)"]:
            energy_now_wh = (
                int(battery_info["Energy Now (Wh)"]) / 1e6
            )  # Convert µWh to Wh
            energy_full_wh = (
                int(battery_info["Energy Full (Wh)"]) / 1e6
            )  # Convert µWh to Wh
            battery_info["Energy Now (Wh)"] = f"{energy_now_wh} Wh"
            battery_info["Energy Full (Wh)"] = f"{energy_full_wh} Wh"

        return battery_info

    def monitor_battery(self):
        max_entries = 60  # Maximum entries to hold 1 hour of data (1 entry per minute)
        while True:
            print("Monitoring battery...")
            info = self.get_battery_info()
            if info:
                self.battery_data.append(info)
                self.battery_data[:] = self.battery_data[-max_entries:]
                self.socketio.emit("/battery", info, namespace="/socket/battery")
            time.sleep(60)

    def start_monitoring(self):
        thread = threading.Thread(target=self.monitor_battery)
        thread.start()
        print("Battery monitoring started")

    def predict_battery_life(self):
        if len(self.battery_data) < 2:
            return "Not enough data to predict battery life."

        # Calculate average change in SoC and Wh
        soc_changes = [
            (b["SoC"] - a["SoC"])
            for a, b in zip(self.battery_data, self.battery_data[1:])
        ]
        wh_changes = [
            (b["Wh"] - a["Wh"])
            for a, b in zip(self.battery_data, self.battery_data[1:])
        ]
        avg_soc_change = sum(soc_changes) / len(soc_changes)
        avg_wh_change = sum(wh_changes) / len(wh_changes)

        # Predict remaining battery life
        if avg_soc_change >= 0:
            return "Battery is charging. Can't predict when fully charged."
        remaining_soc = self.battery_data[-1]["SoC"]
        remaining_wh = self.battery_data[-1]["Wh"]
        hours_left_soc = -(remaining_soc / avg_soc_change) / 60
        hours_left_wh = -(remaining_wh / avg_wh_change) / 60

        return {
            "Average SoC change per minute": avg_soc_change,
            "Average Wh change per minute": avg_wh_change,
            "Estimated hours left (SoC based)": hours_left_soc,
            "Estimated hours left (Wh based)": hours_left_wh,
        }


app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

monitor = BatteryMonitor(socketio)


@socketio.on("connect", namespace="/socket/battery")
def handle_connect():
    print("Battery monitor connected")


@app.route("/current", methods=["GET"])
def index():
    return monitor.battery_data[-1]


with app.app_context():
    monitor.start_monitoring()

if __name__ == "__main__":
    socketio.run(app, port=5013, allow_unsafe_werkzeug=True, host="0.0.0.0")
