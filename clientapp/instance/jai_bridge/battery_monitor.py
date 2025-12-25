import subprocess
import time
import threading
from typing import Dict, Optional, List
from flask_socketio import SocketIO


class BatteryMonitor:
    """
    Battery monitoring module for JAI Bridge application.
    Monitors laptop battery status and emits updates via SocketIO.
    """

    def __init__(self, socketio: SocketIO):
        self.socketio = socketio
        self.battery_data: List[Dict] = []
        self.data_lock = threading.Lock()
        self.monitoring_thread: Optional[threading.Thread] = None
        self.running = False
        self.battery_available = self._check_battery_availability()

    def _check_battery_availability(self) -> bool:
        """Check if battery exists on this system"""
        try:
            result = subprocess.run(
                ["test", "-d", "/sys/class/power_supply/BAT0"],
                capture_output=True
            )
            return result.returncode == 0
        except Exception:
            return False

    def read_battery_file(self, file_path: str) -> Optional[str]:
        """Utility function to read a file from battery info directory"""
        try:
            result = subprocess.run(
                ["cat", file_path],
                capture_output=True,
                text=True,
                check=True
            )
            return result.stdout.strip()
        except subprocess.CalledProcessError:
            return None

    def get_battery_info(self) -> Dict:
        """Get current battery information"""
        if not self.battery_available:
            return {
                "SoC": 0,
                "Energy Now (Wh)": "N/A",
                "Energy Full (Wh)": "N/A",
                "State": "No Battery",
                "Measurement Time": time.time(),
                "available": False
            }

        base_path = "/sys/class/power_supply/BAT0"

        files_to_read = {
            "SoC": f"{base_path}/capacity",
            "Energy Now (Wh)": f"{base_path}/energy_now",
            "Energy Full (Wh)": f"{base_path}/energy_full",
            "State": f"{base_path}/status",
            "Measurement Time": "timestamp",
        }

        battery_info = {}

        for key, file_path in files_to_read.items():
            if key == "Measurement Time":
                battery_info[key] = time.time()
            else:
                value = self.read_battery_file(file_path)
                battery_info[key] = value if value is not None else "N/A"

        # Calculate energy in Wh if possible
        if battery_info["Energy Now (Wh)"] != "N/A" and battery_info["Energy Full (Wh)"] != "N/A":
            try:
                energy_now_wh = int(battery_info["Energy Now (Wh)"]) / 1e6  # Convert µWh to Wh
                energy_full_wh = int(battery_info["Energy Full (Wh)"]) / 1e6  # Convert µWh to Wh
                battery_info["Energy Now (Wh)"] = f"{energy_now_wh} Wh"
                battery_info["Energy Full (Wh)"] = f"{energy_full_wh} Wh"
            except (ValueError, TypeError):
                pass

        battery_info["available"] = True
        return battery_info

    def monitor_battery(self):
        """Background thread function to monitor battery"""
        max_entries = 60  # Maximum entries to hold 1 hour of data
        while self.running:
            try:
                info = self.get_battery_info()
                if info:
                    with self.data_lock:
                        self.battery_data.append(info)
                        self.battery_data = self.battery_data[-max_entries:]
                    # Emit to default namespace
                    self.socketio.emit("/battery", info)
            except Exception as e:
                print(f"Battery monitoring error: {e}")

            time.sleep(60)  # Update every 60 seconds

    def start_monitoring(self):
        """Start the battery monitoring thread"""
        if self.monitoring_thread and self.monitoring_thread.is_alive():
            return

        self.running = True
        self.monitoring_thread = threading.Thread(
            target=self.monitor_battery,
            daemon=True
        )
        self.monitoring_thread.start()
        print(f"Battery monitoring started (available: {self.battery_available})")

    def stop_monitoring(self):
        """Stop the battery monitoring thread"""
        self.running = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=5)

    def get_latest_battery_data(self) -> Dict:
        """Get the most recent battery reading"""
        with self.data_lock:
            if not self.battery_data:
                return self.get_battery_info()
            return self.battery_data[-1]
