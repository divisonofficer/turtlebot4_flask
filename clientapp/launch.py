from flask import Flask


app = Flask(__name__)


process_map = {
    "main": "app.py",
    "laptop_monitor": "instance/laptop/monitor_app.py",
    "slam": "instance/slam/slam_app.py",
}

import subprocess


class ProcessManager:
    def __init__(self):
        self.processes = {}

        for process_name in process_map.keys():
            self.launch_process(process_name)

    def launch_process(self, process_key: str):
        if process_key in self.processes:
            return {
                "status": "error",
                "message": f"{process_key} already running",
            }
        process = subprocess.Popen(["python3", process_map[process_key]])
        self.processes[process_key] = process
        return {
            "status": "success",
            "message": f"{process_key} launched successfully",
        }

    def cancel_process(self, process_key: str):
        if process_key not in self.processes:
            return {
                "status": "error",
                "message": f"{process_key} not running",
            }
        process = self.processes[process_key]
        process.terminate()
        self.processes.pop(process_key)
        return {
            "status": "success",
            "message": f"{process_key} cancelled successfully",
        }

    def process_running_status(self):
        process_status = {}
        for process_name in process_map.keys():
            process_status[process_name] = process_name in self.processes
        return process_status


process_manager = ProcessManager()


@app.route("/launch/<process_key>", methods=["GET"])
def launch_process(process_key):
    return process_manager.launch_process(process_key)


@app.route("/cancel/<process_key>", methods=["GET"])
def cancel_process(process_key):
    return process_manager.cancel_process(process_key)


@app.route("/", methods=["GET"])
def process_status():
    return process_manager.process_running_status()


if __name__ == "__main__":
    app.run(port=5000)
