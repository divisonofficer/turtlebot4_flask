from flask import Flask, request, send_file
import rclpy
from capture_diagnostic import CaptureDiagnostic
import os
from capture_ros import CaptureNode, CAPTURE_TEMP
import json

app = Flask(__name__)


class NoLidarSignal(Exception):
    pass


class NoImageSignal(Exception):
    pass


class NoPoseSignal(Exception):
    pass


class CaptureStorage:

    def __init__(self):
        pass

    def get_capture_metadata(self, space_id: int, capture_id: int):
        # get number of folders in the capture directory
        capture_dir = os.path.join(CAPTURE_TEMP, str(space_id), str(capture_id))
        scenes = []
        for scene_id in os.listdir(capture_dir):
            scene_dir = os.path.join(capture_dir, scene_id)
            with open(os.path.join(scene_dir, "meta.json"), "r") as f:
                raw = f.read()
                scene = json.loads(raw)

                scene["capture_file"] = (
                    f"/result/{space_id}/{capture_id}/{scene_id}/oakd_mono.jpg"
                )
                scenes.append(scene)

        return scenes

    def get_capture_scene(
        self, space_id: int, capture_id: int, scene_id: int, filename: str
    ):
        return os.path.join(
            CAPTURE_TEMP, str(space_id), str(capture_id), str(scene_id), filename
        )


capture_node: CaptureNode
diag_node: CaptureDiagnostic
capture_storage = CaptureStorage()


with app.app_context():
    rclpy.init()
    diag_node = CaptureDiagnostic()
    capture_node = CaptureNode()


@app.route("/capture", methods=["POST"])
def capture_at():

    if capture_node.flag_capture_running:
        return {"status": "error", "message": "Capture is already running"}

    capture_topic = None
    if request.json is not None:
        capture_topic = request.json.get("capture_topic")
    if capture_topic:
        capture_node.image_topic = capture_topic
    else:
        capture_node.image_topic = "/oakd/rgb/preview/image_raw"

    return capture_node.run_capture_queue_thread()


@app.route("/capture/abort", methods=["POST"])
def capture_abort():
    if not capture_node.flag_capture_running:
        return {"status": "error", "message": "Capture is not running"}

    capture_node.flag_abort = True

    return {"status": "success", "message": "Capture is aborted"}


@app.route("/result/<space_id>/<capture_id>", methods=["GET"])
def capture_result(space_id, capture_id):
    return capture_storage.get_capture_metadata(int(space_id), int(capture_id))


@app.route("/result/<space_id>/<capture_id>/<scene_id>/<filename>", methods=["GET"])
def capture_result_image(space_id, capture_id, scene_id, filename):
    return send_file(
        capture_storage.get_capture_scene(
            int(space_id), int(capture_id), int(scene_id), filename
        )
    )


if __name__ == "__main__":
    app.run(port=5012)
