from flask import Flask, request, send_file, Response
import rclpy
from capture_diagnostic import CaptureDiagnostic
from capture_ros import CaptureNode
from capture_storage import CaptureStorage

app = Flask(__name__)


capture_node: CaptureNode
diag_node: CaptureDiagnostic
capture_storage = CaptureStorage()


with app.app_context():
    rclpy.init()
    diag_node = CaptureDiagnostic()
    capture_node = CaptureNode(capture_storage)


@app.route("/capture", methods=["POST"])
def capture_at():

    if capture_node.flag_capture_running:
        return {"status": "error", "message": "Capture is already running"}

    capture_topic = None
    if request.json is not None:
        capture_topic = request.json.get("capture_topic")
    if capture_topic:
        capture_node.image_topics = capture_topic
    else:
        capture_node.image_topics = ["/oakd/rgb/preview/image_raw"]

    return capture_node.run_capture_queue_thread()


@app.route("/capture/single", methods=["POST"])
def capture_single():
    if capture_node.flag_capture_running:
        return {"status": "error", "message": "Capture is already running"}

    capture_topic = None
    if request.json is not None:
        capture_topic = request.json.get("capture_topic")
    if capture_topic:
        capture_node.image_topics = capture_topic
    else:
        capture_node.image_topics = ["/oakd/rgb/preview/image_raw"]
    result = capture_node.run_capture_queue_single()
    if not result:
        return Response(status=500)
    return result


@app.route("/capture/abort", methods=["POST"])
def capture_abort():
    if not capture_node.flag_capture_running:
        return {"status": "error", "message": "Capture is not running"}

    capture_node.flag_abort = True

    return {"status": "success", "message": "Capture is aborted"}


@app.route("/result/<space_id>")
def capture_result_list(space_id):
    return capture_storage.get_space_metadata(int(space_id))


@app.route("/result/<space_id>/scenes")
def capture_result_all_scenes(space_id):
    return capture_storage.get_space_all_scenes(int(space_id))


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
