import sys

sys.path.append("../..")
sys.path.append("../../../public/proto/python")
sys.path.append("../public/proto/python")

from flask import Flask, request, send_file, Response
import rclpy
from capture_diagnostic import CaptureDiagnostic
from capture_ros import CaptureNode
from capture_storage import CaptureStorage
from flask_socketio import SocketIO
import cv2
import threading
from google.protobuf.json_format import MessageToJson
import json


def messageToDict(message):
    if isinstance(message, list):
        return [messageToDict(m) for m in message]

    return json.loads(MessageToJson(message, True))


app = Flask(__name__)
socketIO = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

capture_node: CaptureNode
diag_node: CaptureDiagnostic
capture_storage = CaptureStorage()


with app.app_context():
    rclpy.init()
    diag_node = CaptureDiagnostic()
    capture_node = CaptureNode(capture_storage, socketIO)

    def spin():
        rclpy.spin(capture_node)

    threading.Thread(target=spin, daemon=True).start()


@socketIO.on("connect", namespace="/socket")
def connectSocket():
    print("Connected to socket")


@app.route("/init", methods=["POST"])
def init_space():
    space_id = request.json.get("space_id") if request and request.json else None
    space_name = request.json.get("space_name") if request and request.json else None
    result = capture_node.init_space(
        int(space_id) if space_id else None, space_name if not space_id else None
    )

    if not result:
        return Response(status=500)

    if result["status"] == "error":
        return Response(status=400, response=result)

    return result


@app.route("/close", methods=["POST"])
def close_space():
    result = capture_node.empty_space()
    if not result:
        return Response(status=500)
    return result


@app.route("/")
def get_capture_status():
    return {
        "space_ready": capture_node.space_id is not None,
        "space_id": capture_node.space_id,
    }


@app.route("/capture", methods=["POST"])
def capture_at():
    capture_topic = None
    if request.json is not None:
        capture_topic = request.json.get("capture_topic")
    if capture_topic:
        capture_node.image_topics = capture_topic
    else:
        capture_node.image_topics = ["/oakd/rgb/preview/image_raw"]

    result = capture_node.run_capture_queue_thread()
    if not result:
        return Response(status=500)
    if result["status"] == "error":
        return Response(status=400, response=result)
    return result


@app.route("/capture/single", methods=["POST"])
def capture_single():
    capture_topic = None
    if request.json is not None:
        capture_topic = request.json.get("topics")
    if capture_topic:
        capture_node.image_topics = capture_topic
    else:
        capture_node.image_topics = ["/oakd/rgb/preview/image_raw"]
    print(capture_node.image_topics)
    result = capture_node.run_capture_queue_single()
    if not result:
        return Response(status=500)
    if result["status"] == "error":
        return Response(status=400, response=result)
    return result


@app.route("/capture/abort", methods=["POST"])
def capture_abort():
    if not capture_node.flag_capture_running:
        return {"status": "error", "message": "Capture is not running"}

    capture_node.flag_abort = True

    return {"status": "success", "message": "Capture is aborted"}


@app.route("/result", methods=["GET"])
def capture_result_space_list():
    return messageToDict(capture_storage.get_all_spaces())


@app.route("/result/<space_id>")
def capture_result_list(space_id):
    return messageToDict(capture_storage.get_space_metadata(int(space_id)))


@app.route("/result/<space_id>/captures")
def capture_result_all_captures(space_id):
    return messageToDict(capture_storage.get_space_all_captures(int(space_id)))


@app.route("/result/<space_id>/scenes")
def capture_result_all_scenes(space_id):
    return messageToDict(capture_storage.get_space_all_scenes(int(space_id)))


@app.route("/result/<space_id>/<capture_id>", methods=["GET"])
def capture_result(space_id, capture_id):
    return messageToDict(
        capture_storage.get_capture_metadata(int(space_id), int(capture_id))
    )


@app.route("/result/<space_id>/<capture_id>/<scene_id>", methods=["GET"])
def capture_result_scene(space_id, capture_id, scene_id):
    return messageToDict(
        capture_storage.get_capture_scene_meta(
            int(space_id), int(capture_id), int(scene_id)
        )
    )


@app.route("/result/<space_id>/<capture_id>/<scene_id>/images", methods=["GET"])
def capture_result_scene_images(space_id, capture_id, scene_id):
    return capture_storage.get_capture_scene_images_paths(
        space_id, capture_id, scene_id
    )


@app.route("/result/<space_id>/<capture_id>/<scene_id>/<filename>", methods=["GET"])
def capture_result_image(space_id, capture_id, scene_id, filename):
    return send_file(
        capture_storage.get_capture_scene_filepath(
            int(space_id), int(capture_id), int(scene_id), filename
        )
    )


@app.route(
    "/result/<space_id>/<capture_id>/<scene_id>/<filename>/thumb", methods=["GET"]
)
def capture_result_image_thumb(space_id, capture_id, scene_id, filename):
    img = cv2.imread(
        capture_storage.get_capture_scene_filepath(
            int(space_id), int(capture_id), int(scene_id), filename
        )
    )
    img = cv2.resize(img, (128, 128))
    _, img_encoded = cv2.imencode(".png", img)
    return Response(img_encoded.tobytes(), mimetype="image/jpeg")


if __name__ == "__main__":
    socketIO.run(app, port=5012, allow_unsafe_werkzeug=True, host="0.0.0.0")
