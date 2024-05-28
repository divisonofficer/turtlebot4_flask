import sys

sys.path.append("../..")
sys.path.append("../../../public/proto/python")
sys.path.append("../public/proto/python")

import logging

logging.basicConfig(
    format="CaptureApp %(message)s",
)

from flask import Flask, request, send_file, Response
import rclpy
from capture_diagnostic import CaptureDiagnostic
from capture_ros import CaptureNode
from capture_storage import CaptureStorage
from flask_socketio import SocketIO
import cv2
import threading
from google.protobuf.json_format import MessageToJson, MessageToDict
import json
from polarization_compute import PolarizationCompute
import numpy as np


def messageToDict(message):
    if isinstance(message, list):
        return [messageToDict(m) for m in message]

    return json.loads(MessageToJson(message, True))


app = Flask(__name__)
socketIO = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

capture_node: CaptureNode
diag_node: CaptureDiagnostic
capture_storage = CaptureStorage()
p_compute = PolarizationCompute()


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
    space_id = request.json.get("space_id") if request.json else None
    space_name = request.json.get("space_name") if request.json else None
    use_slam = request.json.get("use_slam") if request.json else None
    result = capture_node.init_space(
        int(space_id) if space_id else None,
        space_name if not space_id else None,
        use_slam=use_slam,
    )

    if not result:
        return Response(status=500)

    if result["status"] == "error":
        return Response(status=400, response=f"{result}")

    return result


@app.route("/close", methods=["POST"])
def close_space():
    result = capture_node.empty_space()
    if not result:
        return Response(status=500)
    return Response(status=200, response=f"{result}")


@app.route("/")
def get_capture_status():
    return capture_node.get_status()


@app.route("/message_group/<group>/<cmd>", methods=["POST"])
def message_group_cmd(group, cmd):
    if cmd == "enable":
        capture_node.update_capture_topic(group, True)
    if cmd == "disable":
        capture_node.update_capture_topic(group, False)
    return capture_node.get_status()


@app.route("/capture", methods=["POST"])
def capture_at():
    result = capture_node.run_capture_queue_thread()
    if not result:
        return Response(status=500)
    if result["status"] == "error":
        return Response(status=400, response=f"{result}")
    return result


@app.route("/capture/single", methods=["POST"])
def capture_single():
    result = capture_node.run_capture_queue_single()
    if not result:
        return Response(status=500)
    if result["status"] == "error":
        return Response(status=400, response=f"{result}")
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
        "../../"
        + capture_storage.get_capture_scene_filepath(
            int(space_id), int(capture_id), int(scene_id), filename
        )
    )


@app.route(
    "/result/<space_id>/<capture_id>/<scene_id>/<filename>/thumb", methods=["GET"]
)
def capture_result_image_thumb(space_id, capture_id, scene_id, filename):
    if not space_id or not capture_id or not scene_id:
        return Response(status=404)

    return Response(
        status=200,
        response=capture_storage.get_capture_scene_image_thumb(
            int(space_id), int(capture_id), int(scene_id), filename
        ).tobytes(),  # type: ignore
        mimetype="image/jpeg",
    )


@app.route("/hyperparameters", methods=["GET"])
def get_scenario_hyperparameters():
    return MessageToDict(capture_node.scenario_hyper.to_msg())


@app.route("/hyperparameters", methods=["POST"])
def set_scenario_hyperparameters():
    name = request.json.get("name") if request.json else None
    value = request.json.get("value") if request.json else None
    capture_node.scenario_hyper.update(name, value)

    return MessageToDict(capture_node.scenario_hyper.to_msg())


@app.route("/polarization/qwpangle", methods=["POST"])
def update_qwp_angles():
    angles = request.json.get("angles") if request.json else None
    if angles and type(angles) == list:
        capture_node.messageDef.ANGLES = angles

    return {"status": "success"}


@app.route(
    "/polarization/linear/<space>/<capture>/<scene>/<channel>/<angle>", methods=["GET"]
)
def get_polarization_calibration(space, capture, scene, channel, angle):

    images = capture_storage.get_capture_scene_images_paths(
        int(space), int(capture), int(scene)
    )
    images = [x for x in images if "jai_1600_left_" + channel in x]
    print(images)
    p_compute.update_qwp_angles(
        [int(x.split(channel)[1].split(".")[0]) for x in images]
    )
    images = [f"tmp/oakd_capture/{space}/{capture}/{scene}/{x}" for x in images]
    p_compute.put_image_list(images)

    p_compute.update_linear_matrix(np.pi / 180 * float(angle))

    # Create a list of images
    images = p_compute.compute()

    # Create a new image by concatenating the four images horizontally
    images = [np.concatenate(x, axis=0) for x in images]
    result_image = np.concatenate(images, axis=1)

    # Convert the result image back to an ndarray
    result_array = np.array(result_image)
    result_image_bytes = cv2.imencode(".jpg", result_array)[1].tobytes()
    response = Response(result_image_bytes, mimetype="image/jpeg")
    return response


if __name__ == "__main__":
    socketIO.run(
        app,
        port=5012,
        allow_unsafe_werkzeug=True,
        host="0.0.0.0",
        use_reloader=False,
        log_output=True,
    )
