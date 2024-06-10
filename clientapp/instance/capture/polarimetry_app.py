import sys

sys.path.append("../..")
sys.path.append("../../../public/proto/python")
sys.path.append("../public/proto/python")
from flask import Flask, request, Response
import json
import cv2

app = Flask(__name__)
import numpy as np
from polarization_compute import PolarizationCompute
from capture_storage import CaptureStorage

p_compute = PolarizationCompute()
capture_storage = CaptureStorage()


@app.route("/linear/<space>/<capture>/<scene>/<angle>/<qwpangle>")
def polarization_calibration_render(space, capture, scene, angle, qwpangle):

    images = capture_storage.get_capture_scene_images_paths(
        int(space), int(capture), int(scene)
    )
    images = [x for x in images if "jai_1600_left_" in x]
    # print(images)
    p_compute.update_qwp_angles(
        [
            (int(x.split("channel_0_")[1].split(".")[0]) + float(qwpangle))
            for x in images
            if "channel_0" in x
        ]
    )
    images = [f"tmp/oakd_capture/{space}/{capture}/{scene}/{x}" for x in images]
    p_compute.put_image_list((int(space), int(capture), int(scene)), images)

    p_compute.update_linear_matrix(np.pi / 180 * float(angle))
    p_compute.compute()
    return Response(response=p_compute.image_np_dict.keys())


@app.route("/view/<property>/<y>/<x>", methods=["GET"])
def get_polarization_calibration_pixel_color(property, y, x):
    image = p_compute.get_image_by_property(property)
    if image is None:
        return Response(status=404)
    return Response(
        response=json.dumps(
            p_compute.get_color_by_property(property, float(x), float(y))
        ),
    )


@app.route(
    "/view/<property>",
    methods=["GET"],
)
def get_polarization_calibration(property):
    image = p_compute.get_image_by_property(property)
    if image is None:
        return Response(status=404)

    result_image_bytes = cv2.imencode(".jpg", image)[1].tobytes()
    response = Response(result_image_bytes, mimetype="image/jpeg")
    return response


if __name__ == "__main__":
    app.run(port=5017)
