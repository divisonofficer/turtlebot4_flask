from flask import Flask, Response, request, send_from_directory
from flask_cors import CORS
from elliptecRotationStage import elliptecMount

app = Flask(__name__)
CORS(app)

import logging

logging.basicConfig(
    format="Ell14App %(message)s",
)


@app.route("/angle", methods=["POST"])
def set_device_angle():
    angle = int(request.json["angle"]) % 360 if request.json else None

    elliptecMount.move_by(angle)
    return Response(status=200)


@app.route("/angle/home", methods=["POST"])
def initiate_angle_home():
    elliptecMount.home()
    return Response(status=200)


if __name__ == "__main__":
    app.run(port=5016)
