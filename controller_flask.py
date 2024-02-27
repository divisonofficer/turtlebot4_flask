from flask import Flask, request, jsonify
from controller import Controller
import json


app = Flask(__name__)
controller = Controller()

@app.route('/stop_motor', methods=['POST'])
def post_stop_motor():
    response = controller.run_command("stop_motor")
    return jsonify(response)


@app.route('/camera/preview_raw', methods=['GET'])
def get_camera_preview_raw():
    '''
    Create Socket Connection with Client
    '''
    
