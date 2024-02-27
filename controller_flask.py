from flask import Flask, request, jsonify, render_template, Response
import threading
from flask_socketio import SocketIO
from controller import Controller
import json
import time
import rclpy
from cv_bridge import CvBridge
import cv2
from camera_function.preview_stream import generate_preview, create_ros_subscriber

app = Flask(__name__)
controller = Controller()
socketio = SocketIO(app)


@app.route('/stop_motor', methods=['POST'])
def post_stop_motor():
    response = controller.run_command("stop_motor")
    return jsonify(response)


@socketio.on('connect')
def handle_connect():
    print('Client connected')

@app.route('/resource/<resource>', methods=['GET'])
def get_resource(resource):
    '''
    return files in resource/ folder
    '''
    return app.send_static_file('resource/'+resource)

@app.route('/', methods=['GET'])
def index():
    return render_template('index.html')

frameTime = time.time()

def raw_callback(x):
    global frameTime
    '''
    sensor_msgs.msg.Image to Serealized Data
    '''
    beginTime = time.time()
    x = {
        'width': x.width,
        'height': x.height,
        'data' : x.data.tolist(),
    }
    endTime = time.time()
    
    timeTaken = endTime - beginTime
    frameTimeTaken = endTime - frameTime
    x['timeTaken'] = frameTimeTaken
    frameTime = endTime
    socketio.emit('preview_image_response', json.dumps(x))
    
@app.route("/preview_video_feed")
def preview_video_feed():
    return Response(generate_preview(),
                    mimetype = "multipart/x-mixed-replace; boundary=frame")


    
if __name__ == '__main__':
    create_ros_subscriber(controller)
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    