from flask import Flask, request, jsonify, render_template, Response
import threading
from flask_socketio import SocketIO
from controller import Controller
import json
import time
import rclpy
from cv_bridge import CvBridge
import cv2


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
    
    
output_frame = None
lock = threading.Lock()
def cv_raw_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    global output_frame
    output_frame = cv_image

def create_ros_subscriber():
    node = controller.con_subscribe_camera_preview(None, cv_raw_callback)
    # Spin in a separate thread
    def spin():
        rclpy.spin(node)

    thread = threading.Thread(target=spin)
    thread.start()

@app.route("/preview_video_feed")
def preview_video_feed():
    return Response(generate(),
                    mimetype = "multipart/x-mixed-replace; boundary=frame")

def generate():
    global output_frame, lock
    while True:
        with lock:
            if output_frame is None:
                continue
            (flag, encodedImage) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')

    
if __name__ == '__main__':
    create_ros_subscriber()
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
    