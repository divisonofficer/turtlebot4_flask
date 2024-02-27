from flask import Flask, request, jsonify, render_template
import threading
from flask_socketio import SocketIO
from controller import Controller
import json

import rclpy


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



def raw_callback(x):
    x['pixels'] = x['pixels'].tolist()
    socketio.emit('preview_image_response', {'image_data': json.dumps(x)})

def create_ros_subscriber():
    node = controller.con_subscribe_camera_preview(None, raw_callback)
    # Spin in a separate thread
    def spin():
        rclpy.spin(node)

    thread = threading.Thread(target=spin)
    thread.start()
    
    
if __name__ == '__main__':
    create_ros_subscriber()
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
    