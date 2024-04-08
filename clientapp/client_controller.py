from share.controller import Controller
from nodes.diagnostic import DiagnosticNode
from nodes.lidar import LidarNode


class ClientController(Controller):
    def init(self, socketio, socket_namespace: str):
        super().init(socketio, socket_namespace)
        self.lidar = LidarNode(socketio)
        self.diagnostic = DiagnosticNode()
        self.executor.executor_thread(
            [self.rospy.simple_subscriber, self.lidar, self.diagnostic]
        )

        self.diagnostic.callback = lambda x: socketio.emit(
            "/turtlebot/diagnostic", x, namespace="/socket/ros"
        )

    def start_lidar_node(self):
        self.lidar.is_running = True

    def stop_lidar_node(self):
        self.lidar.is_running = False
