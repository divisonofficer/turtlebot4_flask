from rclpy.node import Node
from rcl_interfaces.msg import Log
import sqlite3


class RosOutNode(Node):
    def __init__(self):
        super().__init__("client_rosout_node")
        self.subscription = self.create_subscription(
            Log, "/rosout", self.rosout_callback, 10
        )
        self.subscription

    def rosout_callback(self, msg: Log):
        print(msg.msg)


db = sqlite3.connect("rosout.db")


from flask import Flask


app = Flask(__name__)


if __name__ == "__main__":
    import rclpy

    rclpy.init()
    node = RosOutNode()
    app.run(port=5011)
