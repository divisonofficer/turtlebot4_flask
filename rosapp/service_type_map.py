from sensor_msgs.msg import (
    BatteryState,
    CameraInfo,
    Image,
    Imu,
    JointState,
    Joy,
    JoyFeedbackArray,
    LaserScan,
)
from std_srvs.srv import Empty, SetBool, Trigger
from irobot_create_msgs.msg import (
    AudioNoteVector,
    LightringLeds,
    DockStatus,
    HazardDetectionVector,
    InterfaceButtons,
    IrIntensityVector,
    IrOpcode,
    KidnapStatus,
    Mouse,
    SlipStatus,
    StopStatus,
    WheelStatus,
    WheelTicks,
    WheelVels,
)
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterEvent
from lifecycle_msgs.msg import TransitionEvent
from tf2_msgs.msg import TFMessage
from theora_image_transport.msg import Packet
from turtlebot4_msgs.msg import UserButton, UserDisplay
from std_msgs.msg import String


service_type_map = {
    "sensor/message/BatteryState": BatteryState,
    "irobot_create_msgs/msg/AudioNoteVector": AudioNoteVector,
    "irobot_create_msgs/msg/LightringLeds": LightringLeds,
    "geometry_msgs/msg/Twist": Twist,
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/Image": Image,
    "diagnostic_msgs/msg/DiagnosticArray": DiagnosticArray,
    "diagnostic_msgs/msg/DiagnosticStatus": DiagnosticStatus,
    "irobot_create_msgs/msg/DockStatus": DockStatus,
    "irobot_create_msgs/msg/HazardDetectionVector": HazardDetectionVector,
    "turtlebot4_msgs/msg/UserButton": UserButton,
    "turtlebot4_msgs/msg/UserDisplay": UserDisplay,
    "sensor_msgs/msg/Imu": Imu,
    "irobot_create_msgs/msg/InterfaceButtons": InterfaceButtons,
    "irobot_create_msgs/msg/IrIntensityVector": IrIntensityVector,
    "irobot_create_msgs/msg/IrOpcode": IrOpcode,
    "sensor_msgs/msg/JointState": JointState,
    "sensor_msgs/msg/Joy": Joy,
    "sensor_msgs/msg/JoyFeedbackArray": JoyFeedbackArray,
    "irobot_create_msgs/msg/KidnapStatus": KidnapStatus,
    "lifecycle_msgs/msg/TransitionEvent": TransitionEvent,
    "irobot_create_msgs/msg/Mouse": Mouse,
    "theora_image_transport/msg/Packet": Packet,
    "nav_msgs/msg/Odometry": Odometry,
    "rcl_interfaces/msg/ParameterEvent": ParameterEvent,
    "sensor_msgs/msg/LaserScan": LaserScan,
    "irobot_create_msgs/msg/SlipStatus": SlipStatus,
    "irobot_create_msgs/msg/StopStatus": StopStatus,
    "tf2_msgs/msg/TFMessage": TFMessage,
    "irobot_create_msgs/msg/WheelStatus": WheelStatus,
    "irobot_create_msgs/msg/WheelTicks": WheelTicks,
    "irobot_create_msgs/msg/WheelVels": WheelVels,
    "std_srvs/srv/Empty": Empty,
    "std_srvs/srv/SetBool": SetBool,
    "std_srvs/srv/Trigger": Trigger,
    "std_msgs/msg/String": String,
}
