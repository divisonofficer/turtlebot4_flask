from sensor_msgs.msg import (
    BatteryState,
    CameraInfo,
    Image,
    CompressedImage,
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
from rcl_interfaces.msg import ParameterEvent, Log
from lifecycle_msgs.msg import TransitionEvent
from tf2_msgs.msg import TFMessage
from theora_image_transport.msg import Packet
from turtlebot4_msgs.msg import UserButton, UserDisplay
from std_msgs.msg import String, Int16, Int32, Int64, Float32, Float64, Bool
from rclpy.qos import QoSPresetProfiles


service_type_map = {
    "rcl_interfaces/msg/Log": Log,
    "sensor_msgs/msg/BatteryState": BatteryState,
    "irobot_create_msgs/msg/AudioNoteVector": AudioNoteVector,
    "irobot_create_msgs/msg/LightringLeds": LightringLeds,
    "geometry_msgs/msg/Twist": Twist,
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/CompressedImage": CompressedImage,
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
    "std_msgs/msg/Int16": Int16,
    "std_msgs/msg/Int32": Int32,
    "std_msgs/msg/Int64": Int64,
    "std_msgs/msg/Float32": Float32,
    "std_msgs/msg/Float64": Float64,
}


service_type_qos_map = {
    "sensor_msgs/msg/BatteryState": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "irobot_create_msgs/msg/AudioNoteVector": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "irobot_create_msgs/msg/LightringLeds": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "geometry_msgs/msg/Twist": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "sensor_msgs/msg/CameraInfo": QoSPresetProfiles.SENSOR_DATA.value,
    "sensor_msgs/msg/Image": QoSPresetProfiles.SENSOR_DATA.value,
    "diagnostic_msgs/msg/DiagnosticArray": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "diagnostic_msgs/msg/DiagnosticStatus": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "irobot_create_msgs/msg/DockStatus": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "irobot_create_msgs/msg/HazardDetectionVector": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "turtlebot4_msgs/msg/UserButton": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "turtlebot4_msgs/msg/UserDisplay": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "sensor_msgs/msg/Imu": QoSPresetProfiles.SENSOR_DATA.value,
    "irobot_create_msgs/msg/InterfaceButtons": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "irobot_create_msgs/msg/IrIntensityVector": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "irobot_create_msgs/msg/IrOpcode": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "sensor_msgs/msg/JointState": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "sensor_msgs/msg/Joy": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "sensor_msgs/msg/JoyFeedbackArray": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "irobot_create_msgs/msg/KidnapStatus": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "lifecycle_msgs/msg/TransitionEvent": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "irobot_create_msgs/msg/Mouse": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "theora_image_transport/msg/Packet": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "nav_msgs/msg/Odometry": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "rcl_interfaces/msg/ParameterEvent": QoSPresetProfiles.PARAMETER_EVENTS.value,
    "sensor_msgs/msg/LaserScan": QoSPresetProfiles.SENSOR_DATA.value,
    "irobot_create_msgs/msg/SlipStatus": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "irobot_create_msgs/msg/StopStatus": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "tf2_msgs/msg/TFMessage": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "irobot_create_msgs/msg/WheelStatus": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "irobot_create_msgs/msg/WheelTicks": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "irobot_create_msgs/msg/WheelVels": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    # Service types, defaulting to reliable for critical operations
    "std_srvs/srv/Empty": QoSPresetProfiles.SERVICES_DEFAULT.value,
    "std_srvs/srv/SetBool": QoSPresetProfiles.SERVICES_DEFAULT.value,
    "std_srvs/srv/Trigger": QoSPresetProfiles.SERVICES_DEFAULT.value,
    "std_msgs/msg/String": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "std_msgs/msg/Int16": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "std_msgs/msg/Int32": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "std_msgs/msg/Int64": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "std_msgs/msg/Float32": QoSPresetProfiles.SYSTEM_DEFAULT.value,
    "std_msgs/msg/Float64": QoSPresetProfiles.SYSTEM_DEFAULT.value,
}
