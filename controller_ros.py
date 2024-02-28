from ros_call import call_ros2_service, subscribe_topic
from logger import log
import numpy as np

from sensor_msgs.msg import CameraInfo


def ros_lidar_stop_motor():
    """
    ros2 service call /stop_motor std_srvs/srv/Empty {}
    """
    response = call_ros2_service("/stop_motor", "std_srvs/srv/Empty")
    log("info", f"Response: {response}")
    return response


def ros_camera_preview_raw(callback):
    """
    ros2 topic echo /oakd/rgb/preview/image_raw sensor_msgs/msg/Image
    """
    response = subscribe_topic(
        "/oakd/rgb/preview/image_raw", "sensor_msgs/msg/Image", callback
    )
    log("info", f"Response: {response}")
    return response


def ros_camera_info(callback):
    """
    ros2 topic echo /oakd/rgb/camera_info sensor_msgs/msg/CameraInfo
    """

    def toDict(info: CameraInfo):
        return {
            "height": info.height,
            "width": info.width,
            "distortion_model": info.distortion_model,
            "d": list(info.d),
            "k": info.k.tolist() if hasattr(info.k, "tolist") else list(info.k),
            "r": info.r.tolist() if hasattr(info.r, "tolist") else list(info.r),
            "p": info.p.tolist() if hasattr(info.p, "tolist") else list(info.p),
            "binning_x": info.binning_x,
            "binning_y": info.binning_y,
            # "roi": info.roi.to_dict() if hasattr(info.roi, "to_dict") else info.roi,
        }

    response = subscribe_topic(
        "/oakd/rgb/preview/camera_info",
        CameraInfo,
        lambda info: callback(toDict(info)),
    )
    log("info", f"Response: {response}")
    return response
