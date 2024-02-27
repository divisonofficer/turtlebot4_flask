from ros_call import call_ros2_service, subscribe_topic
from logger import log
from rclpy.qos import qos_profile_sensor_data
def ros_lidar_stop_motor():
    '''
    ros2 service call /stop_motor std_srvs/srv/Empty {}
    '''
    response = call_ros2_service('/stop_motor', 'std_srvs/srv/Empty')
    log('info', f"Response: {response}")
    return response

def ros_camera_preview_raw(callback):
    '''
    ros2 topic echo /oakd/rgb/preview/image_raw sensor_msgs/msg/Image
    '''
    response = subscribe_topic("/oakd/rgb/preview/image_raw", "sensor_msgs/msg/Image", callback)
    log('info', f"Response: {response}")
    return response