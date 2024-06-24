"""
!!! For ROS2 see ros2/ folder for a more complete example !!!

Runs spectacularAI mapping and publishes poses and frames in ROS.

Make sure to have your ROS environment sourced before running this script. Tested with ROS noetic.

The SpectacularAI SDK and other dependencies can for example be installed in a virtual environment.
"""

import threading
from typing import Any, Callable, Optional
import spectacularAI
import depthai
from slam_depth_convert import depth_to_laser_scan
import rclpy
import numpy as np
import rclpy.clock
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from scipy.spatial.transform import Rotation
from slam_pb2 import Pose3D
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, LaserScan


def to_tf_message(camToWorld, ts, frame_id):
    msg = TFMessage()
    msg.transforms = []
    transform = TransformStamped()
    transform.header.stamp = ts
    transform.header.frame_id = "world"
    transform.child_frame_id = frame_id
    transform.transform.translation.x = camToWorld[0, 3]
    transform.transform.translation.y = camToWorld[1, 3]
    transform.transform.translation.z = camToWorld[2, 3]
    R_CW = Rotation.from_matrix(camToWorld[0:3, 0:3])
    q_cw = R_CW.as_quat()
    transform.transform.rotation.x = q_cw[0]
    transform.transform.rotation.y = q_cw[1]
    transform.transform.rotation.z = q_cw[2]
    transform.transform.rotation.w = q_cw[3]
    msg.transforms.append(transform)
    return msg


def to_camera_info_message(camera, frame, ts):
    intrinsic = camera.getIntrinsicMatrix()
    msg = CameraInfo()
    msg.header.stamp = ts
    msg.header.frame_id = "rgb_optical"
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.distortion_model = "none"
    msg.d = []
    msg.k = intrinsic.ravel().tolist()
    return msg


class SLAMNode(Node):

    def __init__(self):
        super().__init__("spectacular_slam_node")  # type: ignore
        self.odometry_publisher = self.create_publisher(
            PoseStamped, "/slam/odometry", 10
        )
        self.keyframe_publisher = self.create_publisher(
            PoseStamped, "/slam/keyframe", 10
        )
        self.rgb_publisher = self.create_publisher(Image, "/slam/rgb", 10)
        self.tf_publisher = self.create_publisher(TFMessage, "/tf", 10)
        self.point_publisher = self.create_publisher(
            PointCloud2, "/slam/pointcloud", 10
        )
        self.depth_publisher = self.create_publisher(Image, "/slam/depth", 10)
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, "/slam/camera_info", 10
        )
        self.scan_publisher = self.create_publisher(LaserScan, "/scan", 10)
        self.bridge = CvBridge()
        self.keyframes = {}

        self.create_timer(10.0, self.remind_msg)
        self.base_pose: Optional[Pose3D] = None

    msg_depth = None
    msg_odometry = None
    msg_pointcloud = None
    on_msg_depth = None
    on_msg_odometry: Optional[Callable] = None
    on_msg_pointcloud: Optional[Callable] = None
    on_msg_tf: Optional[Callable] = None
    on_msg_keyframe: Optional[Callable] = None

    def remind_msg(self):
        if self.msg_depth is not None:
            self.depth_publisher.publish(self.msg_depth)
        if self.msg_odometry is not None:
            self.odometry_publisher.publish(self.msg_odometry)
        if self.msg_pointcloud is not None:
            self.point_publisher.publish(self.msg_pointcloud)

    def to_pose_message(self, camToWorld):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = camToWorld[0, 3]
        msg.pose.position.y = camToWorld[1, 3]
        msg.pose.position.z = camToWorld[2, 3]
        R_CW = Rotation.from_matrix(camToWorld[0:3, 0:3])
        q_cw = R_CW.as_quat()
        msg.pose.orientation.x = q_cw[0]
        msg.pose.orientation.y = q_cw[1]
        msg.pose.orientation.z = q_cw[2]
        msg.pose.orientation.w = q_cw[3]
        return msg

    def has_keyframe(self, frame_id):
        return frame_id in self.keyframes

    def newKeyFrame(self, frame_id, keyframe):
        now = self.get_clock().now().to_msg()
        self.keyframes[frame_id] = True
        camToWorld = keyframe.frameSet.rgbFrame.cameraPose.getCameraToWorldMatrix()
        sequence_number = int(frame_id)
        msg = self.to_pose_message(camToWorld)
        msg.header.stamp = now
        self.keyframe_publisher.publish(msg)
        if self.on_msg_keyframe:
            self.on_msg_keyframe(msg)

        rgb_frame = keyframe.frameSet.getUndistortedFrame(
            keyframe.frameSet.rgbFrame
        ).image
        rgb_frame = rgb_frame.toArray()
        rgb_message = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="rgb8")
        rgb_message.header.stamp = now
        rgb_message.header.frame_id = "rgb_optical"
        # rgb_message.header.seq = sequence_number
        self.rgb_publisher.publish(rgb_message)
        # tf_message = to_tf_message(camToWorld, now, "world")
        # self.tf_publisher.publish(tf_message)
        # if self.on_msg_tf:
        #     self.on_msg_tf(tf_message)

        camera = keyframe.frameSet.rgbFrame.cameraPose.camera
        info_msg = to_camera_info_message(camera, rgb_frame, now)
        self.camera_info_publisher.publish(info_msg)

        self.newPointCloud(keyframe)

        depth_frame = keyframe.frameSet.getAlignedDepthFrame(keyframe.frameSet.rgbFrame)
        depth = depth_frame.image.toArray()
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="mono16")
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = "rgb_optical"
        # depth_msg.header.seq = sequence_number
        self.depth_publisher.publish(depth_msg)

        scan, points = depth_to_laser_scan(info_msg, depth)
        self.scan_publisher.publish(scan)

        self.msg_depth = depth_msg
        if self.on_msg_depth:
            self.on_msg_depth(depth_msg)

    def newOdometryFrame(self, camToWorld):
        msg = self.to_pose_message(camToWorld)
        self.odometry_publisher.publish(msg)
        self.msg_odometry = msg
        if self.on_msg_odometry:
            self.on_msg_odometry(msg)

    def newPointCloud(self, keyframe):
        camToWorld = keyframe.frameSet.rgbFrame.cameraPose.getCameraToWorldMatrix()
        positions = keyframe.pointCloud.getPositionData()
        pc = np.zeros((positions.shape[0], 6), dtype=np.float32)
        p_C = np.vstack((positions.T, np.ones((1, positions.shape[0])))).T
        pc[:, :3] = (camToWorld @ p_C[:, :, None])[:, :3, 0]

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        if keyframe.pointCloud.hasColors():
            pc[:, 3:] = keyframe.pointCloud.getRGB24Data() * (1.0 / 255.0)
        msg.point_step = 4 * 6
        msg.height = 1
        msg.width = pc.shape[0]
        msg.row_step = msg.point_step * pc.shape[0]
        msg.data = pc.tobytes()
        msg.is_bigendian = False
        msg.is_dense = False
        ros_dtype = PointField.FLOAT32
        itemsize = np.dtype(np.float32).itemsize
        msg.fields = [
            PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate("xyzrgb")
        ]
        self.point_publisher.publish(msg)

        self.msg_pointcloud = msg
        if self.on_msg_pointcloud:
            self.on_msg_pointcloud(msg)


def parseArgs():
    import argparse

    p = argparse.ArgumentParser(__doc__)
    p.add_argument(
        "--ir_dot_brightness",
        help="OAK-D Pro (W) IR laser projector brightness (mA), 0 - 1200",
        type=float,
        default=0,
    )
    p.add_argument(
        "--useRectification",
        help="--dataFolder option can also be used with some non-OAK-D recordings, but this parameter must be set if the videos inputs are not rectified.",
        action="store_true",
    )
    p.add_argument("--map", help="Map file to load", default=None)
    p.add_argument("--save-map", help="Map file to save", default=None)
    p.add_argument("--color", help="Use RGB camera for tracking", action="store_true")
    return p.parse_args()


def node_main(slam_node: SLAMNode, configInternal: dict[str, Any] = {}):

    def onVioOutput(vioOutput):
        cameraPose = vioOutput.getCameraPose(0)
        camToWorld = cameraPose.getCameraToWorldMatrix()
        slam_node.newOdometryFrame(camToWorld)

    def onMappingOutput(output):
        for frame_id in output.updatedKeyFrames:
            keyFrame = output.map.keyFrames.get(frame_id)

            # Remove deleted key frames from visualisation
            if not keyFrame:
                continue

            # Check that point cloud exists
            if not keyFrame.pointCloud:
                continue

            if not slam_node.has_keyframe(frame_id):
                slam_node.newKeyFrame(frame_id, keyFrame)

        if output.finalMap:
            print("Final map ready!")

    print("Starting OAK-D device")
    pipeline = depthai.Pipeline()
    config = spectacularAI.depthai.Configuration()  # type: ignore
    config.internalParameters = configInternal
    config.useSlam = True
    config.ensureSufficientUsbSpeed = False
    config.useStereo = True
    config.fastImu = True

    # if args.color:
    #     config.useColor = True
    # if args.map is not None:
    #     config.mapLoadPath = args.map
    # if args.save_map is not None:
    #     config.mapSavePath = args.save_map
    vioPipeline = spectacularAI.depthai.Pipeline(pipeline, config, onMappingOutput)  # type: ignore

    with depthai.Device(pipeline) as device, vioPipeline.startSession(
        device
    ) as vio_session:
        # if args.ir_dot_brightness > 0:
        device.setIrLaserDotProjectorBrightness(100)
        while rclpy.ok():
            onVioOutput(vio_session.waitForOutput())


def node_main_thread():
    node = SLAMNode()
    threading.Thread(target=node_main, args=(node,)).start()

    return node


if __name__ == "__main__":

    args = parseArgs()

    configInternal = {
        "computeStereoPointCloud": "true",
        # "pointCloudNormalsEnabled": "true",
        "computeDenseStereoDepth": "true",
    }
    if args.useRectification:
        configInternal["useRectification"] = "true"
    rclpy.init()
    slam_node = SLAMNode()
    node_main(slam_node, configInternal)
