import base64
from dataclasses import dataclass, asdict

import sys

from flask_socketio import SocketIO
from sympy import Q
import torch
import torch.nn.functional as F

sys.path.append("instance/jai_bridge/modules/RAFT_Stereo")


from typing import Literal, Tuple
from rclpy.node import Node
from lucid.stereo_queue import StereoItemMerged
import numpy as np
from sensor_msgs.msg import CompressedImage
import cv2
import threading

from argparse import Namespace
import os

from ultralytics import YOLO
from matplotlib import cm


class IgevStereoWrapper:
    def __init__(self):
        self.build_model()

    def build_model(self):
        args = Namespace(
            restore_ckpt="instance/jai_bridge/modules/igevpp/chkpnt/latest_IGEVppFusion.pth",
            dataset="sceneflow",
            mixed_precision=False,
            precision_dtype="float32",
            valid_iters=32,
            hidden_dims=[128, 128, 128],
            corr_levels=2,
            corr_radius=4,
            n_downsample=2,
            n_gru_layers=3,
            max_disp=768,
            s_disp_range=48,
            m_disp_range=96,
            l_disp_range=192,
            s_disp_interval=1,
            m_disp_interval=2,
            l_disp_interval=4,
        )

        from modules.igevpp.core.igev_stereo_fusion import IGEVStereoFusion

        model = IGEVStereoFusion(args)
        ckpt = torch.load(args.restore_ckpt)["model_state_dict"]
        model.load_state_dict(ckpt, strict=True)
        model.eval()
        model.cuda()
        self.model = model

    def input_padding(self, img: torch.Tensor, divisor: int = 32):
        h, w = img.shape[-2:]
        pad_h = (divisor - h % divisor) % divisor
        pad_w = (divisor - w % divisor) % divisor
        if pad_h > 0 or pad_w > 0:
            img = torch.nn.functional.pad(img, (0, pad_w, 0, pad_h), mode="replicate")
        return img

    def infer(self, inputs: Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]):
        inputs = [self.input_padding(x, divisor=32) for x in inputs]
        with torch.no_grad():
            output = self.model(*inputs, test_mode=True)
        return output[0, 0].cpu().numpy()

    def get_attention(self, left: torch.Tensor, right: torch.Tensor):
        with torch.no_grad():
            left, right = [self.input_padding(x, divisor=32) for x in (left, right)]
            attention = self.model.feature(left, right, debug=True)
        attention = attention[0].permute(1, 2, 0).cpu().numpy().mean(axis=-1)
        return attention


class YoloWrapper:
    def __init__(self):
        self.build_model()

    def build_model(self):
        self.model = YOLO("yolo11n.pt")
        self.model.fuse()  # Fuse model layers for faster inference
        self.model.to("cuda")  # Move model to GPU

    def infer(self, img: np.ndarray):
        if isinstance(img, torch.Tensor):

            img = F.interpolate(
                img,
                size=(640, 640),
                mode="bilinear",
                align_corners=False,
            )

            img = img[0].permute(1, 2, 0).cpu().numpy()  # Convert to numpy array

            img = np.ascontiguousarray(img.astype(np.uint8))  # Convert to uint8
        else:
            img = cv2.resize(img, (640, 640))  # Resize to model input size
        results = self.model(img)
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            for box in boxes:
                x1, y1, x2, y2 = box[:4].astype(int)
                print(img.dtype, img.min(), img.max())
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        return img  # Return the image with bounding boxes drawn


class FusionWrapper:
    def __init__(self):
        self.build_hsv_model()

    def build_hsv_model(self):
        from modules.HSVNet import HSVNet

        hsvnet = HSVNet({})
        hsvnet.load_state_dict(
            torch.load("instance/jai_bridge/modules/latest_HSVFusionRes.pth")[
                "model_state_dict"
            ],
            strict=False,
        )
        self.hsvnet = hsvnet.eval().cuda()

    def infer_fusion(
        self, inputs: Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]
    ):
        with torch.no_grad():
            inputs = [x for x in inputs]
            output = self.hsvnet(inputs[:2], inputs[2:])
        return output[:1], output[1:]


class StereoUndistortion:

    def __init__(self, calibration_file: str):

        calibration = np.load(calibration_file)
        self.mtx_left = calibration["mtx_left"]
        self.mtx_right = calibration["mtx_right"]
        self.dist_left = calibration["dist_left"]
        self.dist_right = calibration["dist_right"]
        self.R = calibration["R"]
        self.T = calibration["T"]
        self.stereo_rectify = cv2.stereoRectify(
            self.mtx_left,
            self.dist_left,
            self.mtx_right,
            self.dist_right,
            (1440, 1080),  # Assuming a fixed resolution for stereo images
            self.R,
            self.T,
            alpha=0,
        )
        self.map_left_x, self.map_left_y = cv2.initUndistortRectifyMap(
            self.mtx_left,
            self.dist_left,
            self.stereo_rectify[0],
            self.stereo_rectify[2],
            (1440, 1080),  # Assuming a fixed resolution for stereo images
            cv2.CV_32FC1,
        )
        self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(
            self.mtx_right,
            self.dist_right,
            self.stereo_rectify[1],
            self.stereo_rectify[3],
            (1440, 1080),  # Assuming a fixed resolution for stereo images
            cv2.CV_32FC1,
        )

    def undistort(
        self, left_img: np.ndarray, right_img: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        left_img_rectified = cv2.remap(
            left_img, self.map_left_x, self.map_left_y, cv2.INTER_LINEAR
        )
        right_img_rectified = cv2.remap(
            right_img, self.map_right_x, self.map_right_y, cv2.INTER_LINEAR
        )
        return left_img_rectified, right_img_rectified


class JaiDemoNode(Node):
    @dataclass
    class Config:
        gamma: float = 1.8
        max_disp: int = 64
        depth_scale: float = 2
        fusion_scale: float = 2.0
        mode: Literal["RGB", "NIR", "IF", "FF"] = "RGB"

    def __init__(self, socket: SocketIO):
        super().__init__("jai_stereo_depth")  # type: ignore
        self.socket = socket
        self.config = self.Config()
        self.igev_wrapper = IgevStereoWrapper()
        self.fusion_wrapper = FusionWrapper()
        self.yolo_wrapper = YoloWrapper()
        """
        JAI Stereo (synchronized) topic subscription
        """
        try:
            self.stereo_merged_subscription = self.create_subscription(
                CompressedImage,
                "/jai_1600_stereo/merged",
                self.callback_sensor_stereo,
                10,
            )
            self.stereo_merged_subscription
        except Exception as e:
            print(e)

        self.undistortion = StereoUndistortion(
            "instance/jai_bridge/calibration_demo.npz"
        )

        self.flag_thumb_publish = threading.Event()
        self.flag_stereo_publish = threading.Event()
        self.flag_fusion_publish = threading.Event()

    def hdr_publish_log(self, log):
        self.socket.emit("hdr_log", log)

    def callback_sensor_stereo(self, msg: CompressedImage):
        if self.flag_thumb_publish.is_set():
            return

        thread = threading.Thread(
            target=self.process_stereo_msg, args=(msg,), daemon=True
        )
        thread.start()

    def process_stereo_msg(self, msg: CompressedImage):
        """
        Process the stereo message and publish the rectified images.
        This function is called in a separate thread to avoid blocking the main thread.
        """
        self.flag_thumb_publish.set()  # Set the flag to indicate that processing is ongoing
        stereo_rgb, stereo_nir = self.unpack_stereo_msg(msg)
        left_rectified, right_rectified = self.undistortion.undistort(
            stereo_rgb.left[0], stereo_rgb.right[0]
        )
        if self.config.gamma != 1.0:
            left_rectified = (
                cv2.pow(left_rectified / 255.0, 1 / self.config.gamma) * 255
            ).astype(np.uint8)
            right_rectified = (
                cv2.pow(right_rectified / 255.0, 1 / self.config.gamma) * 255
            ).astype(np.uint8)
        self.publish_img_socket(left_rectified, "stereo_rgb_left_rectified")
        self.publish_img_socket(right_rectified, "stereo_rgb_right_rectified")

        left_nir_rectified, right_nir_rectified = self.undistortion.undistort(
            stereo_nir.left[0], stereo_nir.right[0]
        )
        if self.config.gamma != 1.0:
            left_nir_rectified = (
                cv2.pow(left_nir_rectified / 255.0, 1 / self.config.gamma) * 255
            ).astype(np.uint8)
            right_nir_rectified = (
                cv2.pow(right_nir_rectified / 255.0, 1 / self.config.gamma) * 255
            ).astype(np.uint8)

        self.publish_img_socket(left_nir_rectified, "stereo_nir_left_rectified")
        self.publish_img_socket(right_nir_rectified, "stereo_nir_right_rectified")

        if (
            not self.flag_fusion_publish.is_set()
            or not self.flag_stereo_publish.is_set()
        ):
            left_rectified = (
                torch.from_numpy(left_rectified)
                .permute(2, 0, 1)
                .unsqueeze(0)
                .float()
                .cuda()
            )
            right_rectified = (
                torch.from_numpy(right_rectified)
                .permute(2, 0, 1)
                .unsqueeze(0)
                .float()
                .cuda()
            )
            left_nir_rectified = (
                torch.from_numpy(left_nir_rectified)
                .unsqueeze(0)
                .unsqueeze(0)
                .float()
                .cuda()
            )
            right_nir_rectified = (
                torch.from_numpy(right_nir_rectified)
                .unsqueeze(0)
                .unsqueeze(0)
                .float()
                .cuda()
            )

            if self.flag_fusion_publish.is_set():
                pass
                # print("Fusion processing is already ongoing, skipping this frame.")
            else:
                # Process stereo fusion in a separate thread
                if self.config.mode == "RGB":
                    thread = threading.Thread(
                        target=self.process_single_stereo,
                        args=(left_rectified, right_rectified),
                        daemon=True,
                    )
                elif self.config.mode == "NIR":
                    thread = threading.Thread(
                        target=self.process_single_stereo,
                        args=(
                            left_nir_rectified.repeat(1, 3, 1, 1),
                            right_nir_rectified.repeat(1, 3, 1, 1),
                        ),
                        daemon=True,
                    )
                elif self.config.mode == "IF":
                    thread = threading.Thread(
                        target=self.process_stereo_fusion,
                        args=(
                            (left_rectified, left_nir_rectified),
                            (right_rectified, right_nir_rectified),
                        ),
                        daemon=True,
                    )
                if self.config.mode != "FF":
                    thread.start()

            if self.flag_stereo_publish.is_set():
                pass
                # print("Stereo processing is already ongoing, skipping this frame.")
            else:
                if self.config.mode == "FF":
                    thread = threading.Thread(
                        target=self.process_stereo_depth,
                        args=(
                            [
                                left_rectified,
                                right_rectified,
                                left_nir_rectified,
                                right_nir_rectified,
                            ],
                        ),
                        daemon=True,
                    )
                    thread.start()

        self.flag_thumb_publish.clear()  # Clear the flag after processing is done

    def process_single_stereo(
        self,
        input_left: torch.Tensor,
        input_right: torch.Tensor,
    ):
        self.flag_fusion_publish.set()  # Set the flag to indicate that processing is ongoing
        if self.config.fusion_scale != 1.0:
            input_left, input_right = [
                F.interpolate(
                    x,
                    size=(
                        1080 // self.config.depth_scale,
                        1440 // self.config.depth_scale,
                    ),
                    mode="bilinear",
                    align_corners=False,
                )
                for x in (input_left, input_right)
            ]
        yolo_detect = self.yolo_wrapper.infer(input_left)
        self.publish_img_socket(yolo_detect, "yolo_detection")
        stereo_depth = self.igev_wrapper.infer(
            (input_left, input_right, input_left, input_right)
        )
        stereo_depth = self.apply_depth_colormap(stereo_depth)
        self.publish_img_socket(stereo_depth, "stereo_depth")
        self.flag_fusion_publish.clear()  # Clear the flag after processing is done

    def process_stereo_fusion(
        self,
        input_left: Tuple[np.ndarray, np.ndarray],
        input_right: Tuple[np.ndarray, np.ndarray],
    ):
        self.flag_fusion_publish.set()  # Set the flag to indicate that processing is ongoing

        if self.config.fusion_scale != 1.0:
            inputs = [
                F.interpolate(
                    x,
                    size=(
                        1080 // self.config.depth_scale,
                        1440 // self.config.depth_scale,
                    ),
                    mode="bilinear",
                    align_corners=False,
                )
                for x in input_left + input_right
            ]
            input_left, input_right = inputs[:2], inputs[2:]

        fusion_left, fusion_right = self.fusion_wrapper.infer_fusion(
            (input_left[0], input_right[0], input_left[1], input_right[1])
        )

        stereo_depth = self.igev_wrapper.infer(
            (fusion_left, fusion_right, fusion_left, fusion_right)
        )
        stereo_depth = self.apply_depth_colormap(stereo_depth)

        self.publish_img_socket(stereo_depth, "stereo_depth")

        fs_left_np = fusion_left[0].permute(1, 2, 0).cpu().numpy()
        fs_right_np = fusion_right[0].permute(1, 2, 0).cpu().numpy()
        self.publish_img_socket(fs_left_np, "stereo_fusion_left")
        self.publish_img_socket(fs_right_np, "stereo_fusion_right")

        yolo_detect = self.yolo_wrapper.infer(fs_left_np)
        self.publish_img_socket(yolo_detect, "yolo_detection")

        self.flag_fusion_publish.clear()  # Clear the flag after processing is done

    def apply_depth_colormap(self, depth: np.ndarray) -> np.ndarray:
        depth = cv2.resize(depth, (480, 360))  # Resize to a standard size
        depth = np.clip(depth, 0, self.config.max_disp) / self.config.max_disp * 255.0
        depth = depth.astype(np.uint8)
        depth = cv2.applyColorMap(depth, cv2.COLORMAP_MAGMA)
        return depth

    def process_stereo_depth(
        self, inputs: Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]
    ):
        self.flag_stereo_publish.set()  # Set the flag to indicate that processing is ongoing
        if self.config.depth_scale != 1.0:
            inputs = [
                F.interpolate(
                    x,
                    size=(
                        1080 // self.config.depth_scale,
                        1440 // self.config.depth_scale,
                    ),
                    mode="bilinear",
                    align_corners=False,
                )
                for x in inputs
            ]
        depth = self.igev_wrapper.infer(inputs)
        depth = self.apply_depth_colormap(depth)
        self.publish_img_socket(depth, "stereo_depth")

        attention = self.igev_wrapper.get_attention(inputs[0], inputs[2])
        attention = cv2.resize(attention, (480, 360))  # Resize to a standard size
        # bwr 컬러맵을 numpy 배열로 변환
        colormap = cm.get_cmap("bwr")  # matplotlib colormap
        colored_attention = colormap(attention)[:, :, :3]  # RGBA 중 RGB만 추출

        # float32 (0~1) → uint8 (0~255)
        colored_attention = (colored_attention * 255).astype(np.uint8)
        self.publish_img_socket(colored_attention, "stereo_attention")

        self.flag_stereo_publish.clear()  # Clear the flag after processing is done

    def publish_img_socket(self, img: np.ndarray, topic_name: str):
        if img.dtype == np.float32 or img.dtype == np.float64:
            if img.max() < 2.0:
                img = img * 255
            img = np.clip(img, 0, 255).astype(np.uint8)
        img = cv2.resize(img, (480, 360))  # Resize to a standard size
        _, buffer = cv2.imencode(".jpg", img)
        encoded = base64.b64encode(buffer).decode("utf-8")
        print(f"Publishing {topic_name} with shape {img.shape} and dtype {img.dtype}")
        self.socket.emit(
            "demo/img/" + topic_name,
            {
                "data": encoded,
                "shape": img.shape,
                "dtype": str(img.dtype),
            },
        )

    def compute_dimension(self, buffer_size: int):
        if buffer_size == 1440 * 1080:
            return 1440, 1080
        else:
            return 720, 540

    def unpack_stereo_msg(self, msg: CompressedImage) -> Tuple[
        StereoItemMerged[Tuple[np.ndarray, float], Tuple[np.ndarray, float]],
        StereoItemMerged[Tuple[np.ndarray, float], Tuple[np.ndarray, float]],
    ]:

        buffer_np = np.frombuffer(msg.data, np.uint8)
        width, height = self.compute_dimension(buffer_np.shape[0] / 8)
        buffer_np = buffer_np.reshape(8, height, width)
        exposure_times = [
            float(x) for x in msg.header.frame_id.split("stereo_")[-1].split("_")
        ]

        return (
            StereoItemMerged(
                (buffer_np[0:3].reshape(height, width, 3), exposure_times[0]),
                (buffer_np[3:6].reshape(height, width, 3), exposure_times[2]),
                msg.header,
            ),
            StereoItemMerged(
                (buffer_np[6], exposure_times[1]),
                (buffer_np[7], exposure_times[3]),
                msg.header,
            ),
        )

    def node_status(self):
        # todo : framerate, queue status, stored_count, left-right sync...
        status_dict = {"config": asdict(self.config)}
        self.socket.emit("stereo_status", status_dict)
        return status_dict
