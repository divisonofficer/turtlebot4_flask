import os
import json
import subprocess
import threading
import cv2
import numpy as np
from capture_type import CaptureSingleScene, ImageBytes
from time import time
from typing import Optional, Dict, Any
from capture_pb2 import CaptureAppCapture, CaptureAppScene, CaptureAppSpace
from google.protobuf import json_format
from google.protobuf.json_format import ParseError

CAPTURE_TEMP = "tmp/oakd_capture"
if not os.path.exists(CAPTURE_TEMP):
    os.mkdir(CAPTURE_TEMP)

DEVICE_DICT = {
    "_jai_1600_left": 2,
    "_jai_1600_right": 2,
}


class CaptureStorage:

    def __init__(self):
        self.image_jai_mask = cv2.imread("instance/capture/jai_binete.png")
        self.image_jai_mask_128 = cv2.resize(self.image_jai_mask, (128, 128))
        pass

    def store_space_metadata(
        self,
        space_id: int,
        space_name: Optional[str] = None,
        map_name: Optional[str] = None,
    ):
        if not os.path.exists(f"{CAPTURE_TEMP}/{space_id}"):
            os.mkdir(f"{CAPTURE_TEMP}/{space_id}")
        metadata = {}
        if os.path.exists(f"{CAPTURE_TEMP}/{space_id}/meta.json"):
            with open(f"{CAPTURE_TEMP}/{space_id}/meta.json", "r") as f:
                raw = f.read()
                metadata = json.loads(raw)
                if not space_name:
                    space_name = metadata["space_name"]
                if not map_name and "map_name" in metadata:
                    map_name = metadata["map_name"]
        else:
            metadata["create_time"] = time()
        metadata["space_id"] = space_id
        metadata["space_name"] = space_name
        metadata["update_time"] = time()
        metadata["map_name"] = map_name
        with open(f"{CAPTURE_TEMP}/{space_id}/meta.json", "w") as f:
            json.dump(metadata, f)

    def get_space_metadata(self, space_id: int):
        metadataFile = os.path.join(CAPTURE_TEMP, str(space_id), "meta.json")
        space = CaptureAppSpace()
        if not os.path.exists(metadataFile):
            space.space_id = space_id
        else:
            with open(metadataFile, "r") as f:
                raw = f.read()
                json_format.Parse(raw, space)

        if not space.map_name or not space.map_name.strip():
            space.map_name = str(space_id)

        space.captures.extend(self.get_space_all_captures(space_id))
        return space

    def get_all_spaces(self):
        spaces = os.listdir(CAPTURE_TEMP)

        metadataList: list[CaptureAppSpace] = []
        for space in spaces:
            if not os.path.isdir(os.path.join(CAPTURE_TEMP, space)):
                continue
            meta = self.get_space_metadata(int(space))

            metadataList.append(meta)

        return metadataList

    def get_capture_scene_meta(self, space_id: int, capture_id: int, scene_id: int):
        capture_dir = os.path.join(CAPTURE_TEMP, str(space_id), str(capture_id))
        scene_dir = os.path.join(capture_dir, str(scene_id))
        try:
            with open(os.path.join(scene_dir, "meta.json"), "r") as f:
                raw = f.read()
                scene = CaptureAppScene()
                scene.space_id = space_id
                scene.scene_id = scene_id
                try:
                    json_format.Parse(raw, scene)
                except ParseError:
                    scene_dict = json.loads(raw)
                    if not "range_max" in scene_dict["lidar_position"]:
                        max_range = max(
                            scene_dict["lidar_position"]["ranges"],
                            default=float("-inf"),
                            key=lambda x: x if x != float("inf") else float("-inf"),
                        )
                        scene_dict["lidar_position"]["range_max"] = max_range
                    scene_dict["lidar_position"]["ranges"] = [
                        (
                            x
                            if x != float("inf")
                            else scene_dict["lidar_position"]["range_max"]
                        )
                        for x in scene_dict["lidar_position"]["ranges"]
                    ]
                    if "robotPose" in scene_dict:
                        scene_dict["robot_pose"] = scene_dict["robotPose"]
                        del scene_dict["robotPose"]
                    eulerAngle: dict = scene_dict["robot_pose"]["orientation"]
                    if "roll" in eulerAngle:
                        yaw, pitch, roll = (
                            eulerAngle["yaw"],
                            eulerAngle["pitch"],
                            eulerAngle["roll"],
                        )
                        cy = np.cos(yaw * 0.5)
                        sy = np.sin(yaw * 0.5)
                        cp = np.cos(pitch * 0.5)
                        sp = np.sin(pitch * 0.5)
                        cr = np.cos(roll * 0.5)
                        sr = np.sin(roll * 0.5)

                        qw = cr * cp * cy + sr * sp * sy
                        qx = sr * cp * cy - cr * sp * sy
                        qy = cr * sp * cy + sr * cp * sy
                        qz = cr * cp * sy - sr * sp * cy
                        scene_dict["robot_pose"]["orientation"] = {
                            "x": qx,
                            "y": qy,
                            "z": qz,
                            "w": qw,
                        }

                    for key, value in scene_dict.items():
                        if hasattr(scene, key):
                            if isinstance(value, dict):
                                json_format.ParseDict(value, getattr(scene, key))
                            elif isinstance(value, list):
                                getattr(scene, key).extend(value)
                            else:
                                setattr(scene, key, value)
                scene.images[:] = self.get_capture_scene_images_paths(
                    space_id, capture_id, scene_id
                )

                return scene
        except FileNotFoundError:
            return None
        except json.JSONDecodeError:
            with open(os.path.join(scene_dir, "meta.json"), "r") as f:
                print(f"JSON Decoder Error {f.read()}")
            return None

    def get_capture_metadata(self, space_id: int, capture_id: int):
        # get number of folders in the capture directory
        capture_dir = os.path.join(CAPTURE_TEMP, str(space_id), str(capture_id))
        capture = CaptureAppCapture(space_id=space_id, capture_id=capture_id, scenes=[])
        scenes = []
        for scene_id in os.listdir(capture_dir):
            try:
                scene = self.get_capture_scene_meta(space_id, capture_id, int(scene_id))
                if scene:
                    scenes.append(scene)
            except FileNotFoundError:
                continue
        if len(scenes) == 0:
            return None
        scenes.sort(key=lambda x: x.scene_id)
        capture.scenes.extend(scenes)
        return capture

    def get_capture_scene_filepath(
        self, space_id: int, capture_id: int, scene_id: int, filename: str
    ):
        if not "." in filename:
            filename += ".png"
        return f"{CAPTURE_TEMP}/{space_id}/{capture_id}/{scene_id}/{filename}"

    def get_capture_scene_image_thumb(
        self, space_id: int, capture_id: int, scene_id: int, filename: str
    ):
        if not "." in filename:
            filename += ".png"

        if not os.path.exists(
            f"{CAPTURE_TEMP}/{space_id}/{capture_id}/{scene_id}/thumb"
        ):
            try:
                os.mkdir(f"{CAPTURE_TEMP}/{space_id}/{capture_id}/{scene_id}/thumb")
            except FileExistsError:
                pass
        img_path = self.get_capture_scene_filepath(
            space_id, capture_id, scene_id, filename
        )
        if not os.path.exists(img_path):
            return None

        img = cv2.imread(img_path)
        img = cv2.resize(img, (128, 128))

        # if "jai_1600_left" in filename:
        #     img = cv2.bitwise_and(img, self.image_jai_mask_128)
        #     img = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
        #     img[:, :, 3] = self.image_jai_mask_128[:, :, 0]
        img_encoded = cv2.imencode(".png", img)[1]

        bytes = img_encoded.tobytes()
        del img
        del img_encoded
        return bytes

    def get_capture_scene_images_paths(
        self, space_id: int, capture_id: int, scene_id: int
    ):
        scene_dir = f"{CAPTURE_TEMP}/{space_id}/{capture_id}/{scene_id}"
        if not os.path.exists(scene_dir):
            return []
        files = os.listdir(scene_dir)
        image_files = [f for f in files if f.endswith(".png") or f.endswith(".jpg")]

        image_files_with_path = [
            f for f in image_files if any(k in f for k in DEVICE_DICT.keys())
        ]
        image_files_other = [f for f in image_files if f not in image_files_with_path]
        image_files_dict: dict[str, list[str]] = {}
        for k, v in DEVICE_DICT.items():
            image_files_dict[k] = []
            for channel in range(v):
                imList = [
                    f
                    for f in image_files_with_path
                    if f.startswith(f"{k}_channel_{channel}")
                ]
                imList.sort(key=lambda x: int(x.split("_")[-1].split(".")[0]))
                image_files_dict[k].extend(imList)

        return [
            files for device in image_files_dict.values() for files in device
        ] + image_files_other

    def get_space_all_captures(self, space_id: int):
        capture_dir = f"{CAPTURE_TEMP}/{space_id}"
        if not os.path.exists(capture_dir):
            return []
        captures = os.listdir(capture_dir)
        capture_list = [
            x
            for x in [
                self.get_capture_metadata(space_id, int(c))
                for c in captures
                if c.isdigit()
            ]
            if x is not None
        ]
        capture_list.sort(key=lambda x: x.capture_id, reverse=True)
        return capture_list

    def get_space_all_scenes(self, space_id: int):
        capture_dir = os.path.join(CAPTURE_TEMP, str(space_id))
        captures = os.listdir(capture_dir)
        scenes: list[CaptureAppScene] = []
        for capture_id in captures:
            if not capture_id.isdigit():
                continue
            capture = self.get_capture_metadata(space_id, int(capture_id))
            if capture:
                scenes += capture.scenes[:]
        return scenes

    def store_captured_scene(
        self, space_id: int, capture_id: int, scene_id: int, scene: CaptureSingleScene
    ):
        space_dir = f"{CAPTURE_TEMP}/{space_id}"
        capture_dir = f"{CAPTURE_TEMP}/{space_id}/{capture_id}"
        scene_dir = f"{capture_dir}/{scene_id}"

        if capture_id != scene.capture_id or scene_id != scene.scene_id:
            capture_id = scene.capture_id
            scene_id = scene.scene_id
            print(
                f"scene_id and capture_id mismatched. Setting to {capture_id} and {scene_id}"
            )

        for d in [space_dir, capture_dir, scene_dir]:
            if not os.path.exists(d):
                os.mkdir(d)
        time_begin = time()

        threads: list[threading.Thread] = []
        for i, image in enumerate(scene.picture_list):
            filename = f"{scene_dir}/{image.topic.replace('/','_')}.png"
            threads.append(
                threading.Thread(
                    target=cv2.imwrite,
                    args=(filename, image.image),
                )
            )
            threads[-1].start()

        for t in threads:
            t.join()

        print(f"Time to write images: {time() - time_begin}")
        time_begin = time()
        scene_dict = scene.to_dict_light()
        # scene_dict.images[:] = self.get_capture_scene_images_paths(
        #     space_id, capture_id, scene_id
        # )

        print(f"Time to get image paths: {time() - time_begin}")
        with open(f"{scene_dir}/meta.json", "w") as f:
            f.write(json_format.MessageToJson(scene_dict))
        time_begin = time()
        del scene
        print(f"Time to delete scene: {time() - time_begin}")

    def store_captured_scene_image(
        self,
        image_list: list[ImageBytes],
        space_id: int,
        capture_id: int,
        scene_id: int,
    ):
        space_dir = f"{CAPTURE_TEMP}/{space_id}"
        capture_dir = f"{space_dir}/{capture_id}"
        scene_dir = f"{capture_dir}/{scene_id}"
        if not os.path.exists(capture_dir):
            os.mkdir(capture_dir)
        if not os.path.exists(scene_dir):
            os.makedirs(scene_dir)
        threads: list[threading.Thread] = []
        for image in image_list:
            filename = f"{scene_dir}/{image.topic.replace('/','_')}.png"
            cv2.imwrite(filename, image.image)
            threads.append(
                threading.Thread(
                    target=cv2.imwrite,
                    args=(filename, image.image),
                )
            )

        for t in threads:
            t.join()

        for image in image_list:
            del image

    def delete_scene(self, space_id: int, capture_id: int, scene_id: int):
        scene_dir = os.path.join(
            CAPTURE_TEMP, str(space_id), str(capture_id), str(scene_id)
        )
        if os.path.exists(scene_dir):
            for f in os.listdir(scene_dir):
                (
                    os.remove(os.path.join(scene_dir, f))
                    if os.path.isfile(os.path.join(scene_dir, f))
                    else os.rmdir(os.path.join(scene_dir, f))
                )
            os.rmdir(scene_dir)
        return 200

    def compress_space(self, space_id: int):
        space_dir = os.path.join(CAPTURE_TEMP, str(space_id))
        gz_file = f"{space_dir}.tar.gz"
        if os.path.exists(gz_file):
            os.remove(gz_file)
        subprocess.run(["tar", "-czvf", gz_file, space_dir])

    def get_compressed_space(self, space_id: int):
        space_dir = os.path.join(CAPTURE_TEMP, str(space_id))
        gz_file = f"{space_dir}.tar.gz"
        return f"/home/cglab/project/turtlebot4_flask/clientapp/{gz_file}"
