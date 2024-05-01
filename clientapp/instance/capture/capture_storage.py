import os
import json
import cv2
from capture_type import CaptureSingleScene
from time import time
from typing import Optional, Dict, Any

CAPTURE_TEMP = "/tmp/oakd_capture"
if not os.path.exists(CAPTURE_TEMP):
    os.mkdir(CAPTURE_TEMP)


class CaptureStorage:

    def __init__(self):
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
        meta: Dict[str, Any] = {}
        if not os.path.exists(metadataFile):
            meta["space_id"] = space_id
        else:
            with open(metadataFile, "r") as f:
                raw = f.read()
                meta = json.loads(raw)
        if not "map_name" in meta:
            meta["map_name"] = str(space_id)
        capture_dir = os.path.join(CAPTURE_TEMP, str(space_id))
        meta["captures"] = self.get_space_all_captures(space_id)
        return meta

    def get_all_spaces(self):
        spaces = os.listdir(CAPTURE_TEMP)

        metadataList = []
        for space in spaces:
            meta = self.get_space_metadata(int(space))
            meta["space_id"] = int(space)
            meta["captures"] = os.listdir(os.path.join(CAPTURE_TEMP, space))
            metadataList.append(meta)

        return metadataList

    def get_capture_metadata(self, space_id: int, capture_id: int):
        # get number of folders in the capture directory
        capture_dir = os.path.join(CAPTURE_TEMP, str(space_id), str(capture_id))
        scenes = []
        for scene_id in os.listdir(capture_dir):
            scene_dir = os.path.join(capture_dir, scene_id)
            scene_dir_http = f"/capture/result/{space_id}/{capture_id}/{scene_id}"
            try:
                with open(os.path.join(scene_dir, "meta.json"), "r") as f:
                    raw = f.read()
                    scene = json.loads(raw)

                    files = os.listdir(scene_dir)
                    del scene["lidar_position"]
                    scene["images"] = self.get_capture_scene_images(
                        space_id, capture_id, int(scene_id)
                    )
                    scene["space_id"] = space_id
                    scene["capture_id"] = capture_id
                    scene["scene_id"] = int(scene_id)
                    scenes.append(scene)
            except FileNotFoundError:
                continue
        if len(scenes) == 0:
            return None
        return {
            "space_id": space_id,
            "capture_id": capture_id,
            "scenes": scenes,
        }

    def get_capture_scene(
        self, space_id: int, capture_id: int, scene_id: int, filename: str
    ):
        return os.path.join(
            CAPTURE_TEMP, str(space_id), str(capture_id), str(scene_id), filename
        )

    def get_capture_scene_images(self, space_id: int, capture_id: int, scene_id: int):
        scene_dir = os.path.join(
            CAPTURE_TEMP, str(space_id), str(capture_id), str(scene_id)
        )
        files = os.listdir(scene_dir)
        return [f for f in files if f.endswith(".png") or f.endswith(".jpg")]

    def get_space_all_captures(self, space_id: int):
        capture_dir = os.path.join(CAPTURE_TEMP, str(space_id))
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
        capture_list.sort(key=lambda x: x["capture_id"], reverse=True)
        return capture_list

    def get_space_all_scenes(self, space_id: int):
        capture_dir = os.path.join(CAPTURE_TEMP, str(space_id))
        captures = os.listdir(capture_dir)
        scenes = []
        for capture_id in captures:
            if not capture_id.isdigit():
                continue
            scenes.extend(
                (self.get_capture_metadata(space_id, int(capture_id)) or {}).get(
                    "scenes", []
                )
            )
        return scenes

    def store_captured_scene(
        self, space_id: int, capture_id: int, scene_id: int, scene: CaptureSingleScene
    ):
        space_dir = f"{CAPTURE_TEMP}/{space_id}"
        capture_dir = f"{CAPTURE_TEMP}/{space_id}/{capture_id}"
        scene_dir = f"{capture_dir}/{scene_id}"
        for d in [space_dir, capture_dir, scene_dir]:
            if not os.path.exists(d):
                os.mkdir(d)

        for i, image in enumerate(scene.picture_list):
            filename = f"{scene_dir}/{image.topic.replace('/','_')}.png"
            cv2.imwrite(filename, image.image)
            with open(filename.replace(".png", ".json"), "w") as f:
                json.dump(image.data, f)
        with open(f"{scene_dir}/meta.json", "w") as f:
            json.dump(scene.to_dict(), f)
