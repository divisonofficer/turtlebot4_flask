import os
import json
import cv2
from capture_type import CaptureSingleScene


CAPTURE_TEMP = "/tmp/oakd_capture"
if not os.path.exists(CAPTURE_TEMP):
    os.mkdir(CAPTURE_TEMP)


class CaptureStorage:

    def __init__(self):
        pass

    def get_space_metadata(self, space_id: int):
        capture_dir = os.path.join(CAPTURE_TEMP, str(space_id))
        captures = os.listdir(capture_dir)

        return {"space_id": space_id, "captures": [int(c) for c in captures]}

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
                    scene["images"] = [
                        f"{scene_dir_http}/{f}"
                        for f in files
                        if f.endswith(".jpg") and f != "oakd_mono_data.json"
                    ]
                    scenes.append(scene)
            except FileNotFoundError:
                continue

        return scenes

    def get_capture_scene(
        self, space_id: int, capture_id: int, scene_id: int, filename: str
    ):
        return os.path.join(
            CAPTURE_TEMP, str(space_id), str(capture_id), str(scene_id), filename
        )

    def get_space_all_scenes(self, space_id: int):
        capture_dir = os.path.join(CAPTURE_TEMP, str(space_id))
        captures = os.listdir(capture_dir)
        scenes = []
        for capture_id in captures:
            scenes.extend(self.get_capture_metadata(space_id, int(capture_id)))
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
            filename = f"{scene_dir}/{image.topic.replace('/','_')}.jpg"
            cv2.imwrite(filename, image.image)
            with open(filename.replace(".jpg", ".json"), "w") as f:
                json.dump(image.data, f)
        with open(f"{scene_dir}/meta.json", "w") as f:
            json.dump(scene.to_dict(), f)
