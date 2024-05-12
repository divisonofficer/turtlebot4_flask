import os
import time
from PIL import Image
import yaml
from slam_pb2 import *
from google.protobuf import json_format


class SlamRepo:
    def __init__(self):
        self.ROOT = "/tmp/slam"

        if not os.path.exists(self.ROOT):
            os.mkdir(self.ROOT)

    def get_map_list(self):
        files = os.listdir(self.ROOT)
        files = [f for f in files if f.endswith(".yaml")]
        map_values = []
        for i, f in enumerate(files):
            with open(f"{self.ROOT}/{f}", "r") as file:
                file_create_time = os.path.getctime(f"{self.ROOT}/{f}")
                file_create_time_str = time.strftime(
                    "%Y-%m-%d %H:%M:%S", time.localtime(file_create_time)
                )
                name = f.replace(".yaml", "")
                map_values.append(
                    SavedMapInfo(
                        id=i,
                        name=name,
                        content=self.get_map_metadata(name),
                        create_time=file_create_time,
                        create_time_str=file_create_time_str,
                        preview="/slam/map/" + name + "/image",
                    )
                )

        return map_values

    def save_map_available(self, map_name):
        """
        check map_name.yaml is exists in the folder
        """
        yaml_path = f"{self.ROOT}/{map_name}.yaml"
        if os.path.exists(yaml_path):
            return None

        return self.ROOT + "/" + map_name

    def get_map_metadata(self, map_name):
        yaml_path = f"{self.ROOT}/{map_name}.yaml"
        if not os.path.exists(yaml_path):
            return None
        with open(yaml_path, "r") as file:
            metadata = yaml.safe_load(file)

            image = Image.open(f"{self.ROOT}/{map_name}.pgm")
            metadata["map_size"] = {
                "width": image.width,
                "height": image.height,
                "resolution": metadata["resolution"],
            }
            metadata["map_origin"] = {
                "x": metadata["origin"][0],
                "y": metadata["origin"][1],
                "z": metadata["origin"][2],
            }
            del metadata["origin"]
            metaProto = SavedMapMeta()
            json_format.ParseDict(metadata, metaProto)

            return metaProto

    def load_map_available(self, map_name):
        """
        load map_name.yaml
        """
        yaml_path = f"{self.ROOT}/{map_name}.yaml"
        if not os.path.exists(yaml_path):
            return None
        return self.ROOT + "/" + map_name

    def get_map_image(self, map_name):
        """
        get map image
        """
        image_path = f"{self.ROOT}/{map_name}.pgm"
        image_path_png = f"{self.ROOT}/{map_name}.png"
        if os.path.exists(image_path_png):
            return image_path_png
        if not os.path.exists(image_path):
            return None

        with Image.open(image_path) as img:
            img.save(image_path_png, "PNG")
        return image_path_png
