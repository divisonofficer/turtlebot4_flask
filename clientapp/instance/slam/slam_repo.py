import os
import time
from PIL import Image


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
                map_values.append(
                    {
                        "id": i,
                        "name": f.replace(".yaml", ""),
                        "content": file.read(),
                        "create_time": file_create_time,
                        "create_time_str": file_create_time_str,
                        "preview": "/slam/map/" + f.replace(".yaml", "") + "/image",
                    }
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
