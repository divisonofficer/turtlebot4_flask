from curses import raw
import os
from typing import Optional
from PIL import Image
import cv2
import tqdm

from moviepy.editor import ImageClip, concatenate_videoclips
import numpy as np


class StereoDepthGif:

    def __init__(self):
        pass

    def create_mp4(
        self,
        folder_path: str,
        from_img: Optional[str] = None,
        to_img: Optional[str] = None,
    ):
        files = self.read_image_files(folder_path, from_img, to_img)
        durations = self.read_files_to_duration_list(files)
        output_mp4 = os.path.join(folder_path, "output.mp4")
        clips = []
        for i in tqdm.tqdm(range(len(files))):
            img = files[i]
            duration = durations[i]
            merged_path = os.path.join(folder_path, img) + "_merged.png"
            if not os.path.exists(merged_path):
                merged = self.merge_images(os.path.join(folder_path, img))
                cv2.imwrite(merged_path, merged)

            clips.append(
                ImageClip(
                    merged_path,
                    duration=duration / 1000,
                ).set_fps(24)
            )

        # 모든 클립을 연결합니다
        video = concatenate_videoclips(clips, method="compose")

        # MP4 파일로 저장합니다
        video.write_videofile(output_mp4, codec="libx264")

    def read_image_files(
        self,
        folder_path: str,
        from_img: Optional[str] = None,
        to_img: Optional[str] = None,
    ):
        files = sorted(os.listdir(folder_path))
        files = [x for x in files if os.path.isdir(os.path.join(folder_path, x))]
        if from_img is not None:
            index_from = [i for i, x in enumerate(files) if from_img in x][0]
            files = files[index_from:]
        if to_img is not None:
            index_to = [i for i, x in enumerate(files) if to_img in x][0]
            files = files[:index_to]

        return files

    def read_files_to_pil_images(self, files):
        images = []
        for file in files:
            images.append(Image.fromarray(self.merge_images(file)))
        return images

    def merge_images(self, folder):
        if os.path.exists(os.path.join(folder, "raw.npz")):
            print(folder)
            raw = np.load(os.path.join(folder, "raw.npz"))
            viz_left = Image.fromarray(raw["left"][:, :, 2])
            viz_right = Image.fromarray(raw["right"][:, :, 2])
            lidar_range = Image.fromarray((raw["ranges"] / 256).astype(np.uint8))
            lidar_reflectivity = Image.fromarray(raw["reflectivity"].astype(np.uint8))

        else:
            viz_left_path = os.path.join(folder, "rgb", "left.png")
            viz_right_path = os.path.join(folder, "rgb", "right.png")
            lidar_range_path = os.path.join(folder, "lidar_range.png")
            lidar_reflectivity_path = os.path.join(folder, "lidar_reflectivity.png")

            lidar_range = Image.open(lidar_range_path)
            lidar_reflectivity = Image.open(lidar_reflectivity_path)
            viz_left = Image.open(viz_left_path)
            viz_right = Image.open(viz_right_path)
        viz_image_width = lidar_range.width // 2
        viz_image_height = int(viz_left.height * viz_image_width // viz_left.width)
        viz_left = viz_left.resize((viz_image_width, viz_image_height))
        viz_right = viz_right.resize((viz_image_width, viz_image_height))

        viz_image = np.concatenate(
            (
                cv2.cvtColor(np.array(viz_left), cv2.COLOR_BayerRG2BGR),
                cv2.cvtColor(np.array(viz_right), cv2.COLOR_BayerRG2BGR),
            ),
            axis=1,
        )
        if viz_image.shape[-1] != lidar_range.width:
            viz_image = cv2.resize(viz_image, (lidar_range.width, viz_image_height))
        viz_image = np.concatenate(
            (
                viz_image,
                cv2.cvtColor(np.array(lidar_range), cv2.COLOR_GRAY2BGR),
                cv2.cvtColor(np.array(lidar_reflectivity), cv2.COLOR_GRAY2BGR),
            ),
            axis=0,
        )

        return viz_image

    def read_files_to_duration_list(self, files):
        duration = []
        for i, img in enumerate(files):
            if i == 0:
                duration.append(1000)
                continue
            duration.append(
                (self.get_time_second(files[i]) - self.get_time_second(files[i - 1]))
                * 1000
            )
        return duration

    def create_gif(
        self,
        folder_path: str,
        from_img: Optional[str] = None,
        to_img: Optional[str] = None,
    ):

        output_gif = os.path.join(folder_path, "output.gif")
        files = self.read_image_files(folder_path, from_img, to_img)
        images, duration = self.read_files_to_pil_images(
            files
        ), self.read_files_to_duration_list(files)
        images[0].save(
            output_gif,
            save_all=True,
            append_images=images[1:],
            duration=duration,
            loop=0,
        )

    def get_time_second(self, path: str):
        time_str = path.split("_")
        return (
            int(time_str[0]) * 3600
            + int(time_str[1]) * 60
            + int(time_str[2])
            + (
                int(time_str[3]) / 1000.0
                if len(time_str) > 3 and time_str[3].isdigit()
                else 0
            )
        )


import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder_path", type=str)
    parser.add_argument("--from_img", type=str, default=None)
    parser.add_argument("--to_img", type=str, default=None)
    parser.add_argument("--mp4", action="store_true")
    parser.add_argument("--gif", action="store_true")
    args = parser.parse_args()
    gif = StereoDepthGif()
    if args.mp4:
        gif.create_mp4(args.folder_path, args.from_img, args.to_img)
    if args.gif:
        gif.create_gif(args.folder_path, args.from_img, args.to_img)
