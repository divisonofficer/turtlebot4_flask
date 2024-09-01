import os
from typing import Optional
from PIL import Image
import tqdm

from moviepy.editor import ImageClip, concatenate_videoclips
import cv2
import numpy as np


class StereoDepthGif:

    def __init__(self, args):
        self.args = args

    def create_mp4(
        self,
        folder_path: str,
        from_img: Optional[str] = None,
        to_img: Optional[str] = None,
    ):
        files = self.read_image_files(folder_path, from_img, to_img)
        durations = self.read_files_to_duration_list(files)
        output_mp4 = os.path.join(folder_path, self.args.output)
        clips = []
        for i in tqdm.tqdm(range(len(files))):
            img = files[i] + "_merged.png"
            image_merged = self.merge_image_property(
                os.path.join(folder_path, files[i])
            )
            cv2.imwrite(os.path.join(folder_path, img), image_merged)
            duration = durations[i]
            clips.append(
                ImageClip(
                    os.path.join(folder_path, img), duration=duration / 1000
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
            files = files[: index_to + 1]

        return files

    def merge_image_property(self, scene: str):
        left_rgb = cv2.imread(os.path.join(scene, "rgb", "left.png"))
        right_rgb = cv2.imread(os.path.join(scene, "rgb", "right.png"))
        left_nir = (
            cv2.imread(os.path.join(scene, "nir", "left.png"), cv2.IMREAD_GRAYSCALE)
            .reshape((left_rgb.shape[0], left_rgb.shape[1], 1))
            .repeat(3, axis=2)
        )
        right_nir = (
            cv2.imread(os.path.join(scene, "nir", "right.png"), cv2.IMREAD_GRAYSCALE)
            .reshape((left_rgb.shape[0], left_rgb.shape[1], 1))
            .repeat(3, axis=2)
        )

        dis_rgb = cv2.imread(
            os.path.join(scene, "rgb", "disparity.png"), cv2.IMREAD_GRAYSCALE
        )
        dis_nir = cv2.imread(
            os.path.join(scene, "nir", "disparity.png"), cv2.IMREAD_GRAYSCALE
        )
        dis_rgb = self.disparity_colorize(dis_rgb, self.args.max_disparity)
        dis_nir = self.disparity_colorize(dis_nir, self.args.max_disparity)
        rgb = np.concatenate((left_rgb, right_rgb, dis_rgb), axis=1)
        nir = np.concatenate((left_nir, right_nir, dis_nir), axis=1)
        merged = np.concatenate((rgb, nir), axis=0)
        colorbar = self.colorbar(merged.shape[0], self.args.max_disparity)
        return np.concatenate((merged, colorbar), axis=1)

    def disparity_colorize(self, disparity, max_disparity):
        disparity = np.clip(disparity, 0, max_disparity) * 255.0 / max_disparity
        return cv2.applyColorMap(disparity.astype(np.uint8), cv2.COLORMAP_MAGMA)

    def colorbar(self, height, max_disparity):
        colorbar = np.linspace(0, 255).astype(np.uint8)
        colorbar = cv2.applyColorMap(colorbar, cv2.COLORMAP_MAGMA)

        colorbar = cv2.resize(colorbar, (128, height))

        colorbar[:, 64:] = (255, 255, 255)
        cv2.putText(
            colorbar,
            "0",
            (64, 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 0),
            2,
        )
        cv2.putText(
            colorbar,
            str(max_disparity),
            (64, height - 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 0),
            2,
        )
        return colorbar

    def read_files_to_pil_images(self, files):
        images = []
        for scene in files:
            merged_image = self.merge_image_property(scene)
            images.append(Image.fromarray(merged_image))
        return images

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
    parser.add_argument("--output", type=str, default="output.mp4")
    parser.add_argument("--max_disparity", type=int, default=64)
    args = parser.parse_args()
    gif = StereoDepthGif(args)
    if args.mp4:
        gif.create_mp4(args.folder_path, args.from_img, args.to_img)
    if args.gif:
        gif.create_gif(args.folder_path, args.from_img, args.to_img)
