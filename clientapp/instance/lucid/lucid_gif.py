from curses import raw
import os
from typing import Optional
from PIL import Image
import cv2
import tqdm

from moviepy.editor import ImageClip, concatenate_videoclips
import numpy as np
from lucid_postprocess import LucidPostProcess

COLORBAR_WIDTH = 32
MAX_RANGE = 24000.0
MAX_REFLECTIVITY = 255.0


class StereoDepthGif:

    def __init__(self):

        # 감마 값 설정
        gamma = 2.2

        # 최대 값 설정 (24비트 이미지의 최대 값은 16,777,215)
        max_value = 2**12 - 1

        # 감마 보정을 위한 LookUp Table 생성
        inv_gamma = 1.0 / gamma
        self.gamma_table = np.array(
            [
                ((i / max_value) ** inv_gamma) * max_value
                for i in np.arange(0, max_value + 1)
            ]
        ).astype("uint32")

        self.post_process = LucidPostProcess()

    def create_mp4(
        self,
        folder_path: str,
        from_img: Optional[str] = None,
        to_img: Optional[str] = None,
    ):
        files = self.read_image_files(folder_path, from_img, to_img)
        durations = self.read_files_to_duration_list(files)
        output_mp4 = folder_path + ".mp4"
        clips = []
        for i in tqdm.tqdm(range(len(files))):
            img = files[i]
            duration = durations[i]
            merged_path = os.path.join(folder_path, img) + "_merged.png"
            prev_folder = os.path.join(folder_path, files[i - 1]) if i > 0 else None
            next_folder = (
                os.path.join(folder_path, files[i + 1]) if i < len(files) - 1 else None
            )
            merged = self.merge_images(
                os.path.join(folder_path, img), prev_folder, next_folder
            )

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

    def gamma_correction(self, img):
        img = img.astype(np.uint32)
        img = img[:, :, 0] + (img[:, :, 1] << 8) + (img[:, :, 2] << 16)
        img = img >> 12
        img = self.gamma_table[img]
        return img

    def merge_images(
        self,
        folder,
        prev_folder: Optional[str] = None,
        next_folder: Optional[str] = None,
    ):
        if os.path.exists(os.path.join(folder, "raw.npz")):
            raw = np.load(os.path.join(folder, "raw.npz"))

            concat_raw = np.concatenate(
                (raw["left"], raw["right"]),
                axis=1,
            )

            if prev_folder is not None:
                prev_raw = np.load(os.path.join(prev_folder, "raw.npz"))
                prev_concat = np.concatenate(
                    (prev_raw["left"], prev_raw["right"]),
                    axis=1,
                )
                concat_raw = np.concatenate(
                    (prev_concat, concat_raw),
                    axis=0,
                )
            if next_folder is not None:
                next_raw = np.load(os.path.join(next_folder, "raw.npz"))
                next_concat = np.concatenate(
                    (next_raw["left"], next_raw["right"]),
                    axis=1,
                )
                concat_raw = np.concatenate(
                    (concat_raw, next_concat),
                    axis=0,
                )

            concat_raw = self.post_process.rawUint8ToTonemappedBgr(concat_raw)
            np_left = concat_raw[:, : concat_raw.shape[1] // 2]
            np_right = concat_raw[:, concat_raw.shape[1] // 2 :]

            if prev_folder is not None:
                np_left = np_left[prev_raw["left"].shape[0] :]
                np_right = np_right[prev_raw["right"].shape[0] :]
            if next_folder is not None:
                np_left = np_left[: next_raw["left"].shape[0]]
                np_right = np_right[: next_raw["right"].shape[0]]

            viz_left = Image.fromarray(np_left.astype(np.uint8))
            viz_right = Image.fromarray(np_right.astype(np.uint8))
            lidar_range = Image.fromarray(raw["ranges"])
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
                np.array(viz_left),
                np.array(viz_right),
            ),
            axis=1,
        )
        if viz_image.shape[-1] != lidar_range.width:
            viz_image = cv2.resize(viz_image, (lidar_range.width, viz_image_height))

        lidar_range = np.array(lidar_range)
        lidar_range = (np.clip(lidar_range, 0, MAX_RANGE) / MAX_RANGE * 255).astype(
            np.uint8
        )
        color_bar = cv2.applyColorMap(
            np.arange(256).reshape(256, 1).astype(np.uint8), cv2.COLORMAP_JET
        )
        color_bar = cv2.resize(color_bar, (COLORBAR_WIDTH, lidar_range.shape[0]))
        cv2.putText(
            color_bar,
            "24",
            (0, lidar_range.shape[0] - 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        lidar_range = cv2.applyColorMap(lidar_range, cv2.COLORMAP_JET)
        lidar_range[:, :COLORBAR_WIDTH] = color_bar

        lidar_reflectivity = np.array(lidar_reflectivity)
        lidar_reflectivity = (
            np.clip(lidar_reflectivity, 0, MAX_REFLECTIVITY) / MAX_REFLECTIVITY * 255
        ).astype(np.uint8)

        color_bar = cv2.applyColorMap(
            np.arange(256).reshape(256, 1).astype(np.uint8), cv2.COLORMAP_MAGMA
        )
        color_bar = cv2.resize(color_bar, (COLORBAR_WIDTH, lidar_reflectivity.shape[0]))
        cv2.putText(
            color_bar,
            "255",
            (0, lidar_range.shape[0] - 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        lidar_reflectivity = cv2.applyColorMap(lidar_reflectivity, cv2.COLORMAP_MAGMA)
        lidar_reflectivity[:, :COLORBAR_WIDTH] = color_bar
        viz_image = np.concatenate(
            (
                viz_image,
                lidar_range,
                lidar_reflectivity,
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
    parser.add_argument("--have_subfolder", action="store_true")
    parser.add_argument("--from_img", type=str, default=None)
    parser.add_argument("--to_img", type=str, default=None)
    parser.add_argument("--mp4", action="store_true")
    parser.add_argument("--gif", action="store_true")
    args = parser.parse_args()
    gif = StereoDepthGif()

    folder_paths = [args.folder_path]
    if args.have_subfolder:
        folder_paths = [
            os.path.join(args.folder_path, x)
            for x in os.listdir(args.folder_path)
            if os.path.isdir(os.path.join(args.folder_path, x))
        ]
    for folder_path in folder_paths:
        if args.mp4:
            gif.create_mp4(folder_path, args.from_img, args.to_img)
        if args.gif:
            gif.create_gif(folder_path, args.from_img, args.to_img)
