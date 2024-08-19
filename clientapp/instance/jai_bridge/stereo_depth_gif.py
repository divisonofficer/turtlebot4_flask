import os
from typing import Optional
from PIL import Image
import tqdm

from moviepy.editor import ImageClip, concatenate_videoclips


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
        files = [x for x in files if x.endswith(".png")]
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
            images.append(Image.open(file))
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
    args = parser.parse_args()
    gif = StereoDepthGif()
    if args.mp4:
        gif.create_mp4(args.folder_path, args.from_img, args.to_img)
    if args.gif:
        gif.create_gif(args.folder_path, args.from_img, args.to_img)
