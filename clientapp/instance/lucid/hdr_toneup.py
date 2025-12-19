import argparse
import cv2
import os
import numpy as np

from lucid_postprocess import LucidPostProcess


args = argparse.ArgumentParser()
args.add_argument("--input", type=str, required=True)
args.add_argument("--gamma", type=float, default=2.2)
args = args.parse_args()

input_folder = args.input
output_folder = f"{input_folder}"
pairs = {}
os.makedirs(output_folder, exist_ok=True)
# Using os.walk to find *.npy files
# 1) os.walk로 하위 폴더까지 모두 탐색
for root, dirs, files in os.walk(input_folder):
    rel_path = os.path.relpath(root, input_folder)  # input_folder 기준 상대 경로
    for file in files:
        if file.endswith("lucid_left_image.npy"):
            if rel_path not in pairs:
                pairs[rel_path] = {}
            pairs[rel_path]["left"] = file

        elif file.endswith("lucid_right_image.npy"):
            if rel_path not in pairs:
                pairs[rel_path] = {}
            pairs[rel_path]["right"] = file


tonemap = cv2.createTonemapMantiuk(gamma=args.gamma)
import tqdm


lucid_post = LucidPostProcess()
lucid_post.load_calibration("calibration.npz")


pairs_items = list(pairs.items())
pairs_items.sort(key=lambda x: x[0])

# 2) 페어로 있는 폴더들만 순회하며 처리
for i, (rel_path, lr_dict) in tqdm.tqdm(enumerate(pairs_items)):

    left_file = lr_dict.get("left", None)
    right_file = lr_dict.get("right", None)

    # 만약 해당 상대 경로에 left와 right가 모두 있다면 처리한다.
    if left_file and right_file:
        # if os.path.exists(f"{output_folder}/{rel_path}/left_rectified.npy"):
        #     continue
        # 실제 파일 전체 경로
        left_path = os.path.join(input_folder, rel_path, left_file)
        right_path = os.path.join(input_folder, rel_path, right_file)
        # try:
        #     os.remove(os.path.join(input_folder, rel_path, "left.png"))
        #     os.remove(os.path.join(input_folder, rel_path, "right.png"))
        # except FileNotFoundError:
        #     pass
        # try:
        #     os.remove(os.path.join(input_folder, rel_path, "stereo_toneup.png"))
        # except FileNotFoundError:
        #     pass

        # NumPy Array 로딩
        try:
            left_img = np.load(left_path)
            right_img = np.load(right_path)
        except ValueError:
            continue

        left_img = lucid_post.rawUint8ToUint32(left_img)
        right_img = lucid_post.rawUint8ToUint32(right_img)
        left_img = lucid_post.bayerToBgr(left_img).astype(np.float32)
        right_img = lucid_post.bayerToBgr(right_img).astype(np.float32)
        np.save(f"{output_folder}/{rel_path}/left_rgb.npy", left_img)
        np.save(f"{output_folder}/{rel_path}/right_rgb.npy", right_img)

        left_img, right_img = lucid_post.rectify_stereo(left_img, right_img)

        np.save(f"{output_folder}/{rel_path}/left_rectified.npy", left_img)
        np.save(f"{output_folder}/{rel_path}/right_rectified.npy", right_img)

        img = np.concatenate([left_img, right_img], axis=1)
        cv2.imwrite(
            f"{output_folder}/{rel_path}/stereo.png",
            np.clip(img * 255 * 16, 0, 255).astype(np.uint8),
        )
        # img = tonemap.process(img)
        # img = np.clip(img * 255, 0, 255).astype(np.uint8)
        # cv2.imwrite(f"{output_folder}/{rel_path}/stereo_toneup.png", img)
