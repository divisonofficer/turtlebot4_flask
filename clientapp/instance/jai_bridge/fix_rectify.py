import argparse


import tqdm

parser = argparse.ArgumentParser(description="Fix rectify")
parser.add_argument("--input_folder", type=str, help="Input folder")
parser.add_argument("--calibration_file", type=str, help="Calibration file")

args = parser.parse_args()


import os
import cv2
import numpy as np


image_list = [
    x
    for x in os.listdir(args.input_folder)
    if os.path.isdir(os.path.join(args.input_folder, x)) and x.split("_")[-1] != "fixed"
]

calibration = np.load(args.calibration_file)


def compute_invert_map(map1x, map1y, w, h):
    inverse_map1x = np.zeros_like(map1x)
    inverse_map1y = np.zeros_like(map1y)

    for i in range(h):
        for j in range(w):
            # 보정된 좌표 (map1x[i, j], map1y[i, j])를 역으로 변환
            original_x = map1x[i, j]
            original_y = map1y[i, j]

            # 원본 좌표로 복원
            if 0 <= original_x < w and 0 <= original_y < h:
                inverse_map1x[int(original_y), int(original_x)] = j
                inverse_map1y[int(original_y), int(original_x)] = i
    return inverse_map1x, inverse_map1y


def compute_map_from_calibration(cal, invert=False):
    image_size = cal["image_size"] if "image_size" in cal else (720, 540)

    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
        cal["mtx_left"],
        cal["dist_left"],
        cal["mtx_right"],
        cal["dist_right"],
        image_size,
        cal["R"],
        cal["T"],
    )

    map1x, map1y = cv2.initUndistortRectifyMap(
        cal["mtx_left"], cal["dist_left"], R1, P1, image_size, cv2.CV_32FC1
    )
    map2x, map2y = cv2.initUndistortRectifyMap(
        cal["mtx_right"], cal["dist_right"], R2, P2, image_size, cv2.CV_32FC1
    )
    if invert:
        map1x, map1y = compute_invert_map(map1x, map1y, image_size[0], image_size[1])
        map2x, map2y = compute_invert_map(map2x, map2y, image_size[0], image_size[1])

    return map1x, map1y, map2x, map2y


revert_map1x, revert_map1y, revert_map2x, revert_map2y = None, None, None, None


def revert_rectify(image_left, image_right, cal):
    global revert_map1x, revert_map1y, revert_map2x, revert_map2y
    if (
        revert_map1x is None
        or revert_map1y is None
        or revert_map2x is None
        or revert_map2y is None
    ):

        revert_map1x, revert_map1y, revert_map2x, revert_map2y = (
            compute_map_from_calibration(cal, invert=True)
        )
    image_left, image_right = cv2.remap(
        image_left, revert_map1x, revert_map1y, cv2.INTER_LINEAR
    ), cv2.remap(image_right, revert_map2x, revert_map2y, cv2.INTER_LINEAR)
    mask_left = np.ones_like(image_left, dtype=np.uint8) * 255
    mask_left = np.all(image_left == 0, axis=2).astype(np.uint8)
    mask_right = np.all(image_right == 0, axis=2).astype(np.uint8)

    cv2.imwrite("mask_left.png", mask_left)
    cv2.imwrite("mask_right.png", mask_right)

    image_left = cv2.inpaint(image_left, mask_left, 7, cv2.INPAINT_TELEA)
    image_right = cv2.inpaint(image_right, mask_right, 7, cv2.INPAINT_TELEA)
    return image_left, image_right


def rectify(image_left, image_right, cal):
    map1x, map1y, map2x, map2y = compute_map_from_calibration(cal, invert=False)
    return cv2.remap(image_left, map1x, map1y, cv2.INTER_LINEAR), cv2.remap(
        image_right, map2x, map2y, cv2.INTER_LINEAR
    )


C_TOP = 24
C_H = 48

for frame in tqdm.tqdm(image_list):
    folder = os.path.join(args.input_folder, frame)
    pre_calibration = np.load(os.path.join(folder, "post.npz"))
    imleft = cv2.imread(os.path.join(folder, "rgb", "left.png"))
    imright = cv2.imread(os.path.join(folder, "rgb", "right.png"))
    im_nir_left = cv2.imread(os.path.join(folder, "nir", "left.png"))
    im_nir_right = cv2.imread(os.path.join(folder, "nir", "right.png"))
    if pre_calibration["mtx_left"][0, 0] != calibration["mtx_left"][0, 0]:
        try:
            imleft, imright = revert_rectify(imleft, imright, pre_calibration)
            imleft, imright = rectify(imleft, imright, calibration)
            imleft = imleft[C_TOP:, C_H:-C_H]
            imright = imright[C_TOP:, C_H:-C_H]
            im_nir_left, im_nir_right = revert_rectify(
                im_nir_left, im_nir_right, pre_calibration
            )
            im_nir_left, im_nir_right = rectify(im_nir_left, im_nir_right, calibration)
            im_nir_left = im_nir_left[C_TOP:-C_TOP, C_H:-C_H]
            im_nir_right = im_nir_right[C_TOP:-C_TOP, C_H:-C_H]
            pre_calibration = {**pre_calibration}
            for key in calibration:
                pre_calibration[key] = calibration[key]
            pre_calibration["image_size"] = (720 - C_H * 2, 540 - C_TOP * 2)
            pre_calibration["mtx_left"][1, 2] -= C_TOP
            pre_calibration["mtx_right"][1, 2] -= C_TOP
            pre_calibration["mtx_left"][0, 2] -= C_H
            pre_calibration["mtx_right"][0, 2] -= C_H
        except Exception as e:
            print(e)
            print("error ", folder)
            continue
    elif imleft.shape[0] == 540 - C_TOP:
        imleft = imleft[:-C_TOP]
        imright = imright[:-C_TOP]
        im_nir_left = im_nir_left[:-C_TOP]
        im_nir_right = im_nir_right[:-C_TOP]
        pre_calibration = {**pre_calibration}
        pre_calibration["image_size"] = (720 - C_H * 2, 540 - C_TOP * 2)
    elif pre_calibration["image_size"][1] == 540 - C_TOP:
        pre_calibration = {**pre_calibration}
        pre_calibration["image_size"] = (720 - C_H * 2, 540 - C_TOP * 2)
        np.savez(os.path.join(folder, "post.npz"), **pre_calibration)
        continue

    folder_new = os.path.join(
        args.input_folder + ("_fixed" if not "fixed" in args.input_folder else ""),
        frame,
    )
    os.makedirs(os.path.join(folder_new, "rgb"), exist_ok=True)
    os.makedirs(os.path.join(folder_new, "nir"), exist_ok=True)
    cv2.imwrite(os.path.join(folder_new, "rgb", "left.png"), imleft)
    cv2.imwrite(os.path.join(folder_new, "rgb", "right.png"), imright)
    cv2.imwrite(os.path.join(folder_new, "nir", "left.png"), im_nir_left)
    cv2.imwrite(os.path.join(folder_new, "nir", "right.png"), im_nir_right)
    np.savez(os.path.join(folder_new, "post.npz"), **pre_calibration)
