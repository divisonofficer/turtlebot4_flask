import sys
from typing import Union

sys.path.append("../../../public/proto/python")


from capture_type import ImageBytes
import numpy as np
import cv2
from pypolar import mueller
import polanalyser as pa

MM_QWP: list[list] = [[], [], [], []]

MM_L: list = []


class PolarizationCompute:
    image_list_files: list[str] = []

    def __init__(self):
        global MM_L
        global MM_QWP
        if MM_L == []:
            MM_L = self.update_linear_matrix(np.pi / 4 * 0.33)
        if MM_QWP == [[], [], [], []]:
            for i in range(4):
                MM_QWP[i] = self.muller_matrix_qwp(np.pi * i / 4)

    def put_image_list(self, image_list: list[str]):
        if len(self.image_list_files) > 0 and self.image_list_files[0] == image_list[0]:
            return
        self.image_list_files = image_list
        self.image_list = [
            ImageBytes(cv2.imread(image), topic=f"image{i}")
            for i, image in enumerate(image_list)
        ]

    def update_qwp_angles(self, angles: list[float]):
        global MM_QWP
        for i in range(4):
            MM_QWP[i] = self.muller_matrix_qwp(np.pi * angles[i] / 180)
            print(f"QWP {i}({angles[i]}degree): ", MM_QWP[i])
        return MM_QWP

    def compute(self) -> list[list[np.ndarray]]:
        np_MM_QWP = np.array(MM_QWP)
        np_MM_L = np.array(MM_L)
        np_MM = [
            np.dot(
                np_MM_L,
                np_MM_QWP[0],
            ),
            np.dot(
                np_MM_L,
                np_MM_QWP[1],
            ),
            np.dot(
                np_MM_L,
                np_MM_QWP[2],
            ),
            np.dot(
                np_MM_L,
                np_MM_QWP[3],
            ),
        ]
        np_MM_T = np.concatenate([np_MM[i][0].reshape(1, 4) for i in range(4)], axis=0)
        I = self.get_intensity_matrix()
        # I를 h*w*3*4*1 모양으로 확장

        M_inv = np.linalg.pinv(np_MM_T)  # M_inv 계산
        # M_inv를 (4, 4) -> (1, 1, 1, 4, 4)로 확장
        S = np.einsum("...i,ij->...j", I, M_inv)
        print(S.shape)

        intensities = np.moveaxis(I.squeeze(), -1, 0)
        S = pa.calcStokes(intensities, np_MM)

        img_color_stokes = np.array(S)  # (H, W, C, 4)
        img_stokes = img_color_stokes[:, :, 1, :]  # (H, W, 4)
        img_aolp = pa.cvtStokesToAoLP(img_stokes)
        img_dolp = pa.cvtStokesToDoLP(img_stokes)
        img_aolp_vis = pa.applyColorToAoLP(img_aolp, img_dolp)
        img_dolp_vis = np.clip(img_dolp * 255, 0, 255).astype(np.uint8)
        img_dolp_vis = cv2.cvtColor(img_dolp_vis, cv2.COLOR_GRAY2BGR)

        stokes_images = [S[:, :, :, i] for i in range(4)]
        stokes_images = [
            [
                np.clip(image, 0, 255).astype(np.uint8),
                np.clip(-image, 0, 255).astype(np.uint8),
            ]
            for i, image in enumerate(stokes_images)
        ]
        stokes_images += [
            [
                img_aolp_vis,
                img_dolp_vis,
            ]
        ]
        return stokes_images

    def get_intensity_matrix(self):
        intensity_matrix = []

        for image in self.image_list:
            intensity_matrix.append(image.image)
        intensity_matrix = np.stack(intensity_matrix, axis=3)
        return intensity_matrix

    def muller_matrix_qwp(self, angle: float):
        return np.array(mueller.op_quarter_wave_plate(angle)).round(7)

    def muller_matrix_linear(self, angle: float):
        return np.array(mueller.op_linear_polarizer(angle)).round(7)

    def update_linear_matrix(self, angle: float):
        global MM_L
        MM_L = self.muller_matrix_linear(angle)
        return MM_L


if __name__ == "__main__":
    pass
    # image_list = [
    #     ImageBytes(image=cv2.imread("0.png"), topic="image1"),
    #     ImageBytes(image=cv2.imread("45.png"), topic="image2"),
    #     ImageBytes(image=cv2.imread("90.png"), topic="image2"),
    #     ImageBytes(image=cv2.imread("135.png"), topic="image2"),
    # ]
    # pc = PolarizationCompute(image_list)

    # MM_L = pc.muller_matrix_linear(np.pi / 4 * 0.33)
    # print(MM_L)
    # angles = [0, 45, 90, 135]
    # for i in range(4):
    #     # print(45 * i, " degree : ")
    #     mat = pc.muller_matrix_qwp(np.pi * angles[i] / 180 * i)
    #     # print(mat)
    #     if i < 4:
    #         MM_QWP[i] = mat

    # pc.compute()
