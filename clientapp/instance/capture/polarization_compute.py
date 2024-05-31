import sys
from typing import Union

sys.path.append("../../../public/proto/python")


from capture_type import ImageBytes
import numpy as np
import cv2
from pypolar import mueller
import polanalyser as pa

MM_QWP: list[np.ndarray] = [np.array([]), np.array([]), np.array([]), np.array([])]

MM_L: np.ndarray = np.array([])


class PolarizationCompute:

    capture_meta = (-1, -1, -1)
    linear_angle = -1.0
    qwp_angle = -1.0
    image_np_dict: dict[str, np.ndarray] = {}
    image_list: list[tuple[np.ndarray, np.ndarray]] = []

    def __init__(self, mask_path: str = "instance/capture/jai_binete.png"):
        self.image_jai_mask = cv2.imread(mask_path)
        pass

    def prepare_scene(self, linear_angle: float, qwp_angle: float):
        if self.linear_angle == linear_angle and self.qwp_angle == qwp_angle:
            return False

        self.linear_angle = linear_angle

        self.qwp_angle = qwp_angle
        return True

    def put_image_list(self, capture_meta: tuple[int, int, int], image_list: list[str]):
        if self.capture_meta == capture_meta:
            return
        self.capture_meta = capture_meta

        image_rgb_list = [
            self.read_image(image, "rgb")
            for image in [x for x in image_list if "channel_0" in x]
        ]
        image_nir_list = [
            self.read_image(image, "nir")
            for image in [x for x in image_list if "channel_1" in x]
        ]

        self.image_list = [
            (
                image_rgb_list[i],
                image_nir_list[i],
            )
            for i in range(len(image_rgb_list))
        ]

    def read_image(self, image_path: str, format: str = "rgb"):
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)

        print(image.shape, self.image_jai_mask.shape)
        image = cv2.fastNlMeansDenoisingColored(image, None, 5, 5, 5, 15)
        image_masked = cv2.bitwise_and(image, self.image_jai_mask)
        if format == "nir":
            image_masked = cv2.cvtColor(image_masked, cv2.COLOR_BGR2GRAY)

        image_masked = image_masked.astype(np.float64)
        return image_masked

    def update_qwp_angles(self, angles: list[float]):
        global MM_QWP
        MM_QWP = []
        for i in range(len(angles)):
            MM_QWP.append(self.muller_matrix_qwp(np.pi * angles[i] / 180))
            print(f"QWP {i}({angles[i]}degree): ", MM_QWP[i])
        return MM_QWP

    def get_image_by_property(self, property: str):
        return self.image_np_dict[property]

    def compute(self):
        I = self.get_intensity_matrix()
        intensities = np.moveaxis(I.squeeze(), -1, 0)
        S = np.array(
            [
                self.calcStokes(intensities[:, :, :, i], self.get_stacked_mm())
                # pa.calcStokes(intensities[:, :, :, i], self.get_stacked_mm())  # type: ignore
                for i in range(intensities.shape[-1])
            ]
        )  # type: ignore

        S = np.moveaxis(S, -1, 0)
        S = np.moveaxis(S, 1, -1)
        print("Stokes shape: ", S.shape)
        S = self.denoise_stokes_multichannel(S)
        for i, s in enumerate(["s0", "s1", "s2", "s3"]):
            self.get_vis_stokes("rgb", s, S[i, :, :, 0:3])
            self.get_vis_stokes("nir", s, S[i, :, :, 3:4])

        for i, channel in enumerate(["b", "g", "r", "nir"]):
            img_stokes = S[:, :, :, i : i + 1]  # (H, W, 4)

            stokes_images_p = self.get_vis_aolp_dolp(channel, img_stokes)
            self.image_np_dict[f"{channel}_aolp"] = stokes_images_p[0]
            self.image_np_dict[f"{channel}_dolp"] = stokes_images_p[1]

    def calcStokes(self, intensity: np.ndarray, MM: list[np.ndarray]):
        MM_Stacked = np.stack([MM[i][0] for i in range(len(MM))], axis=0)
        intensity = np.moveaxis(intensity, 0, -1)
        print("intensity shape: ", intensity.shape)
        print("MM_Stacked shape: ", MM_Stacked.shape)
        stokes = np.einsum("ij,...j->...i", np.linalg.pinv(MM_Stacked), intensity)
        print("max", np.max(stokes), "min", np.min(stokes))

        return self.denoise_stokes(stokes)

    def denoise_stokes(self, stokes: np.ndarray):

        return stokes

    def denoise_stokes_multichannel(self, stokes: np.ndarray):

        return stokes

    def get_stacked_mm(self):
        np_MM_QWP = np.array(MM_QWP)
        np_MM_L = np.array(MM_L)
        np_MM = [
            np.dot(
                np_MM_L,
                np_MM_QWP[i],
            )
            for i in range(len(MM_QWP))
        ]
        return np_MM

    def get_vis_stokes(self, channel: str, property: str, img_stokes: np.ndarray):
        max_val = max(max(np.max(img_stokes), -np.min(img_stokes)), 255)

        img_stokes *= 255 / max_val

        self.image_np_dict[f"{property}_{channel}_positive"] = np.clip(
            img_stokes, 0, max_val
        ).astype(np.uint8)
        self.image_np_dict[f"{property}_{channel}_negative"] = np.clip(
            -img_stokes, 0, max_val
        ).astype(np.uint8)

    def get_vis_aolp_dolp(self, channel: str, img_stokes):
        print("get_vis_aolp_dolp", img_stokes.shape)
        img_stokes = np.moveaxis(img_stokes, 0, -1)
        img_aolp = pa.cvtStokesToAoLP(img_stokes)
        img_dolp = self.cvtStokesToDoLP(img_stokes)
        img_dolp[np.isnan(img_dolp)] = 0
        # img_dolp = np.clip(img_dolp * 255, 0, 255).astype(np.uint8)
        # img_dolp = (
        #     cv2.fastNlMeansDenoising(img_dolp, None, 10, 7, 21).astype(np.float64) / 255
        # ).reshape(img_dolp.shape[0], img_dolp.shape[1], 1)
        print(np.max(img_aolp), np.min(img_aolp), np.max(img_dolp), np.min(img_dolp))
        img_aolp_vis = cv2.bitwise_and(
            self.applyColorToAoLP(img_aolp, img_dolp).astype(np.uint8),
            self.image_jai_mask,
        )

        img_dolp_vis = np.clip(img_dolp * 255, 0, 255).astype(np.uint8)

        return img_aolp_vis, img_dolp_vis

    def cvtStokesToDoLP(self, stokes: np.ndarray) -> np.ndarray:
        s0 = stokes[..., 0]
        s0 = s0.clip(min=10)
        dolp = np.sqrt(stokes[..., 1] ** 2 + stokes[..., 2] ** 2) / s0
        return dolp

    def applyColorToAoLP(
        self,
        aolp: np.ndarray,
        saturation: Union[float, np.ndarray] = 1.0,
        value: Union[float, np.ndarray] = 1.0,
    ) -> np.ndarray:
        """Apply colormap to AoLP. The colormap is based on HSV.

        Parameters
        ----------
        AoLP : np.ndarray
            AoLP, its shape is (height, width). The range is from 0.0 to pi
        saturation : Union[float, np.ndarray], optional
            Saturation value(s), by default 1.0
        value : Union[float, np.ndarray], optional
            Value value(s), by default 1.0

        Returns
        -------
        aolp_colored : np.ndarray
            An applied colormap to AoLP, its shape is (height, width, 3) and dtype is `np.uint8`
        """
        ones = np.ones_like(aolp)

        hue = np.mod(aolp, np.pi) / np.pi * 179  # [0, pi] to [0, 179]
        saturation = np.clip(ones * saturation * 255, 0, 255)
        value = np.clip(ones * value * 255, 0, 255)

        hsv = cv2.merge([hue, saturation, value]).astype(np.uint8)
        aolp_colored = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        return aolp_colored

    def get_intensity_matrix(self):
        intensity_matrix = []

        for image_rgb, image_nir in self.image_list:
            image_rgb_np = image_rgb
            image_nir_np = np.expand_dims(image_nir, -1)
            image = np.concatenate(
                [
                    image_rgb_np,
                    image_nir_np,
                ],
                axis=2,
            )
            intensity_matrix.append(image)

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

    import os

    image_file_list = os.listdir("tmp_capture/polar_input")
    image_file_list.sort()

    QWP_ADJUST = 12.5

    qwp_degree_list = [
        float(x.split("channel_0_")[1].split(".")[0]) + QWP_ADJUST
        for x in image_file_list
        if "channel_0" in x
    ]

    pc = PolarizationCompute(mask_path="jai_binete.png")
    pc.update_qwp_angles(qwp_degree_list)
    pc.update_linear_matrix(0)
    pc.put_image_list(
        (-1, -1, 0), [f"tmp_capture/polar_input/{x}" for x in image_file_list]
    )

    pc.compute()

    os.makedirs("tmp_capture/polar_output", exist_ok=True)
    for key, value in pc.image_np_dict.items():
        cv2.imwrite(f"tmp_capture/polar_output/{key}.png", value)
