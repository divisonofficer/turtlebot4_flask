from re import M
import sys
from typing import Annotated, Any, Union, TypeVar

sys.path.append("../../../public/proto/python")

import numpy as np
import cv2
from pypolar import mueller
import polanalyser as pa
from nptyping import NDArray, Shape, DType, Number
import os


#
IMAGE_SHAPE = NDArray[Shape["*,*,3"], Number]
"""
MatLike Image Matrix
"""
INTENSITY_SHAPE = NDArray[Shape["*,*,*,4"], Number]
"""
A List of Captured MatLike Image Matrix
"""
STOKES_COLOR_4 = NDArray[Shape["4,*,*,3"], Number]
"""
Each pixel of the image is represented by a 4 x 4 matrix, which is the stokes vectors for each color channels\n
(Stokes_Channel, Width, Height, Color_Channel)
"""
MM_TYPE = NDArray[Shape["4,4"], Number]
"""
4 x 4 Muller Matrix
"""
MM_STACKED_TYPE = NDArray[Shape["*,4"], Number]
"""
Stacked matrix of first row of 4 x 4 Muller Matrix
"""


class PolarizationCompute:
    mm_QWP: list[MM_TYPE] = []
    mm_Linear: MM_TYPE = np.array([])

    capture_meta = (-1, -1, -1)
    linear_angle = -1.0
    qwp_angle = -1.0
    image_np_dict: dict[str, IMAGE_SHAPE] = {}
    image_np_gt_dict: dict[str, IMAGE_SHAPE] = {}
    image_list: list[tuple[IMAGE_SHAPE, IMAGE_SHAPE]] = []
    image_scale = 0.8

    loss_values: dict[str, Any] = {}

    def __init__(self, mask_path: str = "instance/capture/jai_binete.png"):
        self.image_jai_mask: np.ndarray = cv2.imread(mask_path)
        self.image_jai_mask_gray: np.ndarray = cv2.cvtColor(
            self.image_jai_mask, cv2.COLOR_BGR2GRAY
        )
        pass

    def prepare_scene(self, linear_angle: float, qwp_angle: float):
        if self.linear_angle == linear_angle and self.qwp_angle == qwp_angle:
            return False

        self.linear_angle = linear_angle

        self.qwp_angle = qwp_angle
        return True

    def put_image_list(self, capture_meta: tuple[int, int, int], image_list: list[str]):
        """
        Read Image Files from Storage and put them into the class
        To avoid unnecessary duplicated computation,
        the function will return False if the capture_meta is the same as the previous one

        Input
        ----------
        capture_meta : tuple[space_id, capture_id, scene_id]
        image_list : list[image_file_path: str]

        """
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

    def denoise_16bit_image(self, image: np.ndarray):
        image_head = image / np.uint16(256)  # type: ignore
        isSingleChannel = len(image_head.shape) == 2 or image_head.shape[2] == 1
        if isSingleChannel:
            image_head = cv2.fastNlMeansDenoising(
                image_head.astype(np.uint8), None, 5, 7, 21
            ).astype(np.uint16)
        else:
            image_head = cv2.fastNlMeansDenoisingColored(
                image_head.astype(np.uint8), None, 5, 5, 7, 21
            ).astype(np.uint16)
        image = image_head * 256 + image % 256  # type: ignore

    def cut_image(self, image: np.ndarray, scale: float):
        w, h = image.shape[:2]
        w_from = w / 2 - w * scale / 2
        w_to = w / 2 + w * scale / 2
        h_from = h / 2 - h * scale / 2
        h_to = h / 2 + h * scale / 2
        image = image[int(w_from) : int(w_to), int(h_from) : int(h_to)]
        return image

    def read_image(self, image_path: str, format: str = "rgb"):
        """
        Read image file and return the image matrix
        Adjust the image matrix by applying denoising and masking

        Input
        ----------
        image_path : str
        format : ["rgb", "nir"]

        Output
        ----------
        IMAGE_SHAPE : (WIDTH, HEIGHT, COLOR_CHANNEL)

        """

        if os.path.exists(image_path.replace("png", "np.npy")):
            image: np.ndarray = np.load(image_path.replace("png", "np.npy"))
        else:
            image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

        image = self.cut_image(image, self.image_scale)

        bitwise_mask = (
            self.image_jai_mask_gray
            if len(image.shape) == 2 or image.shape[2] == 1
            else self.image_jai_mask
        )
        bitwise_mask = cv2.resize(
            bitwise_mask,
            (image.shape[1], image.shape[0]),
            interpolation=cv2.INTER_NEAREST,
        )

        if image.dtype == np.uint16:
            bitwise_mask = bitwise_mask.astype(np.uint16)
            bitwise_mask[bitwise_mask == 0] = 0
            bitwise_mask[bitwise_mask > 0] = 65535

        image = cv2.bitwise_and(image, bitwise_mask)

        # image = self.denoise_16bit_image(image)

        if format == "nir" and len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        image = cv2.medianBlur(image, 3)

        if image.dtype == np.uint16:
            image = image.astype(np.float32) / 256.0
        else:
            image = image.astype(np.float32)

        return image

    def update_qwp_angles(self, angles: list[float]):
        """
        Prepare the QWP Muller Matrix for each angle

        Input
        ----------
        angles : list[float] - QWP angles in degrees
        """
        self.mm_QWP = [
            self.muller_matrix_qwp(np.pi * angles[i] / 180) for i in range(len(angles))
        ]
        return self.mm_QWP

    def get_image_by_property(self, property: str):
        """
        Get rendered presentation image by property

        Input
        ----------
        property : str - property name, ex) s0_rgb_positive, r_aolp, nir_dolp
        """
        return self.image_np_dict[property] if property in self.image_np_dict else None

    def get_color_by_property(self, property: str, x: float, y: float):
        """
        Get color value of the pixel by property

        Input
        ----------
        property : str - property name, ex) s0_rgb_positive, r_aolp, nir_dolp
        x : int - x coordinate of the pixel
        y : int - y coordinate of the pixel
        """
        image = self.get_image_by_property(property)
        if image is None:
            return None
        x = int(image.shape[1] * x)
        y = int(image.shape[0] * y)
        color = image[y, x]
        color_dict = {}
        if len(image.shape) == 2:
            color_dict["gray"] = color
        else:
            color_dict["b"] = color[0]
            color_dict["g"] = color[1]
            color_dict["r"] = color[2]

        if property in self.image_np_gt_dict:
            color_gt = self.image_np_gt_dict[property][y, x]
            color_dict["value"] = color_gt

        color_dict = {
            k: float(v) if np.isfinite(v) else 0 for k, v in color_dict.items()
        }

        return color_dict

    def compute(self):
        I = self.get_intensity_matrix(
            self.image_list
        )  # (INPUT_CHANNEL, WIDTH, HEIGHT, COLOR_CHANNEL = 4)
        S = self.calcStokesArray(
            I, self.get_stacked_mm()
        )  # (STOKES_CHANNEL = 4, WIDTH, HEIGHT, COLOR_CHANNEL = 4)

        self.visualize(S)

    def visualize(self, S: np.ndarray):
        for i, s in enumerate(["s0", "s1", "s2", "s3"]):
            for channel in ["rgb", "nir"]:
                self.compute_vis_stokes(
                    channel=channel,
                    stokes_property=s,
                    stokes_matrix=(
                        S[i, :, :, 0:3] if channel == "rgb" else S[i, :, :, 3:]
                    ),
                )
            for c, channel in enumerate(["b", "g", "r", "nir"]):
                if s == "s0":
                    self.image_np_dict[f"{channel}_{s}_combined"] = S[i, :, :, c]
                else:
                    self.image_np_dict[f"{channel}_{s}_combined"] = pa.applyColorMap(
                        S[i, :, :, c], "bwr", -255, 255
                    )
        for i, channel in enumerate(["b", "g", "r", "nir"]):
            self.compute_vis_polarization(
                channel=channel, stokes_matrix=S[:, :, :, i : i + 1]
            )  # (STOKES_CHANNEL = 4, WIDTH, HEIGHT, COLOR_CHANNEL = 1)

    def get_intensity_matrix(
        self, image_list: list[tuple[IMAGE_SHAPE, IMAGE_SHAPE]]
    ) -> INTENSITY_SHAPE:
        """
        Get the intensity matrix from the image list
        Merge RGB, NIR images into one matrix

        Output
        ----------
        INTENSITY_SHAPE : (INPUT_CHANNEL, WIDTH, HEIGHT, COLOR_CHANNEL = 4)
        """
        intensity_matrix = []

        for image_rgb, image_nir in image_list:
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
        return np.stack(intensity_matrix, axis=0).squeeze()

    def calcStokes(
        self, intensity: INTENSITY_SHAPE, MM_Stacked: MM_STACKED_TYPE
    ) -> STOKES_COLOR_4:
        """
        Input
        ----------
        INTENSITY_SHAPE : (INPUT_CHANNEL, WIDTH, HEIGHT, COLOR_CHANNEL = 1)

        Output
        ----------
        STOKES_COLOR_4 : (STOKES_CHANNEL = 4, WIDTH, HEIGHT, COLOR_CHANNEL = 1)
        """
        # Get the MM_Stacked. (MM_COUNT, 4, 4)[0] -> (MM_COUNT, 4)

        stokes = np.einsum("ij,j...->i...", np.linalg.pinv(MM_Stacked), intensity)  # type: ignore

        return self.denoise_stokes(stokes)

    def calcStokesArray(self, I: INTENSITY_SHAPE, MM: list[MM_TYPE]) -> STOKES_COLOR_4:
        """
        Input
        ----------
        (INPUT_CHANNEL, WIDTH, HEIGHT, COLOR_CHANNEL = 4)

        Output
        ----------
        STOKES_COLOR_4 : (STOKES_CHANNEL = 4, WIDTH, HEIGHT, COLOR_CHANNEL = 4)
        """
        MM_Stacked: MM_STACKED_TYPE = np.stack(
            [MM[i][0] for i in range(len(MM))], axis=0
        )
        S = np.stack(
            [
                self.calcStokes(I[:, :, :, i], MM_Stacked)
                # pa.calcStokes(I[:, :, :, i], self.get_stacked_mm())  # type: ignore
                for i in range(I.shape[-1])
            ],
            axis=3,
        )
        return S

    def denoise_stokes(self, stokes: np.ndarray):

        return stokes

    def denoise_stokes_multichannel(self, stokes: np.ndarray):

        return stokes

    def get_stacked_mm(self) -> list[MM_TYPE]:
        np_MM_QWP = np.array(self.mm_QWP)
        np_MM_L = np.array(self.mm_Linear)
        np_MM = [
            np.dot(
                np_MM_L,
                np_MM_QWP[i],
            )
            for i in range(len(self.mm_QWP))
        ]
        return np_MM

    def compute_vis_stokes(
        self, channel: str, stokes_property: str, stokes_matrix: STOKES_COLOR_4
    ):
        """
        channel: rgb, nir
        stokes_property: s0, s1, s2, s3
        stokes_matrix: (WIDTH, HEIGHT, COLOR_CHANNEL)
        """
        max_val = max(
            max(np.max(stokes_matrix), -np.min(stokes_matrix)), np.float64(255)
        )

        stokes_matrix *= np.float64(255) / max_val  # type: ignore

        self.image_np_dict[f"{channel}_{stokes_property}_positive"] = np.clip(
            stokes_matrix, 0, 255
        ).astype(np.uint8)
        self.image_np_dict[f"{channel}_{stokes_property}_negative"] = np.clip(
            -stokes_matrix, 0, 255
        ).astype(np.uint8)

        if stokes_property == "s0":
            negative_pixel_count = np.sum(stokes_matrix < 0)
            total_pixel_count = stokes_matrix.size
            self.loss_values[f"{channel}_s0_error"] = (
                negative_pixel_count / total_pixel_count
            )

    def compute_vis_polarization(self, channel, stokes_matrix: STOKES_COLOR_4):
        """
        channel : b, g, r, nir
        stokes_matrix : (STOKES_CHANNEL = 4, HEIGHT,WIDTH, COLOR_CHANNEL = 1)
        """
        stokes_images_p, stokes_images_gt = self.get_vis_aolp_dolp(stokes_matrix)
        mask = cv2.resize(
            self.image_jai_mask, (stokes_matrix.shape[2], stokes_matrix.shape[1])
        )
        for key, value in stokes_images_p.items():
            self.image_np_dict[f"{channel}_{key}"] = cv2.bitwise_and(value, mask)  # type: ignore
        for key, value in stokes_images_gt.items():
            self.image_np_gt_dict[f"{channel}_{key}"] = value

    def get_vis_aolp_dolp(
        self, img_stokes: STOKES_COLOR_4
    ) -> tuple[dict[str, IMAGE_SHAPE], dict[str, IMAGE_SHAPE]]:

        img_stokes = np.moveaxis(img_stokes, 0, -1)[:, :, 0, :]
        img_aolp = pa.cvtStokesToAoLP(img_stokes)
        img_dolp = self.cvtStokesToDoLP(img_stokes)
        img_dolp[np.isnan(img_dolp)] = 0
        img_aolp_vis = self.applyColorToAoLP(img_aolp, img_dolp).astype(np.uint8)

        img_specular = pa.cvtStokesToSpecular(img_stokes)
        img_dop = pa.cvtStokesToDoP(img_stokes)
        img_docp = pa.cvtStokesToDoCP(img_stokes)
        img_diffuse = pa.cvtStokesToDiffuse(img_stokes)
        img_ell_angle = pa.cvtStokesToEllipticityAngle(img_stokes)

        img_dolp_vis = pa.applyColorMap(img_dolp, "gist_heat", 0, 1)

        return {
            "dolp": img_dolp_vis,
            "aolp": img_aolp_vis,
            "specular": pa.applyColorMap(img_specular, "bwr", 0, 256),
            "dop": pa.applyColorMap(img_dop, "gist_heat", 0, 1),
            "docp": pa.applyColorMap(img_docp, "gist_heat", 0, 1),
            "diffuse": pa.applyColorMap(img_diffuse, "bwr", -128, 128),
            "ell_angle": pa.applyColorMap(img_ell_angle, "bwr", -np.pi / 4, np.pi / 4),
        }, {
            "dolp": img_dolp,
            "aolp": img_aolp,
            "specular": img_specular,
            "dop": img_dop,
            "docp": img_docp,
            "diffuse": img_diffuse,
            "ell_angle": img_ell_angle,
        }

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

    def muller_matrix_qwp(self, angle: float) -> MM_TYPE:
        return np.array(mueller.op_quarter_wave_plate(angle)).round(7)

    def muller_matrix_linear(self, angle: float) -> MM_TYPE:
        return np.array(mueller.op_linear_polarizer(angle)).round(7)

    def update_linear_matrix(self, angle: float):
        self.mm_Linear = self.muller_matrix_linear(angle)
        return self.mm_Linear


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
