import cv2
import numpy as np


class LucidPostProcess:
    def load_calibration(self, path: str):

        self.wb = [
            2.0841475322429894,
            1.0,
            1.9215893014341496,
        ]

        calibration = np.load(path)

        k_left = calibration["mtx_left"]
        k_right = calibration["mtx_right"]
        T = calibration["T"]
        R = calibration["R"]
        d_left = calibration["dist_left"]
        d_right = calibration["dist_right"]
        resolution = (1440, 926)
        rectify_left, rectify_right, proj_left, proj_right, Q, roi_left, roi_right = (
            cv2.stereoRectify(
                k_left,
                d_left,
                k_right,
                d_right,
                resolution,
                R,
                T,
            )
        )
        self.map_left_x, self.map_left_y = cv2.initUndistortRectifyMap(
            k_left,
            d_left,
            rectify_left,
            proj_left,
            resolution,
            cv2.CV_32FC1,
        )
        self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(
            k_right,
            d_right,
            rectify_right,
            proj_right,
            resolution,
            cv2.CV_32FC1,
        )

    def rectify_stereo(self, hdr_left, hdr_right):

        hdr_left_rectify = cv2.remap(
            hdr_left,
            self.map_left_x,
            self.map_left_y,
            cv2.INTER_LINEAR,
        )
        hdr_right_rectify = cv2.remap(
            hdr_right,
            self.map_right_x,
            self.map_right_y,
            cv2.INTER_LINEAR,
        )
        return hdr_left_rectify, hdr_right_rectify

    def rawUint8ToTonemappedBgr(self, raw: np.ndarray) -> np.ndarray:
        raw = self.rawUint8ToUint32(raw)
        bayer_img = self.bayerToBgr(raw)
        tonemapped_img = self.hdr_tonemap_to_8bit(bayer_img, self.tonemap)
        return tonemapped_img

    def rawUint8ToUint32(self, raw: np.ndarray) -> np.ndarray:
        raw = raw.astype(np.uint32)

        return raw[:, :, 0] + raw[:, :, 1] * 256 + raw[:, :, 2] * 256 * 256

    def bayerToBgr(self, bayer_img: np.ndarray) -> np.ndarray:
        height, width = bayer_img.shape

        bayer_img_16 = bayer_img & 0xFFFF0000
        bayer_img_16 = bayer_img_16 >> 16
        bayer_img_0 = bayer_img & 0x0000FFFF
        bayer_img_16 = bayer_img_16.astype(np.uint16)
        bayer_img_0 = bayer_img_0.astype(np.uint16)

        rgb_img = cv2.cvtColor(bayer_img_16, cv2.COLOR_BAYER_RGGB2BGR).astype(
            np.float32
        ) / (256)
        rgb_img_0 = cv2.cvtColor(bayer_img_0, cv2.COLOR_BAYER_RGGB2BGR).astype(
            np.float32
        ) / (65536 * 256)

        rgb_img = rgb_img + rgb_img_0
        # Apply white balance
        rgb_img[:, :, 0] *= self.wb[0]
        rgb_img[:, :, 1] *= self.wb[1]
        rgb_img[:, :, 2] *= self.wb[2]
        return rgb_img

        # Initialize the BGR channels
        red_channel = np.zeros((height, width), dtype=np.float32)
        green_channel = np.zeros((height, width), dtype=np.float32)
        blue_channel = np.zeros((height, width), dtype=np.float32)

        # Interpolate R, G, B channels
        # Red channel
        red_channel[0:height:2, 0:width:2] = bayer_img[0:height:2, 0:width:2]
        red_channel[1 : height - 1 : 2, 0:width:2] = (
            bayer_img[0 : height - 2 : 2, 0:width:2] + bayer_img[2:height:2, 0:width:2]
        ) / 2.0
        red_channel[0:height:2, 1 : width - 1 : 2] = (
            bayer_img[0:height:2, 0 : width - 2 : 2] + bayer_img[0:height:2, 2:width:2]
        ) / 2.0
        red_channel[1 : height - 1 : 2, 1 : width - 1 : 2] = (
            bayer_img[0 : height - 2 : 2, 0 : width - 2 : 2]
            + bayer_img[0 : height - 2 : 2, 2:width:2]
            + bayer_img[2:height:2, 0 : width - 2 : 2]
            + bayer_img[2:height:2, 2:width:2]
        ) / 4.0

        # Green channel
        green_channel[0:height:2, 1:width:2] = bayer_img[0:height:2, 1:width:2]
        green_channel[1:height:2, 0:width:2] = bayer_img[1:height:2, 0:width:2]
        green_channel[0 : height - 2 : 2, 0 : width - 2 : 2] = (
            bayer_img[0 : height - 2 : 2, 1 : width - 1 : 2]
            + bayer_img[1 : height - 1 : 2, 0 : width - 2 : 2]
        ) / 2.0
        green_channel[1 : height - 1 : 2, 1 : width - 1 : 2] = (
            bayer_img[1 : height - 2 : 2, 0 : width - 2 : 2]
            + bayer_img[0 : height - 3 : 2, 1 : width - 1 : 2]
            + bayer_img[1 : height - 1 : 2, 2:width:2]
            + bayer_img[2:height:2, 1 : width - 1 : 2]
        ) / 4.0

        # Blue channel
        blue_channel[1:height:2, 1:width:2] = bayer_img[1:height:2, 1:width:2]
        blue_channel[0 : height - 2 : 2, 1:width:2] = (
            bayer_img[0 : height - 2 : 2, 1:width:2] + bayer_img[2:height:2, 1:width:2]
        ) / 2.0
        blue_channel[1:height:2, 0 : width - 2 : 2] = (
            bayer_img[1:height:2, 0 : width - 2 : 2] + bayer_img[1:height:2, 2:width:2]
        ) / 2.0
        blue_channel[0 : height - 2 : 2, 0 : width - 2 : 2] = (
            bayer_img[0 : height - 2 : 2, 0 : width - 2 : 2]
            + bayer_img[0 : height - 2 : 2, 2:width:2]
            + bayer_img[2:height:2, 0 : width - 2 : 2]
            + bayer_img[2:height:2, 2:width:2]
        ) / 4.0

        # Merge the channels into a BGR image
        bgr_image = np.stack((blue_channel, green_channel, red_channel), axis=-1)
        bgr_image = (bgr_image / (1 << 24)).astype(np.float32)
        return bgr_image

    def hdr_tonemap_to_8bit(self, hdr_image: np.ndarray, tonemap):
        # Apply tonemap

        ldr_image = tonemap.process(hdr_image)
        # Convert to 8-bit image
        ldr_image[np.isnan(ldr_image)] = 0
        ldr_image_8bit = np.clip(ldr_image * 255, 0, 255).astype(np.uint8)

        return ldr_image_8bit

    def __init__(self):
        self.tonemap = cv2.createTonemap(gamma=2.2)
        self.tonemap_drago = cv2.createTonemapDrago(gamma=2.2)
        self.tonemap_mantiuk = cv2.createTonemapMantiuk(gamma=2.2)
        self.tonemap_reinhard = cv2.createTonemapReinhard(gamma=2.2)


if __name__ == "__main__":
    lucid_post_process = LucidPostProcess()
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=str, required=True)

    args = parser.parse_args()

    input_path = args.input

    raw = np.load(input_path)["left"]
    bayer_img = lucid_post_process.rawUint8ToUint32(raw)
    bgr_img = lucid_post_process.bayerToBgr(bayer_img)

    for tonemap in [
        lucid_post_process.tonemap,
        lucid_post_process.tonemap_drago,
        lucid_post_process.tonemap_mantiuk,
        lucid_post_process.tonemap_reinhard,
    ]:
        tonemapped_img = lucid_post_process.hdr_tonemap_to_8bit(bgr_img, tonemap)
        name = f"{tonemap}".split(" 0x")[0].split("cv2.")[1]
        cv2.imwrite(f"{name}.png", tonemapped_img)
