import cv2
import numpy as np


class LucidPostProcess:

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
        bgr_image = (bgr_image / (2 << 24)).astype(np.float32)
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
