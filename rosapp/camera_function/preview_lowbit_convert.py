import numpy as np


def preview_lowbit_convert(preview):

    WIDTH = 96
    HEIGHT = 48

    width = preview["width"]
    height = preview["height"]
    pixels = preview["pixels"]

    """
    convert preview pixels from Width x height x 3 to 32 x 32 x 3, by convolution
    """
    new_pixels = np.zeros((HEIGHT, WIDTH, 3))
    for i in range(HEIGHT):
        for j in range(WIDTH):
            new_pixels[i, j] = pixels[i * height // HEIGHT, j * width // WIDTH]

    """
    use single channer
    """
    single_channel = np.zeros((HEIGHT, WIDTH))
    for i in range(HEIGHT):
        for j in range(WIDTH):
            single_channel[i, j] = np.mean(new_pixels[i, j])
    return ascii_art_conversion(single_channel)


ascii_chars = '$@B%8&WM#*oahkbdpqwmZO0QLCJUYXzcvunxrjft/|()1[]?-_+~<>i!lI;:,"^`'


def ascii_art_conversion(pixels):
    """
    convert pixels to ascii art
    input : 2d numpy
    output : 2d numpy of ascii codes
    """
    return np.array(
        [
            [
                ascii_chars[int(pixels[i, j] / 256 * len(ascii_chars))]
                for j in range(pixels.shape[1])
            ]
            for i in range(pixels.shape[0])
        ]
    )
