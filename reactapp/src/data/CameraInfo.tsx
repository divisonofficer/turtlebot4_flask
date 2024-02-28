export type CameraInfo = {
    height: number;
    width: number;
    distortion_model: string;
    d: number[];
    k: number[];
    r: number[];
    p: number[];
    binning_x: number;
    binning_y: number;
}
