import { action, makeAutoObservable } from "mobx";
import { httpDel, httpGet, httpPost } from "../connect/http/request";
import { DeviceInfo, ParameterValue, StereoMatrix } from "../public/proto/jai";
import { jaiSocket } from "../connect/socket/subscribe";

export interface JAIParamValue {
  type: string;
  value: number;
}

export interface JAIParam {
  ExposureTime: JAIParamValue;
  Gain: JAIParamValue;
}

export interface CalibrationMeta {
  id: number;
  timestamp: number;
  month: number;
  day: number;
  hour: number;
  shape: number[];
}

export interface ChessboardShape {
  rows: number;
  cols: number;
  length: number;
}

export interface JaiStereoQueueStatus {
  left_count: number;
  left_last_time: number | undefined;
  right_count: number;
  right_last_time: number | undefined;
  merge_interval: number;
  left_last_delay: number;
  right_last_delay: number;
  diff: number;
}
export interface JaiStereoStatus {
  queue_status: {
    merged: JaiStereoQueueStatus;
    nir: JaiStereoQueueStatus;
    viz: JaiStereoQueueStatus;
  };
  storage_id: string | undefined;
  storage_status: {
    stored_frame_cnt: number;
    frame_rate: number;
    queue_length: number;
    item_store_time: number;
  };
  option_status: {
    single_frame_mode: boolean;
    reset_on_lidar_error: boolean;
    lidar_collect: boolean;
    oakd_collect: boolean;
  };
}

class JaiStore {
  jaiCameraParams: {
    [key: string]: {
      [key: string]: ParameterValue;
    }[];
  } = {};
  jaiDeviceInfo: DeviceInfo[] = [];
  stereoMatrix: StereoMatrix | undefined = undefined;

  calibrationList: CalibrationMeta[] = [];
  chessboardShape: ChessboardShape = {
    rows: 0,
    cols: 0,
    length: 0,
  };

  lidar_transform: number[][] = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
  ];

  stereo_status: JaiStereoStatus | undefined = undefined;

  constructor() {
    makeAutoObservable(this);
    this.fetchJaiDeviceInfo();
    this.fetchJaiDeviceParam();

    jaiSocket.subscribeBuffer("calibration", StereoMatrix, (data) => {
      console.log(data);
      this.stereoMatrix = data;
    });
    jaiSocket.subscribe("stereo_status", (data: JaiStereoStatus) => {
      this.stereo_status = data;
    });
  }
  @action
  fetchJaiDeviceInfo = () => {
    httpGet("/jai/device")
      .onSuccess((data) => {
        this.jaiDeviceInfo = data.map((device: any) =>
          DeviceInfo.fromJSON(device)
        );
      })
      .fetch();
  };
  @action
  fetchJaiDeviceParam = () => {
    httpGet("/jai/status")
      .onSuccess((data) => {
        this.jaiCameraParams = data;
      })
      .fetch();
  };

  @action
  fetchLidarTransform = () => {
    httpGet("/jai/calibrate/lidar/transform")
      .onSuccess((data) => {
        if (data instanceof Array && data.length === 3 && data[0].length === 4)
          this.lidar_transform = data;
      })
      .fetch();
  };

  @action
  updateLidarTransform = (transform: number[][]) => {
    httpPost("/jai/calibrate/lidar/transform", {
      transform: transform,
    })
      .onSuccess(() => {
        this.fetchLidarTransform();
      })
      .fetch();
  };

  fetchJaiCameraHoldAutoExposure(device: string, hold: boolean) {
    httpPost(`/jai/device/${device}/auto_exposure_hold`, {
      hold: hold,
    })
      .onSuccess((data) => {
        this.jaiCameraParams = data;
      })
      .fetch();
  }

  fetchJaiCameraHoldAutoExposureAll(hold: boolean) {
    httpPost(`/jai/all/auto_exposure_hold`, {
      hold: hold,
    })
      .onSuccess((data) => {
        this.jaiCameraParams = data;
      })
      .fetch();
  }

  fetchJaiCameraStreamControl = (deviceName: string, open: boolean) => {
    httpPost(
      `/jai/device/${deviceName}/${open ? "open_stream" : "close_stream"}`
    ).fetch();
  };

  fetchJaiParameterSave = () => {
    httpPost(`/jai/device/update/all`).fetch();
  };

  fetchJaiCameraUpdateParam = (
    device_name: string,
    source: number,
    name: string,
    type: string,
    value: any
  ) => {
    httpPost(`/jai/device/${device_name}/${source}/configure`, {
      key: name,
      type: type,
      value: value,
    })
      .onSuccess(() => {
        setTimeout(() => {
          this.fetchJaiDeviceParam();
        }, 500);
      })
      .fetch();
  };

  downloadCalibrationJson = () => {
    console.log(this.stereoMatrix);
    const jsonFile = JSON.stringify(this.stereoMatrix);
    const element = document.createElement("a");
    const file = new Blob([jsonFile], { type: "application/json" });
    element.href = URL.createObjectURL(file);
    element.download = "calibration.json";
    console.log(element);
    element.click();
    element.remove();
  };

  deleteCalibrationImage = (idx: number) => {
    httpDel(`/jai/calibrate/chessboard/${idx}`).fetch();
  };

  calibrationCapture = () => {
    httpPost(`/jai/calibrate/capture`).fetch();
  };

  fetchSubscribeJaiStream = () => {
    httpPost(`/jai/subscription/video_stream/start`).fetch();
  };

  fetchUnsubscribeJaiStream = () => {
    httpPost(`/jai/subscription/video_stream/stop`).fetch();
  };

  fetchSubscribeJaiCalibration = () => {
    httpPost(`/jai/calibrate/start`).fetch();
    httpGet(`/jai/calibrate/refresh`).fetch();
  };
  fetchUnsubscribeJaiCalibration = () => {
    httpPost(`/jai/calibrate/stop`).fetch();
  };

  fetchCalibrationSave = () => {
    httpPost(`/jai/calibrate/storage/save`)
      .onSuccess((data: { idx: number }) => {})
      .fetch();
  };

  fetchCalibrationLoad = (idx: number) => {
    httpPost(`/jai/calibrate/storage/load/${idx}`, { idx: idx })
      .onSuccess(
        (data: {
          shape: {
            rows: number;
            cols: number;
            length: number;
          };
        }) => {
          this.chessboardShape = data.shape;
        }
      )
      .fetch();
  };

  fetchUpdateCalibrationChessboard = (shape: ChessboardShape) => {
    httpPost(`/jai/calibrate/chessboard/shape`, shape)
      .onSuccess((data) => {
        this.chessboardShape = data;
      })
      .fetch();
  };

  fetchGetCalibrationList = () => {
    httpGet(`/jai/calibrate/storage/all`)
      .onSuccess((data: CalibrationMeta[]) => {
        this.calibrationList = data;
      })
      .fetch();
  };

  fetchDepthSetCurrentCalibration = (id: number) => {
    httpPost(`/jai/stereo/calibration/load`, { calibration_id: id }).fetch();
  };

  fetchGetStereoNodeStatus = () => {
    httpGet(`/jai/stereo`)
      .onSuccess((data: JaiStereoStatus) => {
        this.stereo_status = data;
      })
      .fetch();
  };

  fetchEnableStereoStorage = () => {
    httpPost(`/jai/stereo/storage/enable`, {})
      .onSuccess((data: JaiStereoStatus) => {
        this.stereo_status = data;
      })
      .fetch();
  };

  fetchUpdateStereoOption = (option: string, value: any) => {
    httpPost(`/jai/stereo/option/${option}`, { value: value })
      .onSuccess((data: JaiStereoStatus) => {
        this.stereo_status = data;
      })
      .fetch();
  };

  fetchDisableStereoStorage = () => {
    httpPost(`/jai/stereo/storage/disable`)
      .onSuccess((data: JaiStereoStatus) => {
        this.stereo_status = data;
      })
      .fetch();
  };
}

export const jaiStore = new JaiStore();
