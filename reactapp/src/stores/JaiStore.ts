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

  constructor() {
    makeAutoObservable(this);
    this.fetchJaiDeviceInfo();
    this.fetchJaiDeviceParam();

    jaiSocket.subscribeBuffer("calibration", StereoMatrix, (data) => {
      console.log(data);
      this.stereoMatrix = data;
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
    httpPost(`/jai/calibrate/storage/load/${idx}`, { idx: idx }).fetch();
  };

  fetchGetCalibrationList = () => {
    httpGet(`/jai/calibrate/storage/all`)
      .onSuccess((data: CalibrationMeta[]) => {
        this.calibrationList = data;
      })
      .fetch();
  };
}

export const jaiStore = new JaiStore();
