import { action, makeAutoObservable } from "mobx";
import { httpGet, httpPost } from "../connect/http/request";
import { DeviceInfo, ParameterValue } from "../public/proto/jai";

export interface JAIParamValue {
  type: string;
  value: number;
}

export interface JAIParam {
  ExposureTime: JAIParamValue;
  Gain: JAIParamValue;
}

class JaiStore {
  jaiCameraParams: {
    [key: string]: {
      [key: string]: ParameterValue;
    }[];
  } = {};
  jaiDeviceInfo: DeviceInfo[] = [];

  constructor() {
    makeAutoObservable(this);
    this.fetchJaiDeviceInfo();
    this.fetchJaiDeviceParam();
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
}

export const jaiStore = new JaiStore();
