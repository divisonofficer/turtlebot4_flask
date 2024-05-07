import { action, makeAutoObservable } from "mobx";
import { httpGet, httpPost } from "../connect/http/request";

export interface JAIParamValue {
  type: string;
  value: number;
}

export interface JAIParam {
  ExposureTime: JAIParamValue;
  Gain: JAIParamValue;
}

export interface JAIDeviceInfo {
  name: string;
  source_count: number;
  source_types: string[];
  configurable: {
    name: string;
    type: string;
    min: number;
    max: number;
  }[];
}

class JaiStore {
  jaiCameraParams: { [key: string]: JAIParam[] } = {};
  jaiDeviceInfo: JAIDeviceInfo[] = [];

  constructor() {
    makeAutoObservable(this);
    this.fetchJaiDeviceInfo();
    this.fetchJaiDeviceParam();
  }
  @action
  fetchJaiDeviceInfo = () => {
    httpGet("/jai/device")
      .onSuccess((data) => {
        this.jaiDeviceInfo = data;
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
