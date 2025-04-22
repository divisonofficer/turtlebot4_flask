import { makeAutoObservable } from "mobx";
import { httpGet, httpPost } from "../connect/http/request";

export namespace Sensor {
  export namespace Param {
    export interface Param {
      name: string;
      type: "int" | "float" | "bool" | "string" | "enum";
      hidden?: boolean;
    }
    export interface EnumParam extends Param {
      type: "enum";
      value: string;
      value_list: string[];
    }
    export interface IntParam extends Param {
      type: "int";
      value: number;
      min: number;
      max: number;
    }
    export interface FloatParam extends Param {
      type: "float";
      value: number;
      min: number;
      max: number;
    }
    export interface BoolParam extends Param {
      type: "bool";
      value: boolean;
    }
    export interface StringParam extends Param {
      type: "string";
      value: string;
    }
  }

  export interface Const {
    preview_enabled: boolean;
    preview_keys: string[];
  }

  export interface CameraConst extends Const {
    srcs: number;
    raw_format: "rgb" | "bayer" | "gray" | "depth";
    width: number;
    height: number;
  }

  export interface State {
    timestamp_last?: number;
    fps?: number;
    preview_on: boolean;
  }

  export interface Preview {
    frames: { [key: string]: string | undefined }; // key is string, value is base64 string
  }

  export interface CameraState extends State {
    stream_on: boolean;
  }

  export interface Sensor {
    name: string;
    const: Const;
    state: State;
    type: "camera" | "lidar";
    params?: Param.Param[];
  }

  export interface Camera extends Sensor {
    const: CameraConst;
    state: CameraState;
    camera_info: {};
  }
}

class SensorsStore {
  sensors: Sensor.Sensor[] = [
    // {
    //   name: "camera1",
    //   const: {
    //     srcs: 1,
    //     raw_format: "rgb",
    //     width: 1920,
    //     height: 1080,
    //     preview_enabled: true,
    //   },
    //   status: {
    //     timestamp_last: 0,
    //     fps: 0,
    //     stream_on: false,
    //     preview_on: false,
    //   },
    //   type: "camera",
    //   preview: {
    //     frames: {
    //       rgb: undefined,
    //     },
    //   },
    //   params: [
    //     {
    //       name: "exposure",
    //       type: "int",
    //       min: 0,
    //       max: 1000,
    //     } as Sensor.Param.IntParam,
    //     {
    //       name: "gain",
    //       type: "float",
    //       min: 0,
    //       max: 10,
    //     } as Sensor.Param.FloatParam,
    //     {
    //       name: "auto_exposure",
    //       type: "bool",
    //     } as Sensor.Param.BoolParam,
    //     {
    //       name: "auto_white_balance",
    //       type: "enum",
    //       value: "auto",
    //       value_list: ["auto", "manual", "sequential"],
    //     },
    //   ],
    // } as Sensor.Camera,
    // {
    //   name: "LiDAR1",
    //   type: "lidar",
    //   const: {
    //     preview_enabled: true,
    //   },
    //   status: {
    //     timestamp_last: 0,
    //     fps: 0,
    //     preview_on: false,
    //   },
    //   preview: {
    //     frames: {
    //       LiDAR: undefined,
    //     },
    //   },
    // },
  ];

  constructor() {
    makeAutoObservable(this);
  }

  _updateSensorList = (data: any) => {
    this.sensors = data as Array<Sensor.Sensor>;
  };

  fetchGetSensorsList = () => {
    httpGet("/lucid/sensors").onSuccess(this._updateSensorList).fetch();
  };

  fetchPostPreviewOn = (sensor: Sensor.Sensor) => {
    httpPost(`/lucid/sensors/${sensor.name}/preview/on`)
      .onSuccess(this._updateSensorList)
      .fetch();
  };
  fetchPostPreviewOff = (sensor: Sensor.Sensor) => {
    httpPost(`/lucid/sensors/${sensor.name}/preview/off`)
      .onSuccess(this._updateSensorList)
      .fetch();
  };
}

export const sensorsStore = new SensorsStore();
