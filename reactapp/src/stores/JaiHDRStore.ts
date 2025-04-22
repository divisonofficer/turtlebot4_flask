import { makeAutoObservable } from "mobx";
import { jaiSocket } from "../connect/socket/subscribe";
import { httpGet, httpPost } from "../connect/http/request";
import { alertStore } from "./AlertStore";

export namespace JaiHDRLog {
  export interface ProgressRoot {
    idx: number;
    status: "ready" | "running" | "done" | "error" | "pause" | "abort";
    task: "rotate" | "hdr" | "ambient";
  }
  export interface ProgressSub {
    idx: number;
    type: "hdr" | "rotate";
  }
  export interface Error {
    type: string;
    data: any;
  }

  export interface Log {
    progress_root: ProgressRoot;
    progress_sub: ProgressSub;
    hdr_error_msgs: Array<Error>;
  }

  export interface Config {
    rotate_angle: number;
    capture_cnt: number;
    lidar: boolean;
    side_move_cnt: number;
    side_move_distance: number;
    drive_forward: boolean;
  }
}

class JaiHDRStore {
  hdr_log: JaiHDRLog.Log | undefined = undefined;
  hdr_config: JaiHDRLog.Config = {
    rotate_angle: 0,
    capture_cnt: 0,
    lidar: true,
    side_move_cnt: 0,
    side_move_distance: 0,
    drive_forward: false,
  };
  hdr_latest_capture: {
    space_id: string;
    frame_count: number;
    image?: string;
  } = {
    space_id: "",
    image: undefined,
    frame_count: 0,
  };
  constructor() {
    makeAutoObservable(this);

    jaiSocket.subscribe("hdr_log", (data: JaiHDRLog.Log) => {
      console.log("hdr_log", data);
      this.hdr_log = data;
    });
    jaiSocket.subscribe(
      "hdr/latest_capture",
      (data: { space_id: string; image: string; frame_count: number }) => {
        /**
       * "hdr/latest_capture",
            {
                "space_id": space_id,
                "image": cv2.imencode(".bmp", img_col_concat)[1].tobytes(),
            },
       * 
       */
        this.hdr_latest_capture = data;
      }
    );

    this.fetchGetConfig();
  }

  triggerHDR = () => {
    httpPost("/jai/stereo/hdr/trigger").fetch();
  };

  triggerPause = () => {
    httpPost("/jai/stereo/hdr/trigger/pause").fetch();
  };

  triggerResume = () => {
    httpPost("/jai/stereo/hdr/trigger/resume").fetch();
  };

  triggerStop = () => {
    httpPost("/jai/stereo/hdr/trigger/stop").fetch();
  };

  fetchGetConfig = () => {
    httpGet("/jai/stereo/hdr/config")
      .onSuccess((data) => {
        this.hdr_config = data;
      })
      .fetch();
  };

  fetchGetLatestCapture = () => {
    httpPost("/jai/stereo/hdr/emit_latest_capture").fetch();
  };

  fetchUpdateConfig = (key: string, value: any) => {
    const configUpdate = { [key]: value };
    httpPost("/jai/stereo/hdr/config", { config: configUpdate })
      .onSuccess((d) => {
        this.hdr_config = d;
      })
      .onError((c, m, e) => {
        alertStore.addAlert(
          "error",
          c || m || e.message,
          "Failed to update HDR config"
        );
      })
      .fetch();
  };
}
export const jaiHDRStore = new JaiHDRStore();
