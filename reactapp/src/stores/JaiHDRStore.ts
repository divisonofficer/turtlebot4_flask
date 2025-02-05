import { makeAutoObservable } from "mobx";
import { jaiSocket } from "../connect/socket/subscribe";
import { httpGet, httpPost } from "../connect/http/request";
import { alertStore } from "./AlertStore";

export namespace JaiHDRLog {
  export interface ProgressRoot {
    idx: number;
    status: "ready" | "running" | "done" | "error";
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
  };
  constructor() {
    makeAutoObservable(this);

    jaiSocket.subscribe("hdr_log", (data: JaiHDRLog.Log) => {
      console.log("hdr_log", data);
      this.hdr_log = data;
    });

    this.fetchGetConfig();
  }

  triggerHDR = () => {
    httpPost("/jai/stereo/hdr/trigger").fetch();
  };

  fetchGetConfig = () => {
    httpGet("/jai/stereo/hdr/config")
      .onSuccess((data) => {
        this.hdr_config = data;
      })
      .fetch();
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
