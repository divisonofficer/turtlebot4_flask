import { makeAutoObservable } from "mobx";
import { jaiSocket } from "../connect/socket/subscribe";
import { httpPost } from "../connect/http/request";

namespace JaiHDRLog {
  export interface ProgressRoot {
    idx: number;
    status: "ready" | "running" | "done" | "error";
    task: "rotate" | "hdr";
  }
  export interface ProgressSub {
    idx: number;
    type: "hdr" | "rotate";
  }
  export interface Log {
    progress_root: ProgressRoot;
    progress_sub: ProgressSub;
    hdr_error_msgs: Array<string>;
  }
}

class JaiHDRStore {
  hdr_log: JaiHDRLog.Log | undefined = undefined;
  constructor() {
    makeAutoObservable(this);

    jaiSocket.subscribe("hdr_log", (data: JaiHDRLog.Log) => {
      console.log("hdr_log", data);
      this.hdr_log = data;
    });
  }

  triggerHDR = () => {
    httpPost("/jai/stereo/hdr/trigger").fetch();
  };
}
export const jaiHDRStore = new JaiHDRStore();
