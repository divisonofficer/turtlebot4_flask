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
    drive_mode: "forward" | "side" | "arc";
    arc_radius: number;
    arc_angle: number;
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
    drive_mode: "forward",
    arc_radius: 0,
    arc_angle: 0,
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

  // HDR Storage related states
  hdr_scene_list: string[] = [];
  current_scene_id: string = "";
  current_scene_frames: string[] = [];
  selected_frame_id: string = "";
  frame_preview_image: string = "";
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
    this.fetchHDRSceneList();
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

  // HDR Storage methods
  fetchHDRSceneList = () => {
    httpGet("/jai/stereo/storage/list?root=tmp/stereo/hdr")
      .onSuccess((data: string[]) => {
        this.hdr_scene_list = data;
        // Auto-select the latest scene if available
        if (data.length > 0 && !this.current_scene_id) {
          this.current_scene_id = data[data.length - 1];
          this.fetchSceneFrames(this.current_scene_id);
        }
      })
      .onError((c, m, e) => {
        alertStore.addAlert(
          "error",
          c || m || e.message,
          "Failed to fetch HDR scene list"
        );
      })
      .fetch();
  };

  fetchSceneFrames = (sceneId: string) => {
    httpGet(`/jai/stereo/storage/${sceneId}/frames?root=tmp/stereo/hdr`)
      .onSuccess((data: string[]) => {
        this.current_scene_frames = data;
        this.current_scene_id = sceneId;
        // Auto-select the latest frame if available
        if (data.length > 0) {
          this.selectFrame(data[data.length - 1]);
        }
      })
      .onError((c, m, e) => {
        alertStore.addAlert(
          "error",
          c || m || e.message,
          "Failed to fetch scene frames"
        );
      })
      .fetch();
  };

  selectFrame = (frameId: string) => {
    this.selected_frame_id = frameId;
    this.fetchFramePreview(this.current_scene_id, frameId);
  };

  fetchFramePreview = (sceneId: string, frameId: string) => {
    // Use the new HDR thumbnail API
    httpGet(`/jai/stereo/hdr/frame/${sceneId}/${frameId}/thumbnail`)
      .onSuccess((data: { thumbnail: string }) => {
        this.frame_preview_image = `data:image/jpeg;base64,${data.thumbnail}`;
      })
      .onError((c, m, e) => {
        alertStore.addAlert(
          "error",
          c || m || e.message,
          "Failed to fetch frame preview"
        );
      })
      .fetch();
  };

  deleteFrame = (sceneId: string, frameId: string) => {
    fetch(`/jai/stereo/hdr/frame/${sceneId}/${frameId}`, {
      method: "DELETE",
    })
      .then((response) => {
        if (response.ok) {
          alertStore.addAlert(
            "success",
            "Frame deleted successfully",
            "Delete Frame"
          );
          // Refresh the frame list
          this.fetchSceneFrames(sceneId);
        } else {
          throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
      })
      .catch((error) => {
        alertStore.addAlert("error", error.message, "Failed to delete frame");
      });
  };
}
export const jaiHDRStore = new JaiHDRStore();
