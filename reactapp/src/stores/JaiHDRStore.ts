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

  export interface CaptureTiming {
    duration_ms: number;
    duration_sec: number;
    mode: string;
  }

  export interface Log {
    progress_root: ProgressRoot;
    progress_sub: ProgressSub;
    hdr_error_msgs: Array<Error>;
    capture_timing: CaptureTiming;
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
    use_piper: boolean;
    arc_forward: boolean;
    skip_jai: boolean;
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
    use_piper: false,
    arc_forward: false,
    skip_jai: false,
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
  frame_thumbnails: Map<string, string> = new Map();  // frameId → base64 image data
  scene_representative_thumbnails: Map<string, string> = new Map();  // sceneId → base64 image data
  current_hdr_space_id: string = "";
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
    this.fetchCurrentSpace();
    this.refreshStatus();  // Load initial HDR status
  }

  triggerHDR = (useNewSpace: boolean = true) => {
    const endpoint = useNewSpace
      ? "/jai/stereo/hdr/trigger"           // New space
      : "/jai/stereo/hdr/trigger/continue"; // Continue existing

    httpPost(endpoint).fetch();
  };

  fetchCurrentSpace = () => {
    httpGet("/jai/stereo/hdr/space/current")
      .onSuccess((data: { space_id: string; exists: boolean }) => {
        this.current_hdr_space_id = data.space_id || "";
      })
      .fetch();
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

  triggerForceStop = () => {
    httpPost("/jai/stereo/hdr/trigger/force_stop")
      .onSuccess(() => {
        // Force refresh status after force stop
        this.refreshStatus();
      })
      .fetch();
  };

  refreshStatus = () => {
    httpGet("/jai/stereo/hdr/status")
      .onSuccess((data: { status: string; is_running: boolean; progress: JaiHDRLog.Log }) => {
        this.hdr_log = data.progress;
      })
      .fetch();
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

        // Clear previous thumbnails when switching scenes
        this.frame_thumbnails.clear();

        // Don't preload all thumbnails - let lazy loading handle it
        // Only preload the first few for immediate display
        const preloadCount = Math.min(6, data.length); // Preload first 6 (2 per row)
        for (let i = 0; i < preloadCount; i++) {
          this.fetchFramePreview(sceneId, data[i]);
        }

        // Auto-select the first frame if available
        if (data.length > 0) {
          this.selectFrame(data[0]);
        }

        // Load representative thumbnail for this scene (middle frame)
        if (data.length > 0) {
          const middleIndex = Math.floor(data.length / 2);
          const representativeFrameId = data[middleIndex];
          this.fetchSceneRepresentativeThumbnail(sceneId, representativeFrameId);
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

  fetchSceneRepresentativeThumbnail = (sceneId: string, frameId: string) => {
    httpGet(`/jai/stereo/hdr/frame/${sceneId}/${frameId}/thumbnail`)
      .onSuccess((data: { thumbnail: string }) => {
        const imageData = `data:image/jpeg;base64,${data.thumbnail}`;
        this.scene_representative_thumbnails.set(sceneId, imageData);
      })
      .onError((_c, _m, e) => {
        console.error("Failed to fetch scene representative thumbnail:", e);
      })
      .fetch();
  };

  selectFrame = (frameId: string) => {
    this.selected_frame_id = frameId;

    // Load full quality image for main preview
    this.fetchFrameFull(this.current_scene_id, frameId);
  };

  fetchFramePreview = (sceneId: string, frameId: string) => {
    // Skip if already cached
    if (this.frame_thumbnails.has(frameId)) {
      return;
    }

    httpGet(`/jai/stereo/hdr/frame/${sceneId}/${frameId}/thumbnail`)
      .onSuccess((data: { thumbnail: string }) => {
        const imageData = `data:image/jpeg;base64,${data.thumbnail}`;

        // Cache the thumbnail
        this.frame_thumbnails.set(frameId, imageData);

        // If this is the selected frame, update main preview
        if (this.selected_frame_id === frameId) {
          this.hdr_latest_capture.image = imageData;
        }
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

  fetchFrameFull = (sceneId: string, frameId: string) => {
    httpGet(`/jai/stereo/hdr/frame/${sceneId}/${frameId}/full`)
      .onSuccess((data: { image: string }) => {
        const imageData = `data:image/jpeg;base64,${data.image}`;

        // Update main preview with full quality image
        this.hdr_latest_capture.image = imageData;
      })
      .onError((c, m, e) => {
        alertStore.addAlert(
          "error",
          c || m || e.message,
          "Failed to fetch full image"
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
