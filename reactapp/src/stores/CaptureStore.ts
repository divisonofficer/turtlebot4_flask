import { action, makeAutoObservable, runInAction } from "mobx";
import { httpGet, httpPost } from "../connect/http/request";
import { alertStore } from "./AlertStore";
import { captureSocket } from "../connect/socket/subscribe";
import {
  CaptureAppCapture,
  CaptureAppScene,
  CaptureAppSpace,
  CaptureMessageDefGroup,
  CaptureTaskProgress,
  CaptureTaskProgress_Action,
} from "../public/proto/capture";

export const CAPTURE_TOPICS = {
  OAKD_MONO: {
    name: "OAKD Mono Preview",
    topic: "/oakd/rgb/preview/image_raw",
    default_checked: true,
  },
  OAKD_DEPTH: {
    name: "OAKD Depth",
    topic: "/stereo/depth",
    default_checked: false,
  },
  JAI_RGB: {
    name: "JAI RGB",
    topic: "/jai_1600/channel_0",
    default_checked: false,
  },
  JAI_NIR: {
    name: "JAI NIR",
    topic: "/jai_1600/channel_1",
    default_checked: false,
  },
};

class CaptureStore {
  constructor() {
    makeAutoObservable(this);

    captureSocket.subscribeBuffer(
      "/recent_scene",
      CaptureAppScene,
      (data: CaptureAppScene) => {
        this.extendSceneId(data);
      }
    );
    captureSocket.subscribeBuffer(
      "/progress",
      CaptureTaskProgress,
      (data: CaptureTaskProgress) => this.processCaptureProgress(data)
    );
  }
  /////////////////////////////////////////////
  // For All Backend Stored data
  /////////////////////////////////////////////
  captureSpaceList: CaptureAppSpace[] = [];
  /////////////////////////////////////////////
  // For Single Space data for viewing
  /////////////////////////////////////////////
  captureScenes: CaptureAppScene[] = [];
  captures: CaptureAppCapture[] = [];
  capture_pendings_id: number[] = [];

  map_name?: string = undefined;

  space_id?: number = undefined;
  space_name?: string = undefined;

  map_focused_capture_id: number = -1;
  map_focused_scene_id: number = -1;

  use_slam: boolean = false;

  is_capture_running: boolean = false;
  progress: CaptureTaskProgress[] = [];
  /////////////////////////////////////////////
  // For Capture Control
  /////////////////////////////////////////////
  def_switchs: { [key: string]: CaptureMessageDefGroup } = {};
  /////////////////////////////////////////////
  // Capture Control Action
  /////////////////////////////////////////////
  @action
  extendSceneId(scene_meta: CaptureAppScene) {
    setTimeout(
      () =>
        httpGet(
          `/capture/result/${scene_meta.spaceId}/${scene_meta.captureId}/${scene_meta.sceneId}/images`
        )
          .onSuccess((images: string[]) => {
            scene_meta.images = images;
            this.extendScene(scene_meta);
          })
          .fetch(),
      3000
    );
  }

  @action
  processCaptureProgress(progress: CaptureTaskProgress) {
    console.log(this.progress, progress);
    if (this.progress.find((p) => p.uid === progress.uid)) {
      if (progress.action === CaptureTaskProgress_Action.DONE) {
        this.progress = this.progress.filter((p) => p.uid !== progress.uid);
        return;
      }
      if (progress.action === CaptureTaskProgress_Action.ERROR) {
        this.progress = this.progress.filter((p) => p.uid !== progress.uid);
        alertStore.addAlert(
          "error",
          `Capture Task Error : ${progress.message}`,
          "CaptureStore.processCaptureProgress"
        );
        this.capture_pendings_id = this.capture_pendings_id.filter(
          (id) => id !== progress.captureId
        );
        return;
      }
      this.progress = this.progress.map((p) => {
        if (p.uid === progress.uid) {
          return progress;
        }
        return p;
      });
      return;
    }
    this.progress = [progress, ...this.progress];
  }

  @action
  extendScene(scene: CaptureAppScene) {
    this.captureScenes = [scene, ...this.captureScenes];

    // find capture and put scene
    const capture = this.captures.find((c) => c.captureId === scene.captureId);
    if (!capture) {
      return;
    }
    capture.scenes = [scene, ...capture.scenes];
    this.captures = [
      capture,
      ...this.captures.filter((c) => c.captureId !== scene.captureId),
    ];
    this.capture_pendings_id = this.capture_pendings_id.filter(
      (id) => id !== scene.captureId
    );
  }

  @action
  addPendingCapture(capture_id: number) {
    this.capture_pendings_id.push(capture_id);
    if (this.captures.find((c) => c.captureId === capture_id)) {
      return;
    }
    this.captures = [
      {
        captureId: capture_id,
        scenes: [],
        spaceId: this.space_id ?? -1,
      },
      ...this.captures,
    ];
  }

  @action
  loadStatus() {
    this.is_capture_running = false;
    httpGet("/capture/")
      .onSuccess(
        (data: {
          space_id?: number;
          map_name?: string;
          message_def: { [key: string]: CaptureMessageDefGroup };
          use_slam: boolean;
          space_name?: string;
        }) => {
          if (data.space_id && data.space_id > 0) {
            this.space_id = data.space_id;
            this.map_name = data.map_name;
            this.is_capture_running = true;
            this.use_slam = data.use_slam;
            this.def_switchs = data.message_def;
            this.space_name = data.space_name;

            this.fetchCaptureSpace(data.space_id);
          }
        }
      )
      .onError((c, m, e) => {
        alertStore.addAlert(
          "error",
          `${e ?? ""}` + m?.toString() ?? "Failed to Load Capture Status",
          c?.toString() ?? ""
        );
      })
      .fetch();
  }
  @action
  topicSwitchOn(topic: string) {
    httpPost(`/capture/message_group/${topic}/enable`)
      .onSuccess(
        (data: {
          space_id?: number;
          map_name?: string;
          message_def: { [key: string]: CaptureMessageDefGroup };
          use_slam: boolean;
          space_name?: string;
        }) => {
          this.def_switchs = data.message_def;
        }
      )
      .fetch();
  }
  @action
  topicSwitchOff(topic: string) {
    httpPost(`/capture/message_group/${topic}/disable`)
      .onSuccess(
        (data: {
          space_id?: number;
          map_name?: string;
          message_def: { [key: string]: CaptureMessageDefGroup };
          use_slam: boolean;
          space_name?: string;
        }) => {
          this.def_switchs = data.message_def;
        }
      )
      .fetch();
  }

  loadSpace(space: CaptureAppSpace) {
    this.fetchCaptureSpace(space.spaceId);
    this.fetchCaptureSpaceAllScene(space.spaceId);
    this.space_id = space.spaceId;
    this.map_name = space.mapName;
    this.space_name = space.spaceName;
    runInAction(() => {
      this.is_capture_running = false;
    });
  }
  @action
  initSpace() {
    httpPost("/capture/init", {
      space_id: null,
      space_name: this.space_name,
      use_slam: this.use_slam,
    })
      .onSuccess((d) => {
        this.is_capture_running = true;
        this.space_id = d.space_id;
        this.loadStatus();
      })
      .onError((c, m, e) => {
        this.is_capture_running = false;
        alertStore.addAlert(
          "error",
          "Failed to Init Space",
          "CaptureStore.initSpace"
        );
      })
      .fetch();

    /// fetch Init Space to /capture app
  }
  @action
  enableSpaceCapture() {
    //check space_id is not undefined
    if (!this.space_id) return;
    httpPost("/capture/init", {
      space_id: this.space_id,
    })
      .onSuccess(() => {
        this.is_capture_running = true;
        this.loadStatus();
      })
      .onError((c, m, e) => {
        alertStore.addAlert(
          "error",
          "Failed to Enable Capture",
          "CaptureStore.enableSpaceCapture"
        );
      })
      .fetch();
  }
  @action
  disableSpaceCapture() {
    httpPost("/capture/close")
      .onSuccess(() => {
        this.is_capture_running = false;
      })
      .fetch();
  }

  /////////////////////////////////////////////
  // Capture Request Backend API
  /////////////////////////////////////////////
  @action
  fetchPostCaptureQueue = () => {
    httpPost("/capture/capture")
      .onSuccess((data: { space_id: number; capture_id: number }) => {
        this.addPendingCapture(data.capture_id);
      })
      .onError((c, m, e) => {
        alertStore.addAlert(
          "error",
          "Failed to Capture Single : " + m,
          "CaptureStore.fetchPostCaptureSingle"
        );
      })
      .fetch();
  };
  @action
  fetchPostCaptureSingle = () => {
    httpPost("/capture/capture/single")
      .onSuccess((data: { space_id: number; capture_id: number }) => {
        this.addPendingCapture(data.capture_id);
      })
      .onError((c, m, e) => {
        alertStore.addAlert(
          "error",
          "Failed to Capture Single : " + m,
          "CaptureStore.fetchPostCaptureSingle"
        );
      })
      .fetch();
  };

  @action
  fetchAbortCapture = () => {
    httpPost("/capture/capture/abort")
      .onSuccess(() => {})
      .fetch();
  };

  /////////////////////////////////////////////
  // Result Vieweing Backend API
  /////////////////////////////////////////////
  fetchGetCaptureSpaceList = () => {
    httpGet("/capture/result")
      .onSuccess((data) => {
        runInAction(() => {
          this.captureSpaceList = data;
        });
      })
      .fetch();
  };

  fetchCaptureSpaceAllScene(space_id: number) {
    httpGet(`/capture/result/${space_id}/scenes`)
      .onSuccess((data) => {
        console.log(data);
        runInAction(() => {
          this.captureScenes = data;
          console.log("CaptureScene", this.captureScenes);
        });
      })
      .fetch();
  }

  fetchCaptureSpace(space_id: number) {
    httpGet(`/capture/result/${space_id}/captures`)
      .onSuccess((data) => {
        runInAction(() => {
          this.captures = data;
        });
      })
      .fetch();
  }
}

export const captureStore = new CaptureStore();
