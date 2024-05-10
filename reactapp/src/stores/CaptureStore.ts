import { action, makeAutoObservable, runInAction } from "mobx";
import { RobotPose, SlamRobotPose } from "../page/slam/SlamType";
import { httpGet, httpPost } from "../connect/http/request";
import { alertStore } from "./AlertStore";
import { captureSocket } from "../connect/socket/subscribe";
import {
  CaptureAppCapture,
  CaptureAppScene,
  CaptureAppSpace,
  CaptureTaskProgress,
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

    captureSocket.subscribe("/recent_scene", (data: CaptureAppScene) => {
      this.extendSceneId(data);
    });
    captureSocket.subscribe("/progress", (data: CaptureTaskProgress) =>
      this.processCaptureProgress(data)
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

  map_focused_capture_id: number = -1;
  map_focused_scene_id: number = -1;

  is_capture_running: boolean = false;
  progress?: CaptureTaskProgress = undefined;
  /////////////////////////////////////////////
  // For Capture Control
  /////////////////////////////////////////////
  image_topic_switches = Object.values(CAPTURE_TOPICS).map((topic) => ({
    ...topic,
    checked: topic.default_checked,
  }));
  /////////////////////////////////////////////
  // Capture Control Action
  /////////////////////////////////////////////
  @action
  extendSceneId(scene_meta: CaptureAppScene) {
    httpGet(
      `/capture/result/${this.space_id}/${scene_meta.captureId}/${scene_meta.sceneId}/images`
    )
      .onSuccess((images: string[]) => {
        scene_meta.images = images;
        this.extendScene(scene_meta);
      })
      .fetch();
  }

  @action
  processCaptureProgress(progress: CaptureTaskProgress) {
    this.progress = progress;
    if (progress.progress >= 100) {
      this.progress = undefined;
    }
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
      .onSuccess((data: { space_id?: number; map_name?: string }) => {
        if (data.space_id && data.space_id > 0) {
          this.space_id = data.space_id;
          this.map_name = data.map_name;
          this.is_capture_running = true;
          this.fetchCaptureSpace(data.space_id);
        }
      })
      .onError((c, m, e) => {
        alertStore.addAlert(
          "error",
          `${e ?? ""}` + m?.toString() ?? "Failed to Load Capture Status",
          c?.toString() ?? ""
        );
      })
      .fetch();
  }

  topicSwitchOn(topic: string) {
    runInAction(() =>
      this.image_topic_switches.forEach((t) => {
        if (t.topic === topic) {
          t.checked = true;
        }
      })
    );
  }

  topicSwitchOff(topic: string) {
    runInAction(() =>
      this.image_topic_switches.forEach((t) => {
        if (t.topic === topic) {
          t.checked = false;
        }
      })
    );
  }

  loadSpace(space: CaptureAppSpace) {
    this.fetchCaptureSpace(space.spaceId);
    this.fetchCaptureSpaceAllScene(space.spaceId);
    this.space_id = space.spaceId;
    this.map_name = space.mapName;
    runInAction(() => {
      this.is_capture_running = false;
    });
  }
  @action
  initSpace() {
    httpPost("/capture/init", {
      space_id: null,
    })
      .onSuccess((d) => {
        this.is_capture_running = true;
        this.space_id = d.space_id;
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
    httpPost("/capture/capture", {
      topics: this.image_topic_switches
        .filter((topic) => topic.checked)
        .map((topic) => topic.topic),
    })
      .onSuccess((data: { space_id: number; capture_id: number }) => {
        this.addPendingCapture(data.capture_id);
      })
      .fetch();
  };
  @action
  fetchPostCaptureSingle = () => {
    httpPost("/capture/capture/single", {
      topics: this.image_topic_switches
        .filter((topic) => topic.checked)
        .map((topic) => topic.topic),
    })
      .onSuccess((data: { space_id: number; capture_id: number }) => {
        this.addPendingCapture(data.capture_id);
      })
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

export interface CaptureSingle {
  capture_id: number;
  scenes: CaptureScene[];
}

export interface CaptureScene {
  capture_id: number;
  scene_id: number;
  space_id: number;
  images: string[];
  robot_pose: RobotPose;
  timestamp: number;
}

export interface CaptureSpace {
  space_id: number;
  captures: number[];
  map_name?: string;
}

export const captureStore = new CaptureStore();
