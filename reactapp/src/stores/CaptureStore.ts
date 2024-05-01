import { action, makeAutoObservable, runInAction } from "mobx";
import { RobotPose, SlamRobotPose } from "../page/slam/SlamType";
import { httpGet, httpPost } from "../connect/http/request";
import { alertStore } from "./AlertStore";
import { captureSocket } from "../connect/socket/subscribe";

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
    topic: "/jai_camera/image_raw",
    default_checked: false,
  },
  JAI_NIR: {
    name: "JAI NIR",
    topic: "/jai_camera/nir",
    default_checked: false,
  },
};

class CaptureStore {
  constructor() {
    makeAutoObservable(this);

    captureSocket.subscribe("/recent_scene", (data: CaptureScene) => {
      this.extendSceneId(data.capture_id, data.scene_id);
    });
  }
  /////////////////////////////////////////////
  // For All Backend Stored data
  /////////////////////////////////////////////
  captureSpaceList: CaptureSpace[] = [];
  /////////////////////////////////////////////
  // For Single Space data for viewing
  /////////////////////////////////////////////
  captureScenes: CaptureScene[] = [];
  captures: CaptureSingle[] = [];
  capture_pendings_id: number[] = [];

  map_name?: string = undefined;

  space_id?: number = undefined;

  map_focused_capture_id: number = -1;
  map_focused_scene_id: number = -1;

  is_capture_running: boolean = false;
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

  extendSceneId(capture_id: number, scene_id: number) {
    httpGet(
      `/capture/result/${this.space_id}/${capture_id}/${scene_id}`
    ).onSuccess((scene: CaptureScene) => {
      this.extendScene(scene);
    });
  }

  @action
  extendScene(scene: CaptureScene) {
    this.captureScenes = [scene, ...this.captureScenes];

    // find capture and put scene
    const capture = this.captures.find(
      (c) => c.capture_id === scene.capture_id
    );
    if (!capture) {
      return;
    }
    capture.scenes = [scene, ...capture.scenes];
    this.captures = [
      capture,
      ...this.captures.filter((c) => c.capture_id !== scene.capture_id),
    ];
  }

  @action
  addPendingCapture(capture_id: number) {
    this.capture_pendings_id.push(capture_id);
    if (this.captures.find((c) => c.capture_id === capture_id)) {
      return;
    }
    this.captures = [
      {
        capture_id: capture_id,
        scenes: [],
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

  loadSpace(space: CaptureSpace) {
    this.fetchCaptureSpace(space.space_id);
    this.fetchCaptureSpaceAllScene(space.space_id);
    this.space_id = space.space_id;
    this.map_name = space.map_name;
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
