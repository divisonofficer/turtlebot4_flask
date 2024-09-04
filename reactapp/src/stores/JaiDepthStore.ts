import { makeAutoObservable } from "mobx";
import { httpGet, httpPost } from "../connect/http/request";
import axios from "axios";

export interface FrameInfo {
  frame_id: string;
  scene_id: string;
  capture_rgb: {
    disparity_viz_path?: string;
  };
  capture_nir: {
    disparity_viz_path?: string;
  };
  lidar_path?: string;
  lidar_projected_path?: string;
  lidar_ply_path?: string;
}
class JaiDepthStore {
  remote_storage: boolean = false;

  local_storage_url =
    "/home/cglab/project/turtlebot4_flask/clientapp/tmp/depth";
  remote_storage_url = "/bean/depth";

  scene_list: string[] = [];
  scene_frame_list: string[] = [];

  scene_calibration: Object | undefined = undefined;

  scene_id: string | undefined = undefined;
  frame_id: string | undefined = undefined;

  constructor() {
    makeAutoObservable(this);
  }

  storage_url = () => {
    return this.remote_storage
      ? this.remote_storage_url
      : this.local_storage_url;
  };

  getFrameInfo = (frame: string, callback: (f: FrameInfo) => void) => {
    httpGet(
      `/jai/stereo/storage/${
        this.scene_id
      }/frame/${frame}?root=${this.storage_url()}`
    )
      .onSuccess((data) => {
        callback(data);
      })
      .fetch();
  };

  fetchSceneCalibration = (scene: string) => {
    httpGet(
      `/jai/stereo/storage/${scene}/calibration?root=${this.storage_url()}`
    )
      .onSuccess((data) => {
        this.scene_calibration = data;
      })
      .onError((err) => {
        this.scene_calibration = undefined;
      })
      .fetch();
  };
  fetchSceneRequestLoadCalibration = (scene: string, cal_id: number) => {
    httpPost(
      `/jai/stereo/storage/${scene}/calibration/load?root=${this.storage_url()}`,
      {
        calibration_id: cal_id,
      }
    )
      .onSuccess((data) => {
        this.fetchSceneCalibration(scene);
      })
      .fetch();
  };

  fetchStorageList = () => {
    httpGet(`/jai/stereo/storage/list?root=${this.storage_url()}`)
      .onSuccess((data) => {
        this.scene_list = data;
      })
      .onError((err) => {
        console.log(err);
      })
      .fetch();
  };

  fetchStorageFrames = (scene: string) => {
    httpGet(`/jai/stereo/storage/${scene}/frames?root=${this.storage_url()}`)
      .onSuccess((data) => {
        this.scene_frame_list = data;
        this.scene_id = scene;
        this.fetchSceneCalibration(scene);
      })
      .fetch();
  };

  fetchDownloadFile = (
    frame: string,
    file: string,
    channel: "rgb" | "nir" | null = null
  ) => {
    axios({
      url: `/jai/stereo/storage/${
        this.scene_id
      }/frame/${frame}/file/${file}/${channel}?root=${this.storage_url()}`,
      method: "GET",
      responseType: "blob",
    }).then((response) => {
      const url = window.URL.createObjectURL(new Blob([response.data]));
      const link = document.createElement("a");
      link.href = url;
      link.setAttribute("download", file);
      document.body.appendChild(link);
      link.click();
    });
  };

  fetchProcessCreatePly = (frame: string, onSucess: () => void) => {
    httpPost(
      `/jai/stereo/storage/${
        this.scene_id
      }/frame/${frame}/ply?root=${this.storage_url()}`
    )
      .onSuccess((data) => {
        onSucess();
      })
      .fetch();
  };
  fetchProcessDisparity = (frame: string, onSucess: () => void) => {
    httpPost(
      `/jai/stereo/storage/${
        this.scene_id
      }/frame/${frame}/disparity?root=${this.storage_url()}`
    )
      .onSuccess((data) => {
        onSucess();
      })
      .fetch();
  };
}

export const jaiDepthStore = new JaiDepthStore();
