import { makeAutoObservable, runInAction } from "mobx";
import {
  SlamMapMetaData,
  SlamRobotPose,
  SlamStatus,
} from "../page/slam/SlamType";
import { slamSocket } from "../connect/socket/subscribe";
import { httpGet } from "../connect/http/request";

class SlamStore {
  constructor() {
    makeAutoObservable(this);
    this.subscribeToSlam();
  }

  slamStatus?: SlamStatus = undefined;

  mapMetadataView?: SlamMapMetaData = undefined;

  slamRobotPose?: SlamRobotPose = undefined;
  markers: { id: number; pose: SlamRobotPose }[] = [];

  subscribeToSlam() {
    slamSocket.subscribe("slam_status", (data: SlamStatus) => {
      this.setSlamStatus(data);
      if (data.markers) {
        this.setMarkers(data.markers);
      }
    });

    slamSocket.subscribe("robot_pose", (data: SlamRobotPose) => {
      this.setSlamRobotPose(data);
    });

    slamSocket.subscribe(
      "markers",
      (data: { id: number; pose: SlamRobotPose }[]) => {
        this.setMarkers(data);
      }
    );
  }

  setSlamStatus(status: SlamStatus) {
    this.slamStatus = status;
  }

  setSlamRobotPose(pose: SlamRobotPose) {
    this.slamRobotPose = pose;
  }

  setMarkers(markers: { id: number; pose: SlamRobotPose }[]) {
    this.markers = markers;
  }

  fetchGetMapMetadata = (map_name: string) => {
    httpGet(`/slam/map/${map_name}`)
      .onSuccess((data) => {
        runInAction(() => {
          this.mapMetadataView = data;
        });
      })
      .fetch();
  };
}

export const slamStore = new SlamStore();
