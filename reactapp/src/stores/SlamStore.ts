import { makeAutoObservable, runInAction } from "mobx";
import { SlamMapMetaData } from "../page/slam/SlamType";
import { slamSocket } from "../connect/socket/subscribe";
import { httpGet } from "../connect/http/request";
import { MapMarker, Pose3D, SlamState } from "../public/proto/slam";

class SlamStore {
  constructor() {
    makeAutoObservable(this);
    this.subscribeToSlam();
  }

  slamStatus?: SlamState = undefined;

  mapMetadataView?: SlamMapMetaData = undefined;

  slamRobotPose?: Pose3D = undefined;
  markers: MapMarker[] = [];

  subscribeToSlam() {
    slamSocket.subscribeBuffer("slam_status", SlamState, (state: SlamState) => {
      this.setSlamStatus(state);
      if (state.markers) {
        this.setMarkers(state.markers);
      }
    });

    slamSocket.subscribeBuffer("robot_pose", Pose3D, (data: Pose3D) => {
      this.setSlamRobotPose(data);
    });

    // slamSocket.subscribe("markers", (data: MapMarker[]) => {
    //   this.setMarkers(data);
    // });
  }

  setSlamStatus(status: SlamState) {
    this.slamStatus = status;
  }

  setSlamRobotPose(pose: Pose3D) {
    this.slamRobotPose = pose;
  }

  setMarkers(markers: MapMarker[]) {
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
