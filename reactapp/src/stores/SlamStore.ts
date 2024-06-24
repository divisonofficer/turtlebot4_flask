import { makeAutoObservable, runInAction } from "mobx";
import { SlamMapMetaData } from "../page/slam/SlamType";
import { slamSocket } from "../connect/socket/subscribe";
import { httpGet, httpPost } from "../connect/http/request";
import {
  MapMarker,
  Point3D,
  Point3DArray,
  Pose3D,
  Pose3DArray,
  SlamState,
  SlamState_Status,
} from "../public/proto/slam";

class SlamStore {
  constructor() {
    makeAutoObservable(this);
    this.subscribeToSlam();
    this.fetchCreate3Status();
  }

  slamStatus?: SlamState = undefined;

  slamMode: "offline" | "slam_toolbox" | "create3" = "offline";

  mapMetadataView?: SlamMapMetaData = undefined;

  slamRobotPose?: Pose3D = undefined;
  markers: MapMarker[] = [];

  create3_poses: Pose3D[] = [];
  create3_pose_stamps: Pose3D[] = [];
  create3_pointcloud: Point3D[] = [];
  create3_scanpoints: Point3D[] = [];
  create3_obstacles: Point3D[] = [];
  create3_obstacle_map: Point3D[] = [];

  subscribeToSlam() {
    slamSocket.subscribeBuffer("slam_status", SlamState, (state: SlamState) => {
      this.setSlamStatus(state);
      if (state.status === SlamState_Status.SUCCESS) {
        this.slamMode = "slam_toolbox";
      }
      if (state.markers) {
        this.setMarkers(state.markers);
      }
    });

    slamSocket.subscribeBuffer("robot_pose", Pose3D, (data: Pose3D) => {
      this.setSlamRobotPose(data);
    });

    slamSocket.subscribe("create3_poses", (data: [Pose3D]) => {
      runInAction(() => {
        if (!data) return;
        if (data.length < 1) {
          return;
        }
        this.create3_poses = data;
        this.slamMode = "create3";
      });
    });

    slamSocket.subscribeBuffer(
      "create3_pose_stamps",
      Pose3DArray,
      (data: Pose3DArray) => {
        runInAction(() => {
          this.create3_pose_stamps = data.poses;
          this.slamMode = "create3";
        });
      }
    );

    slamSocket.subscribeBuffer(
      "create3_pointcloud",
      Point3DArray,
      (data: Point3DArray) => {
        runInAction(() => {
          data.points.forEach((p) => {
            p.x = -p.x;
          });
          this.create3_pointcloud = data.points;
          this.slamMode = "create3";
        });
      }
    );

    slamSocket.subscribeBuffer(
      "create3_scan",
      Point3DArray,
      (data: Point3DArray) => {
        runInAction(() => {
          data.points.forEach((p) => {
            p.x = -p.x;
          });
          this.create3_scanpoints = data.points;
          this.slamMode = "create3";
        });
      }
    );

    slamSocket.subscribeBuffer(
      "create3_obstacles",
      Point3DArray,
      (data: Point3DArray) => {
        runInAction(() => {
          data.points.forEach((p) => {
            p.x = -p.x;
          });
          this.create3_obstacles = data.points;
          this.slamMode = "create3";
        });
      }
    );

    slamSocket.subscribeBuffer(
      "create3_obstacle_map",
      Point3DArray,
      (data: Point3DArray) => {
        runInAction(() => {
          data.points.forEach((p) => {
            p.x = -p.x;
          });
          this.create3_obstacle_map = data.points;
          this.slamMode = "create3";
        });
      }
    );

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

  fetchCreate3Status = () => {
    httpGet(`/slam/create3`)
      .onSuccess((data) => {
        runInAction(() => {
          this.slamMode = "create3";
          this.create3_poses = [data];
        });
      })
      .fetch();
  };

  fetchCreate3Navigate = (dest: { x: number; y: number }) => {
    dest.x = -dest.x;
    httpPost(`/slam/create3/action/navigate_to_position`, dest)
      .onSuccess((data) => {})
      .fetch();
  };

  fetchCreate3Drive = (dest: { distance: number }) => {
    httpPost(`/slam/create3/action/drive`, dest)
      .onSuccess((data) => {})
      .fetch();
  };

  fetchCreate3Rotate = (angle: number) => {
    httpPost(`/slam/create3/action/rotate`, { angle })
      .onSuccess((data) => {})
      .fetch();
  };

  fetchCreate3Estop = () => {
    httpPost(`/slam/create3/action/estop`)
      .onSuccess((data) => {})
      .fetch();
  };

  fetchCreate3EstopRelease = () => {
    httpPost(`/slam/create3/action/estop/release`)
      .onSuccess((data) => {})
      .fetch();
  };

  create3ResetPose = () => {
    httpPost(`/slam/create3/action/reset_pose`)
      .onSuccess((data) => {})
      .fetch();
  };
}

export const slamStore = new SlamStore();
