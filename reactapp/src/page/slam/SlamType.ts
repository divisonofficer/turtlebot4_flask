export interface SlamStatus {
  status: string;
  message: string;
  robot_pose?: SlamRobotPosition;
  map_origin?: SlamRobotPosition;
  map_size?: { width: number; height: number; resolution: number };
  markers?: {
    id: number;
    pose: SlamRobotPose;
  }[];
  slam_metadata?: SlamMetaData;
}

export interface SlamMapMetaData {
  map_origin: SlamRobotPosition;
  map_size: MapSize;
}

export interface MapSize {
  width: number;
  height: number;
  resolution: number;
}

export interface SlamRobotPosition {
  x: number;
  y: number;
}
export interface SlamRobotOrientation {
  roll: number;
  pitch: number;
  yaw: number;
}
export interface SlamRobotOrientationQuat {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface SlamRobotPose {
  x: number;
  y: number;
  z: number;
  orientation: SlamRobotOrientation;
  orientation_quaternion: SlamRobotOrientationQuat;
}

export interface RobotPose {
  orientation: SlamRobotOrientation;
  position: SlamRobotPosition;
}

export interface SlamMetaData {
  pos_interval: number;
  map_interval: number;
  lidar_interval: number;
}
