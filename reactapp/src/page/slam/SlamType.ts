export interface SlamStatus {
  status: string;
  message: string;
  robot_pose?: { x: number; y: number };
  map_origin?: { x: number; y: number };
  map_size?: { width: number; height: number; resolution: number };
  markers?: {
    id: number;
    pose: SlamRobotPose;
  }[];
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
