import {
  CalendarIcon,
  ChatIcon,
  InfoIcon,
  PlusSquareIcon,
} from "@chakra-ui/icons";
import {
  Angle,
  ArrowBendDoubleUpLeft,
  Camera,
  CameraRotate,
  CameraSlash,
  MapPin,
  MonitorArrowUp,
  Notification,
  Robot,
  RocketLaunch,
  VideoCameraSlash,
} from "@phosphor-icons/react";
import { ElementType } from "react";

export const menuList: MenuItem[] = [
  {
    icon: InfoIcon,
    name: "Dashboard",
    destination: "/",
  },
  {
    icon: "https://cdn3.iconfinder.com/data/icons/driver-wheel/154/race-car-auto-wheel-512.png",
    name: "Joystick",
    destination: "/joystick",
  },
  {
    icon: Robot,
    name: "ROS",
    subMenu: [
      {
        icon: RocketLaunch,
        name: "Launch",
        destination: "/launch",
      },
      {
        icon: PlusSquareIcon,
        name: "ServiceCall",
        destination: "/service_call",
      },
      {
        icon: ChatIcon,
        name: "Topics",
        destination: "/topic",
      },
      {
        icon: CalendarIcon,
        name: "Nodes",
        destination: "/nodes",
      },
      {
        icon: Notification,
        name: "Diagnotics",
        destination: "/diagnostics",
        subMenu: [
          {
            icon: InfoIcon,
            name: "Detail",
            destination: "/diagnostics/detail",
          },
        ],
      },
    ],
  },
  {
    icon: Camera,
    name: "Capture",
    destination: "/capture",
  },
  {
    icon: ArrowBendDoubleUpLeft,
    name: "Lidar",
    destination: "/lidar",
  },
  {
    icon: MapPin,
    name: "SLAM",
    destination: "/slam",
  },
  {
    icon: CameraSlash,
    name: "JaiBridge",
    destination: "/jaibridge",
    subMenu: [
      {
        icon: CameraRotate,
        name: "Calibration",
        destination: "/jaibridge/calibration",
      },
      {
        icon: MonitorArrowUp,
        name: "Depth",
        destination: "/jaibridge/depth",
        subMenu: [
          {
            icon: VideoCameraSlash,
            name: "Storage",
            destination: "/jaibridge/depth/storage",
          },
        ],
      },
    ],
  },
  {
    icon: Angle,
    name: "Lucid Capture",
    destination: "/lucid",
  },
  {
    icon: Angle,
    name: "Polarization",
    destination: "/polarization",
  },
];

export type MenuItem = {
  icon: ElementType | string;
  name: string;
  destination?: string;
  subMenu?: MenuItem[];
};
