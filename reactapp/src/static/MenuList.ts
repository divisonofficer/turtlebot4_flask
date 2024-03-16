import {
  CalendarIcon,
  ChatIcon,
  InfoIcon,
  PlusSquareIcon,
} from "@chakra-ui/icons";
import {
  ArrowBendDoubleUpLeft,
  Notification,
  Robot,
  RocketLaunch,
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
    icon: ArrowBendDoubleUpLeft,
    name: "Lidar",
    destination: "/lidar",
  },
];

export type MenuItem = {
  icon: ElementType | string;
  name: string;
  destination?: string;
  subMenu?: MenuItem[];
};
