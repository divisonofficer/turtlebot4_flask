import {
  CalendarIcon,
  ChatIcon,
  ChevronRightIcon,
  InfoIcon,
  PlusSquareIcon,
} from "@chakra-ui/icons";
import { ComponentWithAs, IconProps } from "@chakra-ui/react";
import {
  ArrowBendDoubleUpLeft,
  ArrowBendDoubleUpRight,
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
};
