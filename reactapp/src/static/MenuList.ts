import {
  CalendarIcon,
  ChatIcon,
  ChevronRightIcon,
  InfoIcon,
  PlusSquareIcon,
} from "@chakra-ui/icons";
import { ComponentWithAs, IconProps } from "@chakra-ui/react";

export const menuList = [
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
];

export type MenuItem = {
  icon: ComponentWithAs<"svg", IconProps> | string;
  name: string;
  destination?: string;
};
