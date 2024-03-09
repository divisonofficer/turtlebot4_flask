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
    icon: ChevronRightIcon,
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
  },
  {
    icon: CalendarIcon,
    name: "Nodes",
  },
];

export type MenuItem = {
  icon: ComponentWithAs<"svg", IconProps>;
  name: string;
  destination?: string;
};
