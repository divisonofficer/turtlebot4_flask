import { HStack } from "@chakra-ui/react";
import { BatteryMonitorIcon } from "./battery/BatteryMonitor";
import { RobotMonitor } from "./robotmonitor/RobotMonitor";

export const Appbar = () => {
  return (
    <HStack
      style={{
        width: "100%",
        justifyContent: "flex-end",
        height: "2rem",
        overflowX: "hidden",
        overflowY: "hidden",
        zIndex: 100,
      }}
    >
      <RobotMonitor />
      <BatteryMonitorIcon />
    </HStack>
  );
};
