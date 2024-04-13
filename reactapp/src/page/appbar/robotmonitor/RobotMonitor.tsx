import { Flex, HStack } from "@chakra-ui/react";
import { useEffect, useRef, useState } from "react";
import { rosSocket } from "../../../connect/socket/subscribe";
import RingIcon from "../../../design/other/Ring";
import { Color } from "../../../design/color";
import { InfoCard } from "../../../design/other/infocard";

export const RobotMonitor = () => {
  const [robotMonitorData, setRobotMonitorData] = useState<RobotMonitorData>({
    topic_time: 0,
    topic_interval: 0,
    robot_ros_status: false,
    robot_node_status: false,
    robot_network_status: false,
    robot_joy_status: false,
    robot_battery: 0,
    robot_lidar_status: false,
    robot_oakd_status: false,
    create3_status: false,
  });

  useEffect(() => {
    rosSocket.subscribe("/turtlebot/monitor", (raw: string) => {
      const data = JSON.parse(raw);
      setRobotMonitorData(data);
    });
  }, []);

  return (
    <HStack>
      <MonitorIconHover
        hoverView={
          <InfoCard
            color={Color.Indigo}
            title={"Robot Battery"}
            value={robotMonitorData.robot_battery + "%"}
          />
        }
      >
        <RingIcon battery={robotMonitorData.robot_battery} />
      </MonitorIconHover>

      <MonitorIcon
        name="ROS Network"
        icon={require("../../../resource/icon_ros_network.png")}
        active={robotMonitorData.robot_network_status}
      />
      <MonitorIcon
        name="Robot"
        icon={require("../../../resource/icon_robot.png")}
        active={robotMonitorData.robot_ros_status}
      />
      <MonitorIcon
        name="Turtle"
        icon={require("../../../resource/icon_turtle.png")}
        active={robotMonitorData.robot_node_status}
      />
      <MonitorIcon
        name="Create3"
        icon={require("../../../resource/icon_wheel.png")}
        active={robotMonitorData.create3_status}
        altColor={Color.Mint}
      />
      <MonitorIcon
        name="Lidar"
        icon={require("../../../resource/icon_lidar.png")}
        active={robotMonitorData.robot_lidar_status}
        altColor={Color.Mint}
      />
      <MonitorIcon
        name="Camera"
        icon={require("../../../resource/icon_camera.png")}
        active={robotMonitorData.robot_oakd_status}
        altColor={Color.Cyan}
      />
      <MonitorIcon
        name="Joystick"
        icon={require("../../../resource/icon_joystick.png")}
        active={robotMonitorData.robot_joy_status}
      />
    </HStack>
  );
};

export const MonitorIcon = ({
  icon,
  active,
  altColor,
  name,
}: {
  name?: string;
  icon: string;
  active?: boolean;
  altColor?: string;
}) => {
  return (
    <MonitorIconHover
      hoverView={
        <InfoCard
          color={Color.Indigo}
          title={name || ""}
          value={active ? "Active" : "Inactive"}
        />
      }
    >
      <img
        src={icon}
        alt="icon"
        style={{
          width: "24px",
          height: "24px",
          objectFit: "cover",
          ...(!active && {
            filter: "grayscale(100%) opacity(50%)",
          }),
          ...(active &&
            altColor && {
              filter: `drop-shadow(0 0 5px ${altColor})`,
            }),
          ...(active && {
            cursor: "pointer",
          }),
        }}
      />
    </MonitorIconHover>
  );
};

export const MonitorIconHover = ({
  children,
  hoverView,
  dropView,
  style,
}: {
  children: React.ReactNode;
  hoverView?: React.ReactNode;
  dropView?: React.ReactNode;
  style?: React.CSSProperties;
}) => {
  const [hover, setHover] = useState(false);
  const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 });

  const hoverRef = useRef<HTMLDivElement>(null);

  const hoverLeft =
    mousePosition.x > window.innerWidth - (hoverRef.current?.clientWidth || 0)
      ? window.innerWidth - (hoverRef.current?.clientWidth || 0)
      : mousePosition.x;

  return (
    <Flex
      style={{
        width: "24px",
        height: "24px",
        ...style,
      }}
      onMouseEnter={(e) => {
        setHover(true);
      }}
      onMouseMove={(e) => {
        setMousePosition({ x: e.clientX, y: e.clientY });
      }}
      onMouseLeave={(e) => {
        setHover(false);
      }}
    >
      {children}
      {hover && hoverView && (
        <div
          style={{
            position: "absolute",
            top: `${mousePosition.y + 10}px`,
            left: `${hoverLeft}px`,

            color: "white",
            padding: "4px",
            borderRadius: "4px",
            fontSize: "12px",
            whiteSpace: "nowrap",
            pointerEvents: "none",
          }}
          ref={hoverRef}
        >
          {hoverView}
        </div>
      )}
    </Flex>
  );
};

export interface RobotMonitorData {
  topic_time: number;
  topic_interval: number;
  robot_ros_status: boolean;
  robot_node_status: boolean;
  robot_network_status: boolean;
  robot_joy_status: boolean;
  robot_battery: number;
  robot_lidar_status: boolean;
  robot_oakd_status: boolean;
  create3_status: boolean;
}
