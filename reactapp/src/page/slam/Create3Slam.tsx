import {
  Button,
  Flex,
  HStack,
  Img,
  Menu,
  MenuButton,
  MenuItem,
  MenuList,
  Switch,
  VStack,
} from "@chakra-ui/react";
import { observer } from "mobx-react";
import { slamStore } from "../../stores/SlamStore";
import { ArrowRight } from "@phosphor-icons/react";
import { useRef, useState } from "react";
import { Body3 } from "../../design/text/textsystem";
import { Btn } from "../../design/button/button";
import { Color } from "../../design/color";
import RobotPon from "../../resource/icon_robot_pon.png";
import { ChevronDownIcon } from "@chakra-ui/icons";

type Create3Aciton = "navigate" | "rotate" | "drive" | "stop";
const Create3ActionList: Create3Aciton[] = [
  "navigate",
  "rotate",
  "drive",
  "stop",
];

const ActionControlPannel = (props: {
  create3Action: Create3Aciton;
  setCreate3Action: (action: Create3Aciton) => void;
}) => {
  return (
    <VStack>
      <Menu placement="bottom-end">
        <MenuButton as={Button} rightIcon={<ChevronDownIcon />}>
          {props.create3Action}
        </MenuButton>
        <MenuList>
          {Create3ActionList.map((def, index) => {
            return (
              <MenuItem
                key={index}
                fontSize="small"
                onClick={() => {
                  props.setCreate3Action(def);
                }}
              >
                {def}
              </MenuItem>
            );
          })}
        </MenuList>
      </Menu>
    </VStack>
  );
};

export const Line2Point = (props: {
  from: { x: number; y: number };
  to: { x?: number; y?: number; direction?: number; distance?: number };
  orientation?: { x: number; y: number };
  scale: number;
  color: string;
}) => {
  const distance =
    props.to.distance ||
    Math.sqrt(
      Math.pow(props.to.x! - props.from.x, 2) +
        Math.pow(props.to.y! - props.from.y, 2)
    );

  return (
    <Flex
      style={{
        position: "absolute",
        left: `${50 + (props.from.x * 100) / props.scale}%`,
        top: `${50 + (props.from.y * 100) / props.scale}%`,
        width: `${(distance * 100) / props.scale}%`,
        height: `2px`,
        background: props.color,
        transformOrigin: "0 0",
        transform: `rotate(${
          props.to.direction ??
          (Math.atan2(props.to.y! - props.from.y, props.to.x! - props.from.x) *
            180) /
            Math.PI
        }deg)`,
      }}
    ></Flex>
  );
};

export const Create3Map = observer(() => {
  const mapWindowRef = useRef<HTMLDivElement>(null);

  const [mousePosition, setMousePosition] = useState<{ x: number; y: number }>({
    x: 0,
    y: 0,
  });
  const [mouseTarget, setMouseTarget] = useState<{ x: number; y: number }>({
    x: 0,
    y: 0,
  });

  const [mapSize, setMapSize] = useState<{ width: number; height: number }>({
    width: 0,
    height: 0,
  });

  const [orientation, setOrientation] = useState<{ x: number; y: number }>({
    x: 0,
    y: 0,
  });

  const robot_pose =
    slamStore.create3_poses.length > 0 &&
    slamStore.create3_poses[0].position &&
    slamStore.create3_poses[0].position.x
      ? {
          position: {
            x: -slamStore.create3_poses[0].position!.x,
            y: slamStore.create3_poses[0].position!.y,
          },
          orientationEuler: {
            yaw: slamStore.create3_poses[0].orientationEuler!.yaw,
          },
        }
      : undefined;

  const [scale, setScale] = useState<number>(20);

  const [create3Action, setCreate3Action] = useState<Create3Aciton>("stop");

  const [visible, setVisible] = useState<{ [key: string]: boolean }>({
    pointcloud: true,
    obstacles: true,
    pose: true,
    trace_pose: true,
    scan: true,
  });

  const navigate = () => {
    if (!robot_pose) {
      return;
    }
    if (window.confirm(`Are you sure to ${create3Action} ?`)) {
      if (create3Action === "navigate") {
        const destination = {
          x: mouseTarget.x,
          y: mouseTarget.y,
        };

        slamStore.fetchCreate3Navigate(destination);
      }
      if (create3Action === "drive") {
        const distance = {
          distance: Math.sqrt(
            Math.pow(mouseTarget.x - robot_pose.position!.x, 2) +
              Math.pow(mouseTarget.y - robot_pose.position!.y, 2)
          ),
        };

        slamStore.fetchCreate3Drive(distance);
      }
      if (create3Action === "rotate") {
        const angle =
          Math.PI -
          Math.atan2(
            mouseTarget.y - robot_pose.position!.y,
            mouseTarget.x - robot_pose.position!.x
          ) -
          robot_pose.orientationEuler!.yaw;
        console.log("direction : ", angle);
        slamStore.fetchCreate3Rotate(angle);
      }
    }
  };

  return (
    <VStack
      style={{
        width: "100%",
      }}
    >
      <ActionControlPannel
        create3Action={create3Action}
        setCreate3Action={setCreate3Action}
      />
      <HStack>
        {Object.keys(visible).map((key) => (
          <Switch
            key={key}
            isChecked={visible[key]}
            onChange={() => setVisible({ ...visible, [key]: !visible[key] })}
          >
            {key}
          </Switch>
        ))}
      </HStack>
      <Flex
        style={{
          position: "relative",
          width: "50%",
          aspectRatio: 1,
          background: "#f0f0f0",
        }}
        ref={mapWindowRef}
        onMouseMove={(e) => {
          const rect = mapWindowRef.current!.getBoundingClientRect();
          setMapSize({
            width: rect.width,
            height: rect.height,
          });
          setMousePosition({
            x: e.clientX - rect.left,
            y: e.clientY - rect.top,
          });
          setMouseTarget({
            x:
              ((mousePosition.x - mapSize.width / 2) / mapSize.width) * scale +
              orientation.x,
            y:
              ((mousePosition.y - mapSize.height / 2) / mapSize.height) *
                scale +
              orientation.y,
          });
        }}
        onMouseDownCapture={() => navigate()}
      >
        {visible.trace_pose &&
          slamStore.create3_pose_stamps.map((pose, index) => (
            <>
              {pose.position &&
                (index < slamStore.create3_pose_stamps.length - 1 ? (
                  <Line2Point
                    from={pose.position}
                    to={slamStore.create3_pose_stamps[index + 1].position!}
                    scale={scale}
                    color="blue"
                  />
                ) : (
                  robot_pose && (
                    <Line2Point
                      from={pose.position}
                      to={robot_pose.position}
                      scale={scale}
                      color="blue"
                    />
                  )
                ))}
            </>
          ))}
        {Array.from({ length: scale / 10 + 1 }, (_, i) => i).map((_, index) =>
          [
            [0, 1],
            [1, 0],
            [0, -1],
            [-1, 0],
          ].map((direction) => {
            return (
              <Flex
                key={index}
                style={{
                  position: "absolute",
                  left:
                    direction[0] !== 0
                      ? `${50 + ((index * scale * 5) / 4) * direction[0]}%`
                      : "0%",
                  top:
                    direction[1] !== 0
                      ? `${50 + ((index * scale * 5) / 4) * direction[1]}%`
                      : "0%",
                  width: direction[0] === 0 ? "100%" : "1px",
                  height: direction[1] === 0 ? "100%" : "1px",
                  background: "black",
                }}
              />
            );
          })
        )}

        {visible.obstacles &&
          slamStore.create3_obstacle_map.map((point, index) => (
            <div
              key={index}
              style={{
                position: "absolute",
                left: `${50 + (point.x * 100) / scale}%`,
                top: `${50 + (point.y * 100) / scale}%`,
                width: `3px`,
                height: `3px`,
                background: "black",
                opacity: point.z / 20,
              }}
            />
          ))}
        {visible.pointcloud &&
          robot_pose &&
          slamStore.create3_pointcloud.map((point, index) => {
            const mouse_dir = Math.atan2(
              mouseTarget.y - robot_pose.position.y,
              mouseTarget.x - robot_pose.position.x
            );

            const point_dir = Math.atan2(
              point.y - robot_pose.position.y,
              point.x - robot_pose.position.x
            );

            const distance = Math.sqrt(
              Math.pow(point.x - robot_pose.position.x, 2) +
                Math.pow(point.y - robot_pose.position.y, 2)
            );

            const mouse_distance = Math.sqrt(
              Math.pow(mouseTarget.x - robot_pose.position.x, 2) +
                Math.pow(mouseTarget.y - robot_pose.position.y, 2)
            );

            const is_collapsed =
              Math.abs(mouse_dir - point_dir) < Math.PI / 8 &&
              distance < mouse_distance;

            return (
              <div
                key={index}
                style={{
                  position: "absolute",
                  left: `${50 + (point.x * 100) / scale}%`,
                  top: `${50 + (point.y * 100) / scale}%`,
                  width: "1px",
                  height: "1px",
                  background: is_collapsed ? "red" : "#ffaaaa",
                }}
              />
            );
          })}

        {visible.scan &&
          slamStore.create3_scanpoints.map((point, index) => {
            return (
              <div
                key={index}
                style={{
                  position: "absolute",
                  left: `${50 + (point.x * 100) / scale}%`,
                  top: `${50 + (point.y * 100) / scale}%`,
                  width: "1px",
                  height: "1px",
                  background: "green",
                }}
              />
            );
          })}

        {visible.obstacles &&
          slamStore.create3_obstacles.map((point, index) => (
            <div
              key={index}
              style={{
                position: "absolute",
                left: `${50 + (point.x * 100) / scale}%`,
                top: `${50 + (point.y * 100) / scale}%`,
                width: "3px",
                height: "3px",
                background: "red",
              }}
            />
          ))}

        {visible.pose &&
          slamStore.create3_poses.map(
            (pose, index) =>
              pose &&
              pose.position && (
                <Flex
                  key={index}
                  style={{
                    position: "absolute",
                    left: `${50 - (pose.position.x * 100) / scale}%`,
                    top: `${50 + (pose.position.y * 100) / scale}%`,
                    transform: `translate(-50%, -50%) rotate(${
                      180 - ((pose.orientationEuler?.yaw || 0) * 180) / Math.PI
                    }deg)`,
                    transformOrigin: "10px 10px",
                  }}
                >
                  <Img src={RobotPon} width="20px" height="20px" />
                </Flex>
              )
          )}
        <VStack
          style={{
            position: "absolute",
            left: `${(mousePosition.x / mapSize.width) * 100}%`,
            top: `${(mousePosition.y / mapSize.height) * 100}%`,
          }}
          align="center"
          justify="center"
        >
          <Body3>
            {mouseTarget.x.toFixed(3)}, {mouseTarget.y.toFixed(3)}
          </Body3>
          <Body3>
            {robot_pose &&
              Math.sqrt(
                Math.pow(mouseTarget.x - robot_pose.position!.x, 2) +
                  Math.pow(mouseTarget.y - robot_pose.position!.y, 2)
              ).toFixed(3)}
            m
          </Body3>
        </VStack>
        {robot_pose && (
          <>
            <Line2Point
              from={{
                x: robot_pose.position!.x,
                y: robot_pose.position!.y,
              }}
              to={{
                direction:
                  180 -
                  ((robot_pose.orientationEuler?.yaw ?? 0) * 180) / Math.PI,
                distance: 1,
              }}
              scale={scale}
              color="black"
            />

            {create3Action === "navigate" && (
              <Line2Point
                from={{
                  x: robot_pose.position!.x,
                  y: robot_pose.position!.y,
                }}
                to={{
                  x: mouseTarget.x,
                  y: mouseTarget.y,
                }}
                scale={scale}
                color="red"
              />
            )}

            {create3Action === "drive" && (
              <Line2Point
                from={{
                  x: robot_pose.position!.x,
                  y: robot_pose.position!.y,
                }}
                to={{
                  x: mouseTarget.x,
                  y: mouseTarget.y,
                  direction:
                    180 -
                    ((robot_pose.orientationEuler?.yaw ?? 0) * 180) / Math.PI,
                }}
                scale={scale}
                color="green"
              />
            )}

            {create3Action === "rotate" && (
              <Line2Point
                from={{
                  x: robot_pose.position!.x,
                  y: robot_pose.position!.y,
                }}
                to={{
                  x: mouseTarget.x,
                  y: mouseTarget.y,
                }}
                scale={scale}
                color="yellow"
              />
            )}

            <Flex
              style={{
                position: "absolute",
                left: `${50 + (mouseTarget.x * 100) / scale}%`,
                top: `${50 + (mouseTarget.y * 100) / scale}%`,
                transform: `translate(-50%, -50%) rotate(${
                  (Math.atan2(
                    mouseTarget.y - robot_pose.position!.y,
                    mouseTarget.x - robot_pose.position!.x
                  ) *
                    180) /
                  Math.PI
                }deg)`,
                transformOrigin: "10px 10px",
                opacity: 0.7,
              }}
            >
              <Img src={RobotPon} width="20px" height="20px" />
            </Flex>
          </>
        )}
      </Flex>{" "}
    </VStack>
  );
});

export const Create3SlamView = observer(() => {
  return (
    <VStack
      style={{
        width: "100%",
      }}
    >
      <HStack>
        <Btn color={Color.Red} onClick={() => slamStore.fetchCreate3Estop()}>
          {" "}
          E-STOP{" "}
        </Btn>
        <Btn
          color={Color.Cyan}
          onClick={() => slamStore.fetchCreate3EstopRelease()}
        >
          {" "}
          E-STOP Release{" "}
        </Btn>
        <Btn color={Color.Green} onClick={() => slamStore.create3ResetPose()}>
          Reset Pose
        </Btn>
      </HStack>

      <Create3Map />
    </VStack>
  );
});
