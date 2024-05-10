import { Flex, FlexProps, VStack } from "@chakra-ui/react";
import { SlamMap, SlamZoomFlex } from "../slam/SlamPage";
import { SlamMapMarker } from "../slam/MapMarker";
import { observer } from "mobx-react";
import { slamStore } from "../../stores/SlamStore";
import { captureStore } from "../../stores/CaptureStore";
import { Color } from "../../design/color";

const CameraAngleIcon = (
  props: FlexProps & {
    color?: string;
    widthAngle?: number;
    width?: number;
  }
) => {
  const widthAngle = props.widthAngle || 50;
  const color = props.color || "black";
  const width = props.width || 30;

  return (
    <Flex
      {...props}
      style={{
        ...props.style,
        position: "relative",
      }}
    >
      <div
        style={{
          width: `${width}px`,
          height: "2px",
          background: color,
          position: "absolute",

          top: `${
            (-Math.sin((widthAngle / 2) * (Math.PI / 180)) * width) / 2
          }px`,
          transform: `rotate(-${widthAngle / 2}deg)`,
        }}
      />
      <div
        style={{
          width: `${width}px`,

          top: `${
            (Math.sin((widthAngle / 2) * (Math.PI / 180)) * width) / 2
          }px`,
          height: "2px",
          background: color,
          position: "absolute",
          transform: `rotate(${widthAngle / 2}deg)`,
        }}
      />
      <div
        style={{
          width: "20px",
          height: "10px",
          background: color,
          opacity: 0.5,
          position: "absolute",
          left: "-25px",
          top: "-5px",
          objectFit: "cover",
        }}
      />
    </Flex>
  );
};

export const CaptureMap = observer(() => {
  const robotPose = slamStore.slamRobotPose;
  const captureScenes = captureStore.captureScenes;
  return (
    <VStack
      style={{
        width: "100%",
      }}
    >
      <SlamZoomFlex>
        <SlamMap
          savedMapName={
            captureStore.is_capture_running ? undefined : captureStore.map_name
          }
        >
          {robotPose && (
            <SlamMapMarker x={robotPose.x} y={robotPose.y}>
              <CameraAngleIcon
                style={{
                  transform: `rotate(${
                    (-(robotPose?.orientation.yaw || 0) * 180) / Math.PI + 180
                  }deg)`,
                }}
              />
            </SlamMapMarker>
          )}

          {captureScenes.map((scene, index) => (
            <SlamMapMarker
              key={index}
              x={scene.robotPose?.position?.x}
              y={scene.robotPose?.position?.y}
              style={{
                transform: `rotate(${
                  (-(scene.robotPose?.orientation?.yaw || 0) * 180) / Math.PI +
                  180
                }deg)`,
              }}
            >
              <CameraAngleIcon
                color={
                  scene.captureId === captureStore.map_focused_capture_id &&
                  scene.sceneId === captureStore.map_focused_scene_id
                    ? Color.Red
                    : Color.Cyan
                }
              />
            </SlamMapMarker>
          ))}
        </SlamMap>
      </SlamZoomFlex>
    </VStack>
  );
});
