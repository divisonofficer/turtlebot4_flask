import { HTMLProps, useEffect, useRef, useState } from "react";
import { MapSize, SlamRobotPose } from "./SlamType";
import { ArrowCircleRight, ArrowRight } from "@phosphor-icons/react";
import { Color } from "../../design/color";
import { Flex, VStack } from "@chakra-ui/react";
import { Body3 } from "../../design/text/textsystem";
import { Point3D, Pose3D } from "../../public/proto/slam";

export interface SlamMapMarkerProps extends HTMLProps<HTMLDivElement> {
  x?: number;
  y?: number;
  map_size?: MapSize;
  origin?: { x: number; y: number };
}

export const SlamMapMarker = (props: SlamMapMarkerProps) => {
  const { x, y, style, children, ...rest } = props;
  const divRef = useRef<HTMLDivElement>(null);
  const [divSize, setDivSize] = useState<{ width: number; height: number }>({
    width: 0,
    height: 0,
  });

  useEffect(() => {
    if (divRef.current) {
      const { width, height } = divRef.current.getBoundingClientRect();
      setDivSize({ width, height });
    }
  }, []);

  const adjustedLeft = x! - divSize.width / 2;
  const adjustedTop = y! - divSize.height / 2;
  return (
    <div
      {...rest}
      ref={divRef}
      style={{
        ...style,
        position: "absolute",
        left: adjustedLeft,
        top: adjustedTop,
        overflow: "show",
      }}
    >
      {children}
    </div>
  );
};

export const SlamMarkerHover = (
  props: SlamMapMarkerProps & {
    pose: {
      position?: Point3D;
      orientationEuler?: { roll: number; pitch: number; yaw: number };
    };
    _id: number;
  }
) => {
  const [hover, setHover] = useState(false);
  const { pose } = props;
  return (
    <SlamMapMarker {...props}>
      <Flex
        style={{
          width: "2rem",
          height: "2rem",
        }}
      >
        <ArrowCircleRight
          style={{
            zIndex: 101,
            width: "2rem",
            height: "2rem",
            color: Color.Green,
            rotate: `${
              180 - ((props.pose.orientationEuler?.roll || 0) * 180) / Math.PI
            }deg`,
          }}
          onMouseEnter={() => setHover(true)}
          onMouseMove={() => setHover(true)}
          onMouseLeave={() => setHover(false)}
        />
        {hover && (
          <VStack
            style={{
              position: "absolute",
              width: "10rem",
              background: Color.Cyan + "A0",
              zIndex: 100,
              alignItems: "flex-start",
              borderRadius: "0.5rem",
              padding: "0.5rem",
              marginTop: "2rem",
            }}
          >
            <Body3 textColor={"white"}>ID {props._id}</Body3>
            <Body3>
              X {pose.position!.x.toFixed(4)} Y {pose.position!.y.toFixed(4)}
            </Body3>
          </VStack>
        )}
      </Flex>
    </SlamMapMarker>
  );
};
