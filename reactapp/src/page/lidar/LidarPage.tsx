import { Flex, FlexProps, Image, VStack } from "@chakra-ui/react";
import { TopicItem } from "../topic/TopicPage";
import { PageRoot } from "../../design/other/flexs";
import { useEffect, useRef, useState } from "react";
import { rosSocket } from "../../connect/socket/subscribe";
import { Body3 } from "../../design/text/textsystem";

export interface LidarScan {
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
  ranges: number[];
}

const LidarRender = (props: FlexProps) => {
  const flexRef = useRef<HTMLDivElement>(null);

  const [liderScan, setLidarScan] = useState<LidarScan>({
    angle_min: 0,
    angle_max: 0,
    angle_increment: 0,
    time_increment: 0,
    scan_time: 0,
    range_min: 0,
    range_max: 0,
    ranges: [],
  });

  return (
    <Flex
      {...props}
      ref={flexRef}
      style={{
        ...props.style,
        background: "#E3F5FF",
      }}
    >
      <Body3>{liderScan.ranges.length}</Body3>
      {liderScan.ranges.map((range, idx) => {
        return (
          <LidarPoint
            angle_min={liderScan.angle_min}
            angle_max={liderScan.angle_max}
            angle_increment={liderScan.angle_increment}
            idx={idx}
            range={range}
            range_max={liderScan.range_max}
            width={flexRef.current?.offsetWidth || 0}
            height={flexRef.current?.offsetHeight || 0}
          />
        );
      })}
    </Flex>
  );
};

const LidarPoint = ({
  angle_min,
  angle_max,
  angle_increment,
  idx,
  range_max,
  range,
  width,
  height,
}: {
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  idx: number;
  range: number;
  range_max: number;
  width: number;
  height: number;
}) => {
  const angle = angle_min + angle_increment * idx;
  const x = ((Math.cos(angle) * range) / range_max) * width;
  const y = ((Math.sin(angle) * range) / range_max) * height;
  return (
    <div
      style={{
        position: "fixed",
        left: `${x}px`,
        top: `${y}px`,
        width: "2px",
        height: "2px",
        background: "black",
        borderRadius: "1px",
      }}
    />
  );
};

export const LidarBlock = () => {
  return (
    <VStack>
      <TopicItem
        topic={{
          topic: "/scan",
          type: "sensor_msgs/LaserScan",
          running: true,
        }}
      />
      <LidarRender
        style={{
          width: "600px",
          height: "600px",
        }}
      />
    </VStack>
  );
};

export const LidarPage = () => {
  return (
    <PageRoot title={"Lidar"}>
      <Image src="/ros/video/lidar" width="600px" height="600px" />
    </PageRoot>
  );
};
