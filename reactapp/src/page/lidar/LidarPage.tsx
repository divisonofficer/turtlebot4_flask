import { Flex, FlexProps, Image, VStack } from "@chakra-ui/react";
import { TopicItem } from "../topic/TopicPage";
import { PageRoot } from "../../design/other/flexs";
import { useEffect, useRef, useState } from "react";
import { rosSocket } from "../../connect/socket/subscribe";
import { Body3 } from "../../design/text/textsystem";
import { VideoStream } from "../../design/other/video";

export interface LidarScan {
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
}

export const LidarBlock = () => {
  const [scan, setScan] = useState<LidarScan>({
    angle_min: 0,
    angle_max: 0,
    angle_increment: 0,
    range_min: 0,
    range_max: 0,
  });

  useEffect(() => {
    rosSocket.subscribe("/lidar/scan", (msg: LidarScan) => {
      setScan(msg);
    });
  }, []);
  return (
    <VStack>
      <Body3>angle_min: {scan.angle_min}</Body3>
      <Body3>angle_max: {scan.angle_max}</Body3>
      <Body3>angle_increment: {scan.angle_increment}</Body3>
      <Body3>range_min: {scan.range_min}</Body3>
      <Body3>range_max: {scan.range_max}</Body3>
    </VStack>
  );
};

export const LidarPage = () => {
  return (
    <PageRoot title={"Lidar"}>
      <VideoStream url="/ros/video/lidar" width="600px" height="600px" />
      <LidarBlock />
    </PageRoot>
  );
};
