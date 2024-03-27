import React, { useEffect, useState } from "react";
import { PageRoot } from "../../design/other/flexs";
import { VideoStream } from "../../design/other/video";
import { slamSocket } from "../../connect/socket/subscribe";
import {
  Flex,
  HStack,
  Modal,
  ModalBody,
  ModalCloseButton,
  ModalContent,
  ModalHeader,
  ModalOverlay,
  Textarea,
  VStack,
  useDisclosure,
} from "@chakra-ui/react";
import { Btn } from "../../design/button/button";
import { Color } from "../../design/color";
import { httpGet, httpPost } from "../../connect/http/request";
import { Body3 } from "../../design/text/textsystem";
import { InfoCard } from "../../design/other/infocard";
import { ArrowArcLeft, ArrowRight } from "@phosphor-icons/react";
import { MapSize, SlamRobotPose, SlamStatus } from "./SlamType";
import {
  SlamMapMarker,
  SlamMapMarkerProps,
  SlamMarkerHover,
} from "./MapMarker";

const SlamPostionInfo = ({
  slamStatus,
  robotPose,
}: {
  slamStatus?: SlamStatus;
  robotPose?: SlamRobotPose;
}) => {
  const robot_pose = {
    x: robotPose?.x.toFixed(2),
    y: robotPose?.y.toFixed(2),
  };

  const robotOrientation = {
    roll: robotPose?.orientation?.roll.toFixed(2),
    pitch: robotPose?.orientation?.pitch.toFixed(2),
    yaw: robotPose?.orientation?.yaw.toFixed(2),
  };

  const robotOrientationQuat = {
    x: robotPose?.orientation_quaternion?.x.toFixed(2),
    y: robotPose?.orientation_quaternion?.y.toFixed(2),
    z: robotPose?.orientation_quaternion?.z.toFixed(2),
    w: robotPose?.orientation_quaternion?.w.toFixed(2),
  };
  const map_origin = {
    x: slamStatus?.map_origin?.x.toFixed(2),
    y: slamStatus?.map_origin?.y.toFixed(2),
  };

  const map_size = {
    width: slamStatus?.map_size?.width.toFixed(2),
    height: slamStatus?.map_size?.height.toFixed(2),
  };

  return (
    <Flex wrap="wrap" gap={2}>
      <InfoCard
        title="Robot Pose"
        value={`${robot_pose.x}, ${robot_pose.y}`}
        color={Color.Yellow}
      />

      <InfoCard
        title="Map Origin"
        value={`${map_origin.x}, ${map_origin.y}`}
        color={Color.Indigo}
      />
      <InfoCard
        title="Map Size"
        value={`${map_size.width}, ${map_size.height}`}
        color={Color.Orange}
      />
      <InfoCard
        title="Map Resolution"
        value={`${slamStatus?.map_size?.resolution.toFixed(4)}`}
        color={Color.Purple}
      />

      <InfoCard
        title="Orientation"
        value={`${robotOrientation.roll}, ${robotOrientation.pitch}, ${robotOrientation.yaw}`}
        color={Color.Blue}
      />

      <InfoCard
        title="Orientation Quaternion"
        value={`${robotOrientationQuat.x}, ${robotOrientationQuat.y}, ${robotOrientationQuat.z}, ${robotOrientationQuat.w}`}
        color={Color.Green}
      />
    </Flex>
  );
};

export const SlamMap = ({
  origin,
  map_size,
  children,
}: {
  children?: React.ReactNode;
  map_size?: MapSize;
  origin?: { x: number; y: number };
}) => {
  const [videoSize, setVideoSize] = useState<{ width: number; height: number }>(
    {
      width: 0,
      height: 0,
    }
  );

  const handleVideoResize = () => {
    const videoElement = document.getElementById("video-stream");
    if (videoElement) {
      const { width, height } = videoElement.getBoundingClientRect();
      setVideoSize({
        width,
        height:
          height > 0
            ? height
            : (width * (map_size?.height || 1)) / (map_size?.width || 1),
      });
    }
  };

  useEffect(() => {
    handleVideoResize();
    window.addEventListener("resize", handleVideoResize);
    return () => {
      window.removeEventListener("resize", handleVideoResize);
    };
  }, []);

  return (
    <Flex
      style={{
        background: Color.Cyan,
        width: "100%",
        position: "relative",
      }}
    >
      <VideoStream
        id="video-stream"
        url="/slam/map/stream"
        width="100%"
        height="100%"
      />
      <Body3
        style={{
          position: "absolute",
          left: 0,
          top: 0,
        }}
      >{`Map Layout Size: ${videoSize.width.toFixed(
        2
      )} x ${videoSize.height.toFixed(2)}`}</Body3>
      {origin && map_size && (
        <>
          {React.Children.map(children, (child, index) => {
            return (
              React.isValidElement(child) &&
              React.cloneElement(
                child as React.ReactElement<SlamMapMarkerProps>,
                {
                  x:
                    ((map_size.width -
                      (child.props.x - origin!.x) / map_size.resolution) *
                      videoSize.width) /
                    map_size.width,
                  y:
                    (((child.props.y - origin!.y) / map_size.resolution) *
                      videoSize.width) /
                    map_size.width,
                }
              )
            );
          })}
        </>
      )}
    </Flex>
  );
};

export const SlamView = () => {
  const [slamStatus, setSlamStatus] = useState<SlamStatus>();
  const [slamRobotPose, setSlamRobotPose] = useState<SlamRobotPose>();
  const [markers, setMarkers] = useState<{ id: number; pose: SlamRobotPose }[]>(
    []
  );

  const fetchSlamStart = () => {
    httpGet("/slam/launch").fetch();
  };

  const fetchSlamStop = () => {
    httpGet("/slam/cancel").fetch();
  };

  const fetchAddMarkerSelf = () => {
    httpPost("/slam/map/add_marker_self", {}).fetch();
  };

  useEffect(() => {
    slamSocket.subscribe("slam_status", (data: SlamStatus) => {
      setSlamStatus(data);
      if (data.markers) {
        setMarkers(data.markers);
      }
    });

    slamSocket.subscribe("robot_pose", (data: SlamRobotPose) => {
      setSlamRobotPose(data);
    });

    slamSocket.subscribe(
      "markers",
      (data: { id: number; pose: SlamRobotPose }[]) => {
        setMarkers(data);
      }
    );
  }, []);
  return (
    <VStack width="100%">
      <SlamPostionInfo slamStatus={slamStatus} robotPose={slamRobotPose} />
      <HStack>
        {slamStatus?.status === "success" ? (
          <Btn onClick={fetchSlamStop} color={Color.Red}>
            Stop
          </Btn>
        ) : (
          <Btn onClick={fetchSlamStart} color={Color.Green}>
            Start
          </Btn>
        )}

        <Btn onClick={fetchAddMarkerSelf} color={Color.Blue}>
          Add Self Marker
        </Btn>
      </HStack>

      <SlamMap origin={slamStatus?.map_origin} map_size={slamStatus?.map_size}>
        {slamStatus?.robot_pose && (
          <SlamMapMarker
            x={slamRobotPose?.x || 0}
            y={slamRobotPose?.y || 0}
            style={{
              position: "absolute",
            }}
          >
            <ArrowRight
              style={{
                width: "2rem",
                height: "2rem",
                color: Color.Red,
                rotate: `${
                  180 - ((slamRobotPose?.orientation.roll || 0) * 180) / Math.PI
                }deg`,
              }}
            />
          </SlamMapMarker>
        )}
        {markers.map((marker, index) => (
          <SlamMarkerHover
            key={index}
            pose={marker.pose}
            x={marker.pose.x}
            y={marker.pose.y}
            _id={marker.id}
          />
        ))}
      </SlamMap>
      <SlamMapJsonFetch />
    </VStack>
  );
};

const SlamMapJsonFetch = () => {
  const { isOpen, onOpen, onClose } = useDisclosure();

  const [mapJson, setMapJson] = useState<any>({});

  const fetchMapJson = () => {
    httpGet("/slam/map/data")
      .onSuccess((data: any) => {
        setMapJson(data);
      })
      .fetch();
  };
  return (
    <Flex>
      <Btn
        onClick={() => {
          onOpen();
          fetchMapJson();
        }}
        color={Color.Cyan}
      >
        Map Json
      </Btn>
      {isOpen && (
        <Modal isOpen={isOpen} onClose={onClose} size="md">
          <ModalOverlay />
          <ModalContent>
            <ModalHeader>
              <Flex>
                <Btn
                  onClick={fetchMapJson}
                  color={Color.Cyan}
                  icon={<ArrowArcLeft />}
                  size={"md"}
                >
                  Fetch
                </Btn>
              </Flex>
            </ModalHeader>
            <ModalCloseButton />
            <ModalBody>
              <Textarea
                value={JSON.stringify(mapJson)}
                isReadOnly
                height="20rem"
              />
            </ModalBody>
          </ModalContent>
        </Modal>
      )}
    </Flex>
  );
};

export const SlamPage = () => {
  return (
    <PageRoot title={"SLAM"}>
      <SlamView />
    </PageRoot>
  );
};
