import React, { useEffect, useRef, useState } from "react";
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
import { MapData, MapListModalBtn } from "./MapList";
import { MapSaveBtn } from "./MapSaveBtn";
import { slamStore } from "../../stores/SlamStore";
import { observer } from "mobx-react";

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
      {slamStatus?.slam_metadata && (
        <>
          <InfoCard
            title="Position Interval"
            value={`${slamStatus.slam_metadata.pos_interval.toFixed(4)}`}
            color={Color.Cyan}
          />
          <InfoCard
            title="Map Interval"
            value={`${slamStatus.slam_metadata.map_interval.toFixed(4)}`}
            color={Color.Cyan}
          />
          <InfoCard
            title="Lidar Interval"
            value={`${slamStatus.slam_metadata.lidar_interval.toFixed(4)}`}
            color={Color.Cyan}
          />
        </>
      )}
    </Flex>
  );
};

export const SlamZoomFlex = (props: { children: React.ReactNode }) => {
  const ZOOM_RANGE = [1, 10];
  const [zoomScale, setZoomScale] = useState(1);
  const [zoomOrigin, setZoomOrigin] = useState({ x: 0, y: 0 });

  const [rootSize, setRootSize] = useState({ width: 0, height: 0 });

  const [adjustPad, setAdjustPad] = useState({ x: 0, y: 0 });

  useEffect(() => {
    const rect = document.getElementById("zoom-root")!.getBoundingClientRect();
    const x = rect.right < rootSize.width ? rootSize.width - rect.right : 0;
    const y = rect.bottom < rootSize.height ? rootSize.height - rect.bottom : 0;
    setAdjustPad({ x, y });
  }, [zoomScale, zoomOrigin, rootSize.width, rootSize.height]);

  const rootRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (rootRef.current) {
      const { width, height } = rootRef.current.getBoundingClientRect();
      setRootSize({ width, height });
    }
  }, [rootRef.current]);

  return (
    <Flex
      style={{
        overflow: "hidden",
        width: "100%",
        height: "auto",
      }}
      ref={rootRef}
    >
      <Flex
        id="zoom-root"
        style={{
          width: "100%",
          transform: `scale(${zoomScale})`,
          transformOrigin: `${zoomOrigin.x - adjustPad.x / 2}px ${
            zoomOrigin.y - adjustPad.y / 2
          }px`,
          position: "relative",

          transition: "transform 0.1s",
        }}
        onWheel={(e) => {
          if (e.shiftKey) {
            e.preventDefault();
            const delta = e.deltaY;
            const scale = zoomScale;
            const origin = zoomOrigin;
            const rect = e.currentTarget.getBoundingClientRect();
            const x = e.clientX - (rect.left + rect.right) / 2;
            const y = e.clientY - (rect.top + rect.bottom) / 2;
            const factor = 0.1;
            const newScale = Math.min(
              Math.max(scale + (delta > 0 ? -1 : 1) * factor, ZOOM_RANGE[0]),
              ZOOM_RANGE[1]
            );
            const newOrigin = {
              x: Math.min(
                Math.max(x * (scale - newScale / scale) + origin.x, 0),
                (rect.width * newScale) / scale - rootRef.current!.clientWidth
              ),
              y: Math.min(
                Math.max(y * (scale - newScale / scale) + origin.y, 0),
                (rect.height * newScale) / scale - rootRef.current!.clientHeight
              ),
            };
            setZoomScale(newScale);
            setZoomOrigin(newOrigin);
          }
        }}
      >
        {props.children}
      </Flex>
    </Flex>
  );
};

export const SlamMap = observer(
  ({
    children,
    savedMapName,
  }: {
    children?: React.ReactNode;
    savedMapName?: string;
  }) => {
    const { map_origin, map_size } =
      (savedMapName ? slamStore.mapMetadataView : slamStore.slamStatus) || {};

    const [videoSize, setVideoSize] = useState<{
      width: number;
      height: number;
    }>({
      width: 0,
      height: 0,
    });

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
      if (savedMapName) {
        slamStore.fetchGetMapMetadata(savedMapName);
      }
    }, [savedMapName]);

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
        {savedMapName ? (
          <img
            src={`/slam/map/${savedMapName}/image`}
            id="video-stream"
            width="100%"
            height="auto"
            style={{
              rotate: "180deg",
            }}
            alt=""
          />
        ) : (
          <VideoStream
            id="video-stream"
            url={"/slam/map/stream"}
            width="100%"
            height="auto"
            play={true}
          />
        )}

        <Body3
          style={{
            position: "absolute",
            left: 0,
            top: 0,
          }}
        >{`Map Layout Size: ${videoSize.width.toFixed(
          2
        )} x ${videoSize.height.toFixed(2)}`}</Body3>
        {map_origin && map_size && (
          <>
            {React.Children.map(children, (child, index) => {
              return (
                React.isValidElement(child) &&
                React.cloneElement(
                  child as React.ReactElement<SlamMapMarkerProps>,
                  {
                    x:
                      ((map_size.width -
                        (child.props.x - map_origin!.x) / map_size.resolution) *
                        videoSize.width) /
                      map_size.width,
                    y:
                      (((child.props.y - map_origin!.y) / map_size.resolution) *
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
  }
);

export const SlamView = observer(() => {
  const slamStatus = slamStore.slamStatus;
  const slamRobotPose = slamStore.slamRobotPose;
  const markers = slamStore.markers;

  const fetchLoadMap = (map: MapData) => {
    if (slamStatus?.status !== "success") {
      return false;
    }
    httpPost("/slam/map/load", {
      filename: map.name,
    }).fetch();
    return true;
  };

  const fetchSlamStart = () => {
    httpGet("/slam/launch").fetch();
  };

  const fetchSlamStop = () => {
    httpGet("/slam/cancel").fetch();
  };

  const fetchAddMarkerSelf = () => {
    httpPost("/slam/map/add_marker_self", {}).fetch();
  };

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
        <MapListModalBtn
          onClickMapWithCallback={(map, closeCallback) => {
            if (fetchLoadMap(map)) closeCallback();
          }}
        />
        <MapSaveBtn />
      </HStack>

      <SlamMap>
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
                  180 - ((slamRobotPose?.orientation.yaw || 0) * 180) / Math.PI
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
});

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
