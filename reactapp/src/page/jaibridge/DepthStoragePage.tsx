import { observer } from "mobx-react-lite";
import { PageRoot } from "../../design/other/flexs";
import {
  Button,
  Flex,
  HStack,
  Menu,
  MenuButton,
  MenuItem,
  MenuList,
  Progress,
  ProgressLabel,
  Switch,
  VStack,
} from "@chakra-ui/react";
import {
  ChevronDownIcon,
  ChevronLeftIcon,
  ChevronRightIcon,
} from "@chakra-ui/icons";
import { FrameInfo, jaiDepthStore as jds } from "../../stores/JaiDepthStore";
import { useCallback, useEffect, useState } from "react";
import { Body3, H2, H3, H4 } from "../../design/text/textsystem";
import { Camera, TrafficSignal } from "@phosphor-icons/react";
import { Btn } from "../../design/button/button";
import { jaiStore } from "../../stores/JaiStore";

const SceneLoadDropdown = ({ sceneList }: { sceneList: string[] }) => {
  useEffect(() => {
    jds.fetchStorageList();
  }, []);

  return (
    <Menu placement="bottom-end">
      <MenuButton as={Button} rightIcon={<ChevronDownIcon />}>
        {jds.scene_id || "Load Scene"}
      </MenuButton>
      <MenuList>
        {sceneList.map((scene, index) => {
          return (
            <MenuItem
              key={index}
              fontSize="small"
              onClick={() => {
                jds.fetchStorageFrames(scene);
              }}
            >
              {scene}
            </MenuItem>
          );
        })}
      </MenuList>
    </Menu>
  );
};

const LoadCalibrationButton = observer(() => {
  useEffect(() => {
    jaiStore.fetchGetCalibrationList();
  }, []);

  return (
    <Menu placement="bottom-end">
      <MenuButton as={Button} rightIcon={<ChevronDownIcon />}>
        Load Calibration
      </MenuButton>
      <MenuList>
        {jaiStore.calibrationList.map((meta, index) => {
          return (
            <MenuItem
              key={index}
              fontSize="small"
              onClick={() => {
                jds.fetchSceneRequestLoadCalibration(jds.scene_id!, meta.id);
              }}
            >
              {meta.id} : {meta.month}/{meta.day}/{meta.hour}
            </MenuItem>
          );
        })}
      </MenuList>
    </Menu>
  );
});

const FrameDetailLidar = ({ frame }: { frame: FrameInfo }) => {
  useEffect(() => {}, [frame]);
  return (
    <VStack>
      <Body3>{frame.frame_id}</Body3>
      {frame.lidar_path || <H2>No Lidar</H2>}
      {frame.lidar_path && (
        <>
          {frame.lidar_ply_path || (
            <Flex>
              <Btn
                onClick={() => {
                  jds.fetchProcessCreatePly(frame.frame_id, () => {});
                }}
              >
                Create Ply
              </Btn>
            </Flex>
          )}
          {frame.lidar_ply_path && (
            <Flex>
              <Btn
                onClick={() => {
                  jds.fetchDownloadFile(frame.frame_id, "camera_rgb.ply");
                }}
              >
                Download Camera Ply
              </Btn>
              <Btn
                onClick={() => {
                  jds.fetchDownloadFile(frame.frame_id, "lidar.ply");
                }}
              >
                Download Lidar Ply
              </Btn>
            </Flex>
          )}
        </>
      )}
    </VStack>
  );
};

const FrameDetail = ({ frame }: { frame: string }) => {
  const [frameInfo, setFrameInfo] = useState<FrameInfo | undefined>(undefined);

  const [frameDisplay, setFrameDisplay] = useState<string[]>([]);
  useEffect(() => {
    loadFrame();
  }, [frame]);

  const loadFrame = useCallback(() => {
    jds.getFrameInfo(frame, (frame_out) => {
      setFrameInfo(frame_out);
      if (frame_out?.capture_rgb.disparity_viz_path) {
        setFrameDisplay(["left", "right", "disparity"]);
      } else {
        setFrameDisplay(["left", "right"]);
      }
    });
  }, [frame]);

  return (
    <VStack>
      <H3>{frame}</H3>
      <HStack>
        <VStack>
          {["rgb", "nir"].map((channel) => (
            <HStack>
              {frameDisplay.map((prop) => (
                <VStack>
                  <Body3>{prop}</Body3>
                  <img
                    src={`/jai/stereo/storage/${
                      jds.scene_id
                    }/frame/${frame}/png/${prop}/${channel}?root=${jds.storage_url()}`}
                    style={{
                      width: "20rem",
                      height: "15rem",
                    }}
                    alt={frame}
                  />
                </VStack>
              ))}
            </HStack>
          ))}
        </VStack>
        {frameInfo && !frameInfo.capture_nir.disparity_viz_path && (
          <Flex>
            <Btn
              onClick={() => {
                jds.fetchProcessDisparity(frameInfo.frame_id, () => {
                  loadFrame();
                });
              }}
            >
              Create Disparity
            </Btn>
          </Flex>
        )}
        {frameInfo && <FrameDetailLidar frame={frameInfo} />}
      </HStack>{" "}
    </VStack>
  );
};

const FrameThumb = ({ frame }: { frame: string }) => {
  const [frameInfo, setFrameInfo] = useState<FrameInfo | undefined>(undefined);

  useEffect(() => {
    jds.getFrameInfo(frame, setFrameInfo);
  }, [frame]);

  return (
    <VStack
      style={{
        width: "10rem",
        height: "10rem",
      }}
      onClick={() => {
        jds.frame_id = frame;
      }}
    >
      <img
        src={`/jai/stereo/storage/${
          jds.scene_id
        }/frame/${frame}/png/left/rgb?root=${jds.storage_url()}`}
        style={{
          width: "8rem",
          height: "6rem",
        }}
        alt={frame}
      />
      <HStack>
        {frameInfo?.capture_rgb.disparity_viz_path && (
          <Camera
            style={{
              color: "red",
            }}
          />
        )}
        {frameInfo?.capture_nir.disparity_viz_path && (
          <Camera
            style={{
              color: "gray",
            }}
          />
        )}
        {frameInfo?.lidar_path && <TrafficSignal />}
      </HStack>
    </VStack>
  );
};

const FramesList = observer(() => {
  const [page, setPage] = useState(0);

  return (
    <HStack
      style={{
        overflowX: "auto",
        width: "100%",
      }}
    >
      {page > 0 && (
        <Btn
          size="sm"
          icon={<ChevronLeftIcon />}
          onClick={() => setPage(page - 1)}
        />
      )}
      <HStack style={{}}>
        {jds.scene_frame_list
          .slice(page * 10, page * 10 + 10)
          .map((frame, index) => {
            return <FrameThumb key={index} frame={frame} />;
          })}
      </HStack>
      {page < Math.floor(jds.scene_frame_list.length / 10) && (
        <Btn
          size="sm"
          icon={<ChevronRightIcon />}
          onClick={() => setPage(page + 1)}
        />
      )}
    </HStack>
  );
});

const SceneRootTool = observer(() => {
  return (
    <HStack>
      <Switch
        checked={jds.remote_storage}
        onChange={(e) => {
          jds.remote_storage = e.target.checked;
          jds.fetchStorageList();
        }}
      >
        {jds.remote_storage ? "Remote" : "Local"}
      </Switch>
      <SceneLoadDropdown sceneList={jds.scene_list} />
      {jds.scene_frame_list.length > 0 && (
        <H4>{jds.scene_frame_list.length} Frames</H4>
      )}
      {jds.scene_id && jds.scene_calibration === undefined && (
        <LoadCalibrationButton />
      )}
      <Btn size="sm" onClick={() => jds.fetchNpzH5Post()}>
        npz to h5
      </Btn>
      {jds.progress && (
        <HStack>
          <Progress
            style={{ width: "20rem", height: "2rem" }}
            value={jds.progress * 100}
          />
          <Body3>{(jds.progress * 100).toFixed(2)}% Complete</Body3>
        </HStack>
      )}
    </HStack>
  );
});

export const DepthStoragePage = observer(() => {
  return (
    <PageRoot title="Depth Storage">
      <SceneRootTool />
      <FramesList />
      {jds.frame_id && <FrameDetail frame={jds.frame_id} />}
    </PageRoot>
  );
});
