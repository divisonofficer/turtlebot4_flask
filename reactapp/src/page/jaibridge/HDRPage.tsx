import {
  HStack,
  Switch,
  VStack,
  CircularProgress,
  Flex,
  Slider,
  SliderTrack,
  SliderFilledTrack,
  SliderThumb,
  IconButton,
  Grid,
  Box,
  Image,
  Text,
  Select,
  Divider,
} from "@chakra-ui/react";
import { PageRoot } from "../../design/other/flexs";

import { Body2, H4 } from "../../design/text/textsystem";
import { observer } from "mobx-react";

import { Fragment, useEffect } from "react";
import { InfoCard, InfoCardBtn } from "../../design/other/infocard";
import { jaiHDRStore, JaiHDRLog } from "../../stores/JaiHDRStore";
import { Camera } from "@phosphor-icons/react/dist/ssr";
import {
  CameraRotate,
  GooglePhotosLogo,
  Pause,
  Play,
  Steps,
  Stop,
  Trash,
} from "@phosphor-icons/react";
import { Color } from "../../design/color";

const ErrorCard = ({ error }: { error: JaiHDRLog.Error }) => {
  if (error.type === "error_process_stream_expire") {
    return (
      <InfoCard
        title={"Expired"}
        value={
          error.data.device +
          "/" +
          error.data.stream +
          ":" +
          error.data.expired_time
        }
        color={"yellow"}
      />
    );
  }

  if (error.type === "error_retrieve_buffer") {
    return (
      <InfoCard
        title={"Retrieve fail"}
        value={
          error.data.lResult === "OK" ? error.data.aResult : error.data.lResult
        }
        color={"yellow"}
      />
    );
  }

  if (error.type === "error_crash") {
    return <InfoCard title={"Crash"} value={error.data.cause} color={"red"} />;
  }

  return (
    <InfoCard
      title={error.type}
      value={JSON.stringify(error.data)}
      color={"gray"}
    />
  );
};

const JaiOptionSwitch = ({
  option_id,
  option_name,
  checked,
}: {
  option_id: string;
  option_name: string;
  checked: boolean | undefined;
}) => {
  const isMobile = window.innerWidth < 768;
  return (
    <HStack>
      {isMobile ? <Body2>{option_name}</Body2> : <H4>{option_name}</H4>}

      <Switch
        isChecked={checked}
        onChange={(e) => {
          jaiHDRStore.fetchUpdateConfig(option_id, e.target.checked);
        }}
      />
    </HStack>
  );
};

const OptionEnum = ({
  option_id,
  option_name,
  value,
  value_list,
}: {
  option_id: string;
  option_name: string;
  value: string;
  value_list: string[];
}) => {
  return (
    <HStack>
      <H4>{option_name}</H4>
      <select
        value={value}
        onChange={(e) => {
          jaiHDRStore.fetchUpdateConfig(option_id, e.target.value);
        }}
      >
        {value_list.map((v) => (
          <option key={v} value={v}>
            {v}
          </option>
        ))}
      </select>
    </HStack>
  );
};

const JaiHDRControls = ({ isMobile }: { isMobile: boolean }) => {
  const params_drive = [
    {
      range: [15, 180],
      step: 1,
      name: "rotate_angle",
      value: jaiHDRStore.hdr_config.rotate_angle,
    },
    {
      range: [1, 20],
      step: 1,
      name: "capture_cnt",
      value: jaiHDRStore.hdr_config.capture_cnt,
    },
    {
      range: [1, 10],
      step: 1,
      name: "side_move_cnt",
      value: jaiHDRStore.hdr_config.side_move_cnt,
    },
    {
      range: [-0.2, 0.2],
      step: 0.01,
      name: "side_move_distance",
      value: jaiHDRStore.hdr_config.side_move_distance,
    },
  ];

  const params_arc = [
    {
      range: [0, 90],
      step: 1,
      name: "arc_angle",
      value: jaiHDRStore.hdr_config.arc_angle,
    },
    {
      range: [0, 2.5],
      step: 0.05,
      name: "arc_radius",
      value: jaiHDRStore.hdr_config.arc_radius,
    },
  ];
  const params = [
    ...(jaiHDRStore.hdr_config.drive_mode === "arc" ? params_arc : []),
    ...(jaiHDRStore.hdr_config.drive_mode !== "arc" ? params_drive : []),
  ];

  const renderSliders = () =>
    params.map((param) => (
      <VStack key={param.name} width="100%">
        <Slider
          min={param.range[0]}
          max={param.range[1]}
          size="md"
          width="80%"
          height="1rem"
          step={param.step}
          value={param.value}
          onChange={(value) => {
            jaiHDRStore.fetchUpdateConfig(param.name, value);
          }}
        >
          <SliderTrack>
            <SliderFilledTrack />
          </SliderTrack>
          <SliderThumb />
        </Slider>
        <HStack justifyContent="space-between" width="100%">
          {isMobile ? (
            <Body2>
              {param.name} : {param.value}
            </Body2>
          ) : (
            <>
              <H4>{param.name}</H4>
              <H4>{param.value}</H4>
            </>
          )}
        </HStack>
      </VStack>
    ));

  return (
    <>
      {isMobile ? (
        <Grid templateColumns="repeat(2, 1fr)" gap={4} width="100%">
          {renderSliders()}
          <JaiOptionSwitch
            option_id="lidar"
            option_name="Enable LiDAR acquire"
            checked={jaiHDRStore.hdr_config.lidar}
          />
          <JaiOptionSwitch
            option_id="drive_forward"
            option_name="Drive forward/Side"
            checked={jaiHDRStore.hdr_config.drive_forward}
          />
        </Grid>
      ) : (
        <VStack width="15rem" spacing={4}>
          <OptionEnum
            option_id="drive_mode"
            option_name="Drive mode"
            value={jaiHDRStore.hdr_config.drive_mode}
            value_list={["forward", "side", "arc"]}
          />
          {renderSliders()}
          <JaiOptionSwitch
            option_id="lidar"
            option_name="Enable LiDAR acquire"
            checked={jaiHDRStore.hdr_config.lidar}
          />
          <JaiOptionSwitch
            option_id="drive_forward"
            option_name="Drive forward/Side"
            checked={jaiHDRStore.hdr_config.drive_forward}
          />
        </VStack>
      )}
    </>
  );
};

const HDRProgressView = observer(() => {
  const {
    progress_root: pr,
    progress_sub: ps,
    hdr_error_msgs,
  } = jaiHDRStore.hdr_log || {
    progress_root: {
      idx: 0,
      status: "ready",
      task: "rotate",
    },
    progress_sub: {
      idx: 0,
      type: "hdr",
    },
    hdr_error_msgs: [],
  };

  const isMobile = window.innerWidth < 768;

  return (
    <Flex width="100%" justifyContent="space-between" wrap="wrap">
      {isMobile ? (
        <VStack>
          <JaiHDRControls isMobile={isMobile} />
          <HDRPreview />
        </VStack>
      ) : (
        <HStack width="100%" justifyContent="space-between">
          <JaiHDRControls isMobile={isMobile} />
          <HDRPreview />
        </HStack>
      )}

      {!["running", "pause", "abort"].includes(
        jaiHDRStore.hdr_log?.progress_root.status ?? ""
      ) && (
        <InfoCardBtn
          title="Capture"
          Icon={Camera}
          onClick={() => jaiHDRStore.triggerHDR()}
        />
      )}
      {jaiHDRStore.hdr_log?.progress_root.status === "running" && (
        <InfoCardBtn title={pr.task} Icon={Steps} color={Color.Cyan} />
      )}
      {jaiHDRStore.hdr_log?.progress_root.status === "running" && (
        <InfoCardBtn
          title={"Pause"}
          Icon={Pause}
          onClick={() => jaiHDRStore.triggerPause()}
          color={Color.Green}
        />
      )}
      {jaiHDRStore.hdr_log?.progress_root.status === "pause" && (
        <InfoCardBtn
          title={"Resume"}
          Icon={Play}
          onClick={() => jaiHDRStore.triggerResume()}
          color={Color.Yellow}
        />
      )}
      {jaiHDRStore.hdr_log?.progress_root.status === "running" && (
        <InfoCardBtn
          title={"Abort"}
          Icon={Stop}
          onClick={() => jaiHDRStore.triggerStop()}
          color={Color.Red}
        />
      )}
      <InfoCardBtn
        title={jaiHDRStore.hdr_latest_capture.frame_count.toString()}
        Icon={GooglePhotosLogo}
        color={Color.Indigo}
      />

      <InfoCard
        title={pr.status}
        value={pr.idx}
        progressBar={
          <CircularProgress
            value={
              (pr.idx /
                (jaiHDRStore.hdr_config.capture_cnt *
                  jaiHDRStore.hdr_config.side_move_cnt)) *
              100
            }
          />
        }
      />

      {(pr.task === "hdr" || pr.task === "ambient") && (
        <InfoCard
          title={"Exposure"}
          value={pr.task}
          progressBar={<CircularProgress value={(ps.idx / 4) * 100} />}
        />
      )}
      {hdr_error_msgs.length < 1 && <InfoCard title="Error" value="None" />}
      {hdr_error_msgs.length > 0 &&
        hdr_error_msgs.map((error) => <ErrorCard error={error} />)}
    </Flex>
  );
});

const HDRPreview = observer(() => {
  const { image } = jaiHDRStore.hdr_latest_capture;

  return (
    <VStack width="100%" height="auto">
      <Box
        position="relative"
        width="50%"
        // 원하는 고정 높이가 있으면 지정하거나, aspectRatio 사용 가능
        // height="auto"
      >
        {image ? (
          <Image
            src={`data:image/bmp;base64,${image}`}
            alt="HDR capture"
            objectFit="contain"
            transform="rotate(270deg)" /* CSS transform 사용 */
            width="100%"
          />
        ) : (
          <Box
            width="100%"
            height="200px" /* placeholder 높이 지정 */
            bg="gray.100"
            display="flex"
            alignItems="center"
            justifyContent="center"
          >
            <Text color="gray.500">No Image Available</Text>
          </Box>
        )}

        <IconButton
          icon={<CameraRotate />}
          aria-label="Refresh Capture"
          onClick={() => {
            jaiHDRStore.fetchGetLatestCapture();
          }}
          position="absolute"
          top="2"
          left="2"
          size="sm"
          zIndex="overlay" /* overlay 레이어 보장 */
        />
      </Box>
    </VStack>
  );
});

const HDRStorageBrowser = observer(() => {
  const isMobile = window.innerWidth < 768;

  return (
    <HStack width="100%" spacing={4}>
      <VStack>
        <H4>HDR Storage Browser</H4>

        {/* Scene Selector */}
        <HStack width="100%">
          <Text>Scene:</Text>
          <Select
            value={jaiHDRStore.current_scene_id}
            onChange={(e) => jaiHDRStore.fetchSceneFrames(e.target.value)}
            width={isMobile ? "100%" : "300px"}
          >
            <option value="">Select Scene...</option>
            {jaiHDRStore.hdr_scene_list.map((sceneId) => (
              <option key={sceneId} value={sceneId}>
                {sceneId}
              </option>
            ))}
          </Select>
        </HStack>
      </VStack>

      {/* Frame Browser */}
      {jaiHDRStore.current_scene_id && (
        <>
          <Divider />
          <VStack>
            <HStack width="100%">
              <Text>Frames ({jaiHDRStore.current_scene_frames.length}):</Text>
              <IconButton
                icon={<CameraRotate />}
                aria-label="Refresh Frames"
                onClick={() =>
                  jaiHDRStore.fetchSceneFrames(jaiHDRStore.current_scene_id)
                }
                size="sm"
              />
            </HStack>

            {/* Frame List - Scrollable */}
            <Box
              width="16rem"
              maxHeight="200px"
              overflowY="auto"
              border="1px solid"
              borderColor="gray.200"
              borderRadius="md"
              p={2}
            >
              <VStack spacing={1} align="stretch">
                {jaiHDRStore.current_scene_frames.map((frameId) => (
                  <HStack
                    key={frameId}
                    p={2}
                    bg={
                      jaiHDRStore.selected_frame_id === frameId
                        ? "blue.100"
                        : "gray.50"
                    }
                    borderRadius="md"
                    _hover={{ bg: "blue.50" }}
                    justifyContent="space-between"
                  >
                    <Box
                      flex={1}
                      cursor="pointer"
                      onClick={() => jaiHDRStore.selectFrame(frameId)}
                    >
                      <Text fontSize="sm">{frameId}</Text>
                    </Box>
                    <IconButton
                      icon={<Trash />}
                      aria-label="Delete Frame"
                      size="xs"
                      colorScheme="red"
                      variant="ghost"
                      onClick={(e) => {
                        e.stopPropagation();
                        if (window.confirm(`Delete frame ${frameId}?`)) {
                          jaiHDRStore.deleteFrame(
                            jaiHDRStore.current_scene_id,
                            frameId
                          );
                        }
                      }}
                    />
                  </HStack>
                ))}
              </VStack>
            </Box>
          </VStack>

          {/* Frame Preview */}
          {jaiHDRStore.selected_frame_id && (
            <>
              <Divider />
              <VStack width="16rem">
                <Text>Frame: {jaiHDRStore.selected_frame_id}</Text>
                <Box
                  width={isMobile ? "100%" : "400px"}
                  height="300px"
                  position="relative"
                >
                  {jaiHDRStore.frame_preview_image ? (
                    <Image
                      src={jaiHDRStore.frame_preview_image}
                      alt={`Frame ${jaiHDRStore.selected_frame_id}`}
                      objectFit="contain"
                      width="100%"
                      height="100%"
                    />
                  ) : (
                    <Box
                      width="100%"
                      height="100%"
                      bg="gray.100"
                      display="flex"
                      alignItems="center"
                      justifyContent="center"
                    >
                      <Text color="gray.500">Loading...</Text>
                    </Box>
                  )}
                </Box>
              </VStack>
            </>
          )}
        </>
      )}
    </HStack>
  );
});

export const HDRPage = observer(() => {
  useEffect(() => {
    //jaiStore.fetchGetStereoNodeStatus();
  }, []);

  return (
    <PageRoot title="HDR">
      <VStack spacing={6} width="100%">
        <HDRProgressView />
        <Divider />
        <HDRStorageBrowser />
      </VStack>
    </PageRoot>
  );
});
