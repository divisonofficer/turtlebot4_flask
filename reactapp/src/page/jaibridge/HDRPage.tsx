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
  useMediaQuery,
  Spinner,
} from "@chakra-ui/react";
import { PageRoot } from "../../design/other/flexs";

import { Body2, H4, H3 } from "../../design/text/textsystem";
import { observer } from "mobx-react";

import { useEffect, useState } from "react";
import { InfoCard, InfoCardBtn } from "../../design/other/infocard";
import { jaiHDRStore } from "../../stores/JaiHDRStore";
import { Camera } from "@phosphor-icons/react/dist/ssr";
import {
  ArrowsClockwise,
  CameraRotate,
  GooglePhotosLogo,
  Pause,
  Play,
  Steps,
  Stop,
  Trash,
  Warning,
} from "@phosphor-icons/react";
import { Color } from "../../design/color";

// Card component for consistent styling
const Card = ({ children }: { children: React.ReactNode }) => (
  <Box
    bg="white"
    borderRadius="1rem"
    p={6}
    border="1px solid"
    borderColor="gray.100"
    shadow="sm"
  >
    {children}
  </Box>
);

const JaiOptionSwitch = ({
  option_id,
  option_name,
  checked,
}: {
  option_id: string;
  option_name: string;
  checked: boolean | undefined;
}) => {
  const [isMobile] = useMediaQuery("(max-width: 768px)");
  return (
    <HStack justify="space-between" width="100%">
      {isMobile ? <Body2>{option_name}</Body2> : <Body2>{option_name}</Body2>}
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
    <VStack align="stretch" spacing={2} width="100%">
      <Body2>{option_name}</Body2>
      <Select
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
      </Select>
    </VStack>
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
    {
      range: [1, 20],
      step: 1,
      name: "capture_cnt",
      value: jaiHDRStore.hdr_config.capture_cnt,
    },
  ];
  const params = [
    ...(jaiHDRStore.hdr_config.drive_mode === "arc" ? params_arc : []),
    ...(jaiHDRStore.hdr_config.drive_mode !== "arc" ? params_drive : []),
  ];

  return (
    <VStack spacing={6} align="stretch" width="100%">
      {/* Drive Mode Section */}
      <Box>
        <H4 mb={3}>Drive mode</H4>
        <OptionEnum
          option_id="drive_mode"
          option_name=""
          value={jaiHDRStore.hdr_config.drive_mode}
          value_list={["forward", "side", "arc"]}
        />
      </Box>

      {/* Parameters Section */}
      <Box>
        <H4 mb={3}>Parameters</H4>
        <VStack spacing={4} align="stretch">
          {params.map((param) => (
            <Box key={param.name}>
              <HStack justify="space-between" mb={2}>
                <Body2>{param.name}</Body2>
                <Body2 fontWeight="medium">{param.value}</Body2>
              </HStack>
              <Slider
                min={param.range[0]}
                max={param.range[1]}
                size="md"
                width="100%"
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
            </Box>
          ))}
        </VStack>
      </Box>

      {/* Options Section */}
      <Box>
        <H4 mb={3}>Options</H4>
        <VStack spacing={3} align="stretch">
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
          <JaiOptionSwitch
            option_id="use_piper"
            option_name="Use Piper Arm"
            checked={jaiHDRStore.hdr_config.use_piper}
          />
          <JaiOptionSwitch
            option_id="arc_forward"
            option_name="Arc Forward"
            checked={jaiHDRStore.hdr_config.arc_forward}
          />
          {!isMobile && (
            <JaiOptionSwitch
              option_id="skip_jai"
              option_name="Skip Camera"
              checked={jaiHDRStore.hdr_config.skip_jai}
            />
          )}
        </VStack>
      </Box>
    </VStack>
  );
};

const HDRProgressView = observer(() => {
  const {
    progress_root: pr,
    progress_sub: ps,
    hdr_error_msgs,
    capture_timing,
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
    capture_timing: {
      duration_ms: 0,
      duration_sec: 0,
      mode: "",
    },
  };

  const [isMobile] = useMediaQuery("(max-width: 768px)");
  const [useNewSpace, setUseNewSpace] = useState(true);

  return (
    <Grid templateColumns={{ base: "1fr", xl: "1fr 1fr" }} gap={6} width="100%">
      {/* Left Column - Controls & Buttons */}
      <VStack spacing={6} align="stretch">
        <Card>
          <JaiHDRControls isMobile={isMobile} />
        </Card>

        {/* Space Selector */}
        <Card>
          <VStack align="stretch" spacing={3}>
            <H4>Capture Space</H4>
            <VStack align="stretch" spacing={2}>
              <HStack>
                <input
                  type="radio"
                  checked={useNewSpace}
                  onChange={() => setUseNewSpace(true)}
                  style={{ cursor: "pointer" }}
                />
                <Body2>Create New Space</Body2>
              </HStack>

              <HStack>
                <input
                  type="radio"
                  checked={!useNewSpace}
                  onChange={() => setUseNewSpace(false)}
                  disabled={!jaiHDRStore.current_hdr_space_id}
                  style={{
                    cursor: jaiHDRStore.current_hdr_space_id
                      ? "pointer"
                      : "not-allowed",
                  }}
                />
                <Body2
                  color={
                    !jaiHDRStore.current_hdr_space_id ? "gray.400" : "inherit"
                  }
                >
                  Continue from: {jaiHDRStore.current_hdr_space_id || "(None)"}
                </Body2>
              </HStack>
            </VStack>
          </VStack>
        </Card>

        {/* Action Buttons */}
        <Grid templateColumns="repeat(2, 1fr)" gap={3}>
          {!["running", "pause", "abort"].includes(
            jaiHDRStore.hdr_log?.progress_root.status ?? ""
          ) && (
            <InfoCardBtn
              title="Capture"
              Icon={Camera}
              onClick={() => jaiHDRStore.triggerHDR(useNewSpace)}
              color="#1C1C1C"
            />
          )}

          {jaiHDRStore.hdr_log?.progress_root.status === "running" && (
            <>
              <InfoCardBtn title={pr.task} Icon={Steps} color={Color.Cyan} />
              <InfoCardBtn
                title={"Pause"}
                Icon={Pause}
                onClick={() => jaiHDRStore.triggerPause()}
                color={Color.Green}
              />
              <InfoCardBtn
                title={"Abort"}
                Icon={Stop}
                onClick={() => jaiHDRStore.triggerStop()}
                color={Color.Red}
              />
              <InfoCardBtn
                title={"Force Stop"}
                Icon={Warning}
                onClick={() => {
                  if (window.confirm("강제 종료하시겠습니까? 현재 작업이 중단됩니다.")) {
                    jaiHDRStore.triggerForceStop();
                  }
                }}
                color={Color.Red}
              />
            </>
          )}

          {jaiHDRStore.hdr_log?.progress_root.status === "pause" && (
            <>
              <InfoCardBtn title={pr.task} Icon={Steps} color={Color.Cyan} />
              <InfoCardBtn
                title={"Resume"}
                Icon={Play}
                onClick={() => jaiHDRStore.triggerResume()}
                color={Color.Yellow}
              />
              <InfoCardBtn
                title={"Abort"}
                Icon={Stop}
                onClick={() => jaiHDRStore.triggerStop()}
                color={Color.Red}
              />
            </>
          )}

          <InfoCardBtn
            title={jaiHDRStore.hdr_latest_capture.frame_count.toString()}
            Icon={GooglePhotosLogo}
            color={Color.Indigo}
          />
        </Grid>
      </VStack>

      {/* Right Column - Preview, Status & Logs */}
      <VStack spacing={6} align="stretch">
        {/* Preview */}
        <Card>
          <HDRPreview />
        </Card>

        {/* Status Cards */}
        <Grid templateColumns="repeat(4, 1fr)" gap={4}>
          <InfoCardBtn
            title="Refresh"
            Icon={ArrowsClockwise}
            onClick={() => jaiHDRStore.refreshStatus()}
            color={Color.Blue}
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

          {pr.task === "hdr" || pr.task === "ambient" ? (
            <InfoCard
              title={"Exposure"}
              value={pr.task}
              progressBar={<CircularProgress value={(ps.idx / 6) * 100} />}
            />
          ) : (
            <InfoCard title="Status" value={pr.idx} />
          )}

          {
            <InfoCard
              title={`Capture (${capture_timing.mode})`}
              value={`${capture_timing.duration_sec.toFixed(2)}s`}
            />
          }
        </Grid>

        {/* Compact Error/Warning Messages */}
        {hdr_error_msgs.length > 0 && (
          <VStack align="stretch" spacing={1} width="100%">
            {hdr_error_msgs.map((error, idx) => {
              let message = "";
              let severity: "error" | "warning" | "info" = "info";

              if (error.type === "error_process_stream_expire") {
                message = `Stream expired: ${error.data.device}/${error.data.stream} (${error.data.expired_time}ms)`;
                severity = "warning";
              } else if (error.type === "error_retrieve_buffer") {
                message = `Buffer retrieve failed: ${
                  error.data.lResult === "OK"
                    ? error.data.aResult
                    : error.data.lResult
                }`;
                severity = "warning";
              } else if (error.type === "error_crash") {
                message = `Crash: ${error.data.cause}`;
                severity = "error";
              } else {
                message = `${error.type}: ${JSON.stringify(error.data)}`;
                severity = "info";
              }

              const bgColor =
                severity === "error"
                  ? "red.50"
                  : severity === "warning"
                  ? "yellow.50"
                  : "blue.50";
              const borderColor =
                severity === "error"
                  ? "red.400"
                  : severity === "warning"
                  ? "yellow.400"
                  : "blue.400";
              const textColor =
                severity === "error"
                  ? "red.700"
                  : severity === "warning"
                  ? "yellow.700"
                  : "blue.700";

              return (
                <Box
                  key={idx}
                  bg={bgColor}
                  borderLeft="3px solid"
                  borderLeftColor={borderColor}
                  borderRadius="md"
                  px={3}
                  py={2}
                >
                  <HStack spacing={2}>
                    <Text
                      fontSize="xs"
                      fontWeight="bold"
                      color={textColor}
                      textTransform="uppercase"
                    >
                      {severity}
                    </Text>
                    <Text fontSize="xs" color={textColor} flex={1}>
                      {message}
                    </Text>
                  </HStack>
                </Box>
              );
            })}
          </VStack>
        )}
      </VStack>
    </Grid>
  );
});

const HDRPreview = observer(() => {
  const { image } = jaiHDRStore.hdr_latest_capture;

  return (
    <Box position="relative" width="100%">
      {image ? (
        <Image
          src={
            image.startsWith("data:") ? image : `data:image/bmp;base64,${image}`
          }
          alt="HDR capture"
          objectFit="contain"
          width="100%"
          maxH="400px"
          borderRadius="md"
          onError={(e) => {
            console.error("Image failed to load", e);
          }}
        />
      ) : (
        <Box
          width="100%"
          height="300px"
          bg="gray.50"
          borderRadius="md"
          display="flex"
          alignItems="center"
          justifyContent="center"
        >
          <VStack spacing={2}>
            <Camera size={48} color="gray" />
            <Body2 color="gray.500">No Image Available</Body2>
          </VStack>
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
        right="2"
        size="sm"
        colorScheme="blackAlpha"
        zIndex="overlay"
      />
    </Box>
  );
});

const HDRStorageBrowser = observer(() => {
  const [scenesExpanded, setScenesExpanded] = useState(true);
  const [scrollContainerRef, setScrollContainerRef] =
    useState<HTMLDivElement | null>(null);
  const [visibleFrameIndices, setVisibleFrameIndices] = useState<Set<number>>(
    new Set()
  );

  // Lazy loading effect - load thumbnails for visible frames
  useEffect(() => {
    if (!scrollContainerRef) return;

    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            const index = parseInt(
              entry.target.getAttribute("data-index") || "0"
            );
            setVisibleFrameIndices((prev) => new Set(prev).add(index));
          }
        });
      },
      {
        root: scrollContainerRef,
        rootMargin: "200px", // Preload items 200px before they enter viewport
        threshold: 0.01,
      }
    );

    const items = scrollContainerRef.querySelectorAll("[data-index]");
    items.forEach((item) => observer.observe(item));

    return () => observer.disconnect();
  }, [scrollContainerRef]);

  // Load thumbnails for visible frames
  useEffect(() => {
    visibleFrameIndices.forEach((index) => {
      const frameId = jaiHDRStore.current_scene_frames[index];
      if (frameId && !jaiHDRStore.frame_thumbnails.has(frameId)) {
        jaiHDRStore.fetchFramePreview(jaiHDRStore.current_scene_id, frameId);
      }
    });
  }, [visibleFrameIndices]);

  return (
    <Card>
      <VStack align="stretch" spacing={4} width="100%">
        <HStack justify="space-between">
          <H3>HDR Storage Browser</H3>
          {jaiHDRStore.current_scene_id && (
            <IconButton
              icon={<CameraRotate />}
              aria-label="Refresh Scene"
              onClick={() =>
                jaiHDRStore.fetchSceneFrames(jaiHDRStore.current_scene_id)
              }
              size="sm"
              variant="ghost"
            />
          )}
        </HStack>

        {/* Scene Selector - Collapsible */}
        <VStack align="stretch" spacing={2}>
          <HStack
            justify="space-between"
            cursor="pointer"
            onClick={() => setScenesExpanded(!scenesExpanded)}
            p={2}
            borderRadius="md"
            _hover={{ bg: "gray.50" }}
          >
            <H4>Available Scenes ({jaiHDRStore.hdr_scene_list.length})</H4>
            <Text fontSize="xl">{scenesExpanded ? "▼" : "▶"}</Text>
          </HStack>

          {scenesExpanded && (
            <>
              {jaiHDRStore.hdr_scene_list.length > 0 ? (
                <HStack
                  overflowX="auto"
                  spacing={3}
                  pb={2}
                  css={{
                    "&::-webkit-scrollbar": {
                      height: "8px",
                    },
                    "&::-webkit-scrollbar-track": {
                      background: "#f1f1f1",
                      borderRadius: "4px",
                    },
                    "&::-webkit-scrollbar-thumb": {
                      background: "#888",
                      borderRadius: "4px",
                    },
                    "&::-webkit-scrollbar-thumb:hover": {
                      background: "#555",
                    },
                  }}
                >
                  {jaiHDRStore.hdr_scene_list.map((sceneId) => {
                    const thumbnail =
                      jaiHDRStore.scene_representative_thumbnails.get(sceneId);
                    const isSelected = jaiHDRStore.current_scene_id === sceneId;

                    return (
                      <Box
                        key={sceneId}
                        cursor="pointer"
                        onClick={() => jaiHDRStore.fetchSceneFrames(sceneId)}
                        borderRadius="md"
                        overflow="hidden"
                        border="2px solid"
                        borderColor={isSelected ? "blue.500" : "gray.200"}
                        _hover={{
                          borderColor: "blue.300",
                          shadow: "md",
                        }}
                        transition="all 0.2s"
                        minW="150px"
                        flexShrink={0}
                      >
                        {/* Thumbnail */}
                        <Box
                          width="150px"
                          height="100px"
                          position="relative"
                          bg="gray.100"
                        >
                          {thumbnail ? (
                            <Image
                              src={thumbnail}
                              alt={sceneId}
                              width="100%"
                              height="100%"
                              objectFit="cover"
                            />
                          ) : (
                            <Flex
                              width="100%"
                              height="100%"
                              alignItems="center"
                              justifyContent="center"
                            >
                              <Camera size={32} color="gray" opacity={0.3} />
                            </Flex>
                          )}
                        </Box>

                        {/* Scene Info */}
                        <Box p={2} bg={isSelected ? "blue.50" : "white"}>
                          <Text
                            fontSize="xs"
                            fontWeight={isSelected ? "bold" : "medium"}
                            noOfLines={1}
                          >
                            {sceneId}
                          </Text>
                        </Box>
                      </Box>
                    );
                  })}
                </HStack>
              ) : (
                <Box p={4} textAlign="center" color="gray.500">
                  <Text>No scenes available</Text>
                </Box>
              )}
            </>
          )}
        </VStack>

        {/* Image Gallery - 3 Rows Horizontal Scroll with Lazy Loading */}
        {jaiHDRStore.current_scene_id &&
        jaiHDRStore.current_scene_frames.length > 0 ? (
          <Box>
            <Body2 mb={3} color="gray.600">
              {jaiHDRStore.current_scene_frames.length} images
            </Body2>
            <Box
              ref={setScrollContainerRef}
              overflowX="auto"
              overflowY="hidden"
              pb={2}
              css={{
                "&::-webkit-scrollbar": {
                  height: "10px",
                },
                "&::-webkit-scrollbar-track": {
                  background: "#f1f1f1",
                  borderRadius: "4px",
                },
                "&::-webkit-scrollbar-thumb": {
                  background: "#888",
                  borderRadius: "4px",
                },
                "&::-webkit-scrollbar-thumb:hover": {
                  background: "#555",
                },
              }}
            >
              <Grid
                templateRows="repeat(3, 1fr)"
                autoFlow="column"
                gap={3}
                minH="450px"
              >
                {jaiHDRStore.current_scene_frames.map((frameId, index) => {
                  // Get cached thumbnail
                  const thumbnail = jaiHDRStore.frame_thumbnails.get(frameId);
                  const isSelected = jaiHDRStore.selected_frame_id === frameId;
                  const isVisible = visibleFrameIndices.has(index);

                  return (
                    <Box
                      key={frameId}
                      data-index={index}
                      position="relative"
                      cursor="pointer"
                      onClick={() => jaiHDRStore.selectFrame(frameId)}
                      borderRadius="md"
                      overflow="hidden"
                      border="2px solid"
                      borderColor={isSelected ? "blue.500" : "gray.200"}
                      _hover={{
                        borderColor: "blue.300",
                        shadow: "md",
                      }}
                      transition="all 0.2s"
                      minW="200px"
                      width="200px"
                    >
                      {/* Image Container */}
                      <Box
                        width="100%"
                        height="120px"
                        position="relative"
                        bg="gray.100"
                      >
                        {isVisible && thumbnail ? (
                          <Image
                            src={thumbnail}
                            alt={frameId}
                            width="100%"
                            height="100%"
                            objectFit="cover"
                          />
                        ) : (
                          <Flex
                            width="100%"
                            height="100%"
                            alignItems="center"
                            justifyContent="center"
                          >
                            <Spinner size="sm" color="blue.500" />
                          </Flex>
                        )}
                      </Box>

                      {/* Image Info */}
                      <Box p={2} bg={isSelected ? "blue.50" : "white"}>
                        <Text
                          fontSize="xs"
                          fontWeight={isSelected ? "bold" : "medium"}
                          noOfLines={1}
                        >
                          {frameId}
                        </Text>
                        <HStack justify="space-between" mt={1}>
                          <Text fontSize="xs" color="gray.500">
                            #{index + 1}
                          </Text>
                          <IconButton
                            icon={<Trash />}
                            aria-label="Delete Frame"
                            size="xs"
                            variant="ghost"
                            colorScheme="red"
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
                      </Box>
                    </Box>
                  );
                })}
              </Grid>
            </Box>
          </Box>
        ) : jaiHDRStore.current_scene_id ? (
          <Box
            p={8}
            textAlign="center"
            border="1px dashed"
            borderColor="gray.300"
            borderRadius="md"
          >
            <Camera
              size={48}
              color="gray"
              opacity={0.3}
              style={{ margin: "0 auto" }}
            />
            <Text mt={3} color="gray.500">
              No images in this scene
            </Text>
          </Box>
        ) : (
          <Box
            p={8}
            textAlign="center"
            border="1px dashed"
            borderColor="gray.300"
            borderRadius="md"
          >
            <GooglePhotosLogo
              size={48}
              color="gray"
              opacity={0.3}
              style={{ margin: "0 auto" }}
            />
            <Text mt={3} color="gray.500">
              Select a scene to view images
            </Text>
          </Box>
        )}
      </VStack>
    </Card>
  );
});

export const HDRPage = observer(() => {
  useEffect(() => {
    jaiHDRStore.fetchHDRSceneList();
    jaiHDRStore.fetchGetLatestCapture();
  }, []);

  return (
    <PageRoot title="HDR">
      <VStack spacing={8} width="100%">
        <HDRProgressView />
        <HDRStorageBrowser />
      </VStack>
    </PageRoot>
  );
});
