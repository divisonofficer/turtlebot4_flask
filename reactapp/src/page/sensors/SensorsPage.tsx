import { observer } from "mobx-react";
import { PageRoot } from "../../design/other/flexs";
import { sensorsStore, Sensor, SensorGroup } from "../../stores/SensorsStore";
import {
  Alert,
  AlertIcon,
  Card,
  Code,
  Flex,
  HStack,
  IconButton,
  Input,
  Switch,
  VStack,
  useToast,
  Box,
  Text,
  Badge,
  Progress,
  Divider,
  CardBody,
  Stat,
  StatLabel,
  StatNumber,
  StatHelpText,
  Grid,
  GridItem,
  Collapse,
  Icon,
  SimpleGrid,
} from "@chakra-ui/react";
import { CameraPlus, Pause, Play, Record } from "@phosphor-icons/react";
import { CheckCircleIcon, InfoIcon, WarningIcon } from "@chakra-ui/icons";
import { Body3, H4 } from "../../design/text/textsystem";
import { useEffect, useRef, useState } from "react";
import { lucidSocket } from "../../connect/socket/subscribe";
import { Btn } from "../../design/button/button";
import { lucidStore } from "../../stores/LucidStore";
import { Color } from "../../design/color";

const SensorStatusIndicator = observer((props: { sensor: Sensor.Sensor }) => {
  const status = props.sensor.state.device_status;
  const color =
    status === "connected"
      ? "green"
      : status === "connecting"
      ? "yellow"
      : status === "error"
      ? "red"
      : "gray";

  const handleClick = () => {
    if (status === "disconnected" || status === "error") {
      sensorsStore.fetchLaunchDevice(props.sensor);
    }
  };

  return (
    <div
      onClick={handleClick}
      style={{
        minWidth: "1rem",
        height: "1rem",
        borderRadius: "50%",
        backgroundColor: color,
        cursor: "pointer",
      }}
    />
  );
});

const SensorCameraControl = observer((props: { sensor: Sensor.Camera }) => {
  return (
    <HStack>
      {props.sensor.state.stream_on || (
        <IconButton
          icon={<Play />}
          aria-label=""
          onClick={() => sensorsStore.stream_start(props.sensor)}
          isDisabled={props.sensor.state.device_status !== "connected"}
        />
      )}
      {props.sensor.state.stream_on && (
        <IconButton
          icon={<Pause />}
          aria-label=""
          onClick={() => sensorsStore.stream_stop(props.sensor)}
        />
      )}
    </HStack>
  );
});

const SensorLidarControl = observer((props: { sensor: Sensor.Sensor }) => {
  const hasStreamOn = "stream_on" in props.sensor.state;
  const streamOn = hasStreamOn ? (props.sensor.state as any).stream_on : false;

  return (
    <HStack>
      {!streamOn && (
        <IconButton
          icon={<Play />}
          aria-label="Start Lidar Stream"
          onClick={() => sensorsStore.stream_start(props.sensor)}
          isDisabled={props.sensor.state.device_status !== "connected"}
        />
      )}
      {streamOn && (
        <IconButton
          icon={<Pause />}
          aria-label="Stop Lidar Stream"
          onClick={() => sensorsStore.stream_stop(props.sensor)}
        />
      )}
    </HStack>
  );
});

const SensorPreview = observer((props: { sensor: Sensor.Sensor }) => {
  const [preview, setPreview] = useState<(string | undefined)[]>(
    new Array(props.sensor.const.preview_keys.length).fill(undefined)
  );

  const canvasRefs = useRef<(HTMLCanvasElement | null)[]>([]);

  useEffect(() => {
    const subscriptions = props.sensor.const.preview_keys.map((key, idx) => {
      lucidSocket.subscribe(
        `/thumb/${props.sensor.name}/${key}`,
        (encoded: string) => {
          const decodedImage = `data:image/jpeg;base64,${encoded}`;
          setPreview((prev) => {
            const newPrev = [...prev];
            newPrev[idx] = decodedImage;
            return newPrev;
          });
        }
      );
      return key;
    });

    return () => {
      subscriptions.forEach((sub) => lucidSocket.unsubscribe(sub));
    };
  }, [props.sensor]);

  useEffect(() => {
    preview.forEach((src, idx) => {
      const canvas = canvasRefs.current[idx];
      if (canvas && src) {
        const ctx = canvas.getContext("2d");
        if (ctx) {
          const image = new Image();
          image.src = src;
          image.onload = () => {
            canvas.width = image.width;
            canvas.height = image.height;
            ctx.drawImage(image, 0, 0);
          };
        }
      }
    });
  }, [preview]);

  return (
    <Card style={{ width: "16rem", margin: "1rem", position: "relative" }}>
      {props.sensor.const.preview_keys.map((_, idx) => (
        <canvas
          key={idx}
          ref={(el) => (canvasRefs.current[idx] = el)}
          style={{
            width: "100%",
            height: "100%",
            display: "block",
            marginBottom: "0.5rem",
          }}
        />
      ))}
      <IconButton
        icon={props.sensor.state.preview_on ? <Pause /> : <Play />}
        aria-label="Pause"
        style={{ position: "absolute", bottom: "0.5rem", right: "0.5rem" }}
        onClick={() => {
          props.sensor.state.preview_on
            ? sensorsStore.fetchPostPreviewOff(props.sensor)
            : sensorsStore.fetchPostPreviewOn(props.sensor);
        }}
      />
    </Card>
  );
});

const SensorStateView = observer(({ state }: { state: Sensor.State }) => {
  const formatFps = (fps: number | undefined) => {
    if (!fps || fps === 0) return "0";
    if (fps < 1) return fps.toFixed(3);
    if (fps < 10) return fps.toFixed(2);
    return Math.round(fps).toString();
  };

  return (
    <VStack>
      <HStack>
        <Card
          style={{
            width: "8rem",
            backgroundColor: "#f0f0f0",
            display: "flex",
            flexDirection: "column",
            justifyContent: "space-between",
            height: "4rem",
            padding: "0.5rem",
          }}
        >
          <H4 style={{ fontSize: "0.75rem", textAlign: "left" }}>Last</H4>
          {"timestamp_last" in state && state.timestamp_last && (
            <H4 style={{ textAlign: "right" }}>
              {new Date(state.timestamp_last * 1000)
                .toISOString()
                .substr(14, 9)}
            </H4>
          )}
        </Card>
        <Card
          style={{
            width: "8rem",
            backgroundColor: "#e0e0e0",
            display: "flex",
            flexDirection: "column",
            justifyContent: "space-between",
            height: "4rem",
            padding: "0.5rem",
          }}
        >
          <H4 style={{ fontSize: "0.75rem", textAlign: "left" }}>FPS</H4>
          <H4 style={{ textAlign: "right" }}>{formatFps(state.fps)}</H4>
        </Card>
      </HStack>
    </VStack>
  );
});

const SensorConfigView = observer((props: { sensor: Sensor.Sensor }) => {
  const params = props.sensor.config || {};
  const [pendingValues, setPendingValues] = useState<Record<string, any>>({});

  // Initialize pendingValues with current values
  useEffect(() => {
    const initialValues: Record<string, any> = {};
    Object.entries(params).forEach(([name, param]) => {
      initialValues[name] = param.value;
    });
    setPendingValues(initialValues);
  }, [params]);

  const updateConfig = (name: string, value: any) => {
    // Dummy function for now - will be implemented later
    // console.log(`Updating ${name} to ${value}`);
    // // Update the pending value
    // setPendingValues((prev) => ({ ...prev, [name]: value }));
    sensorsStore.updateSensorConfig(props.sensor, name, value);
  };

  const ParamInt = (props: { name: string; param: Sensor.Param.IntParam }) => {
    const { name, param } = props;
    const [localValue, setLocalValue] = useState<number>(param.value);

    return (
      <Input
        fontSize="sm"
        p={0.5}
        height="1.6rem"
        type="number"
        min={param.min}
        max={param.max}
        width="8rem"
        value={localValue}
        onChange={(e) => setLocalValue(Number(e.target.value))}
        onKeyDown={(e) => {
          if (e.key === "Enter") {
            updateConfig(name, localValue);
          }
        }}
        onBlur={() => updateConfig(name, localValue)}
      />
    );
  };

  const ParamFloat = (props: {
    name: string;
    param: Sensor.Param.FloatParam;
  }) => {
    const { name, param } = props;
    const [localValue, setLocalValue] = useState<number>(param.value);

    return (
      <Input
        fontSize="sm"
        p={0.5}
        height="1.6rem"
        type="number"
        min={param.min}
        max={param.max}
        width="8rem"
        value={localValue}
        onChange={(e) => setLocalValue(Number(e.target.value))}
        onKeyDown={(e) => {
          if (e.key === "Enter") {
            updateConfig(name, localValue);
          }
        }}
        onBlur={() => updateConfig(name, localValue)}
      />
    );
  };

  const ParamBool = (props: {
    name: string;
    param: Sensor.Param.BoolParam;
  }) => {
    const { name, param } = props;
    return (
      <Switch
        size="sm"
        isChecked={param.value}
        onChange={(e) => updateConfig(name, e.target.checked)}
      />
    );
  };

  const ParamEnum = (props: {
    name: string;
    param: Sensor.Param.EnumParam;
  }) => {
    const { name, param } = props;
    return (
      <select
        style={{
          width: "4rem",
          fontSize: "0.75rem",
        }}
        value={param.value}
        onChange={(e) => updateConfig(name, e.target.value)}
      >
        {param.value_list.map((v, i) => {
          return (
            <option
              key={i}
              style={{
                fontSize: "0.75rem",
              }}
            >
              {v}
            </option>
          );
        })}
      </select>
    );
  };

  const ParamControl = (name: string, param: Sensor.Param.Param) => {
    if (param.type === "int") {
      return <ParamInt name={name} param={param as Sensor.Param.IntParam} />;
    }
    if (param.type === "float") {
      return (
        <ParamFloat name={name} param={param as Sensor.Param.FloatParam} />
      );
    }
    if (param.type === "bool") {
      return <ParamBool name={name} param={param as Sensor.Param.BoolParam} />;
    }
    if (param.type === "enum") {
      return <ParamEnum name={name} param={param as Sensor.Param.EnumParam} />;
    }
    return null;
  };

  return (
    <VStack
      style={{
        width: "100%",
      }}
    >
      {Object.entries(params)
        .filter(([name, param]) => param.hidden !== true)
        .map(([name, param], i) => {
          return (
            <HStack
              key={i}
              width="100%"
              justifyContent="space-between"
              height="2rem"
            >
              <Body3>{name}</Body3>
              {ParamControl(name, param)}
            </HStack>
          );
        })}
    </VStack>
  );
});

const SensorView = observer((props: { sensor: Sensor.Sensor }) => {
  return (
    <Card
      style={{
        width: "16rem",
        margin: "1rem",
      }}
    >
      <VStack>
        <HStack
          justifyContent="space-between"
          width="100%"
          px="0.5rem"
          py="0.25rem"
        >
          <H4
            style={{
              textAlign: "left",
              width: "100%",
              overflow: "hidden",
            }}
          >
            {props.sensor.name}
          </H4>
          <SensorStatusIndicator sensor={props.sensor} />
        </HStack>

        {props.sensor.type === "camera" && (
          <SensorCameraControl sensor={props.sensor as Sensor.Camera} />
        )}
        {props.sensor.type === "sensor" && (
          <SensorLidarControl sensor={props.sensor} />
        )}
        {props.sensor.const.preview_enabled && (
          <SensorPreview sensor={props.sensor} />
        )}
        {(props.sensor.type === "camera" || props.sensor.type === "sensor") && (
          <SensorStateView state={props.sensor.state} />
        )}
        <SensorConfigView sensor={props.sensor} />
      </VStack>
    </Card>
  );
});

const SensorGroupView = observer((props: { groups: SensorGroup[] }) => {
  useEffect(() => {
    sensorsStore.fetchGetGroupsList();
  }, []);
  return (
    <Flex>
      {props.groups.map((group, i) => {
        return (
          <Card
            key={i}
            style={{
              width: "16rem",
              margin: "1rem",
            }}
          >
            <VStack>
              <H4
                style={{
                  textAlign: "left",
                  width: "100%",
                  overflow: "hidden",
                }}
              >
                {group.name}
              </H4>
              {group.state.trigger_loop_on || (
                <IconButton
                  icon={<Play />}
                  aria-label=""
                  onClick={() => sensorsStore.fetchGroupTriggerLoopOn(group)}
                />
              )}
              {group.state.trigger_loop_on && (
                <IconButton
                  icon={<Pause />}
                  aria-label=""
                  onClick={() => sensorsStore.fetchGroupTrigger(group)}
                />
              )}
              <IconButton
                icon={<CameraPlus />}
                aria-label=""
                onClick={() => sensorsStore.fetchGroupTrigger(group)}
              />
            </VStack>
          </Card>
        );
      })}
    </Flex>
  );
});

const StorageControl = observer(() => {
  const status = lucidStore.lucidStatus;

  if (!status) return null;

  const {
    storage_enabled,
    single_storage_mode,
    storage_id,
    storage_queued_cnt,
  } = status;

  return (
    <Card p={4} m={4} variant="outline" maxW="24rem">
      <VStack spacing={4} align="stretch">
        <HStack justifyContent="space-between">
          <H4>Storage</H4>
          <Btn
            size="sm"
            icon={<CameraPlus />}
            onClick={() =>
              lucidStore.fetchUpdateStatusAttributes({
                single_storage_mode: !single_storage_mode,
              })
            }
            color={single_storage_mode ? Color.Mint : "#aaaaaa"}
            varient="filled"
          >
            Single Capture
          </Btn>
        </HStack>

        <HStack spacing={4}>
          {storage_enabled ? (
            <Btn
              icon={<Pause />}
              color={Color.Red}
              onClick={lucidStore.fetchDisableStorage}
              varient="filled"
            >
              Stop
            </Btn>
          ) : (
            <Btn
              icon={<Record />}
              color={Color.Green}
              onClick={lucidStore.fetchEnableStorage}
              varient="filled"
            >
              {single_storage_mode ? "Capture" : "Record"}
            </Btn>
          )}
        </HStack>

        {storage_id && (
          <VStack
            p={3}
            bg="gray.100"
            borderRadius="md"
            align="start"
            spacing={1}
          >
            <HStack justifyContent="space-between" w="100%">
              <Body3>Storage ID</Body3>
              <Code>{storage_id}</Code>
            </HStack>
            <HStack justifyContent="space-between" w="100%">
              <Body3>Frames</Body3>
              <Code>{storage_queued_cnt}</Code>
            </HStack>
          </VStack>
        )}
      </VStack>
    </Card>
  );
});

const HDRBurstControl = observer(() => {
  const {
    hdrBurstMode,
    hdrBurstInProgress,
    hdrBurstProgress,
    hdrBurstCurrentStep,
    hdrHighGainMode,
    hdrHighGainBurstCount,
    hdrHighGainProcessing,
    hdrHighGainProcessingProgress,
    storageEnabled,
  } = lucidStore;

  const toast = useToast();
  const [lastStep, setLastStep] = useState("");

  // Show toast notifications for HDR burst progress
  useEffect(() => {
    if (hdrBurstCurrentStep && hdrBurstCurrentStep !== lastStep) {
      if (hdrBurstCurrentStep === "completed") {
        toast({
          title: "HDR Burst Completed",
          description: "All exposures captured successfully",
          status: "success",
          duration: 3000,
          isClosable: true,
        });
      } else if (hdrBurstCurrentStep.includes("error")) {
        toast({
          title: "HDR Burst Error",
          description: hdrBurstCurrentStep,
          status: "error",
          duration: 5000,
          isClosable: true,
        });
      } else if (hdrBurstCurrentStep === "preparing") {
        toast({
          title: "HDR Burst Started",
          description: "Preparing cameras for multi-exposure capture",
          status: "info",
          duration: 2000,
          isClosable: true,
        });
      }
      setLastStep(hdrBurstCurrentStep);
    }
  }, [hdrBurstCurrentStep, lastStep, toast]);

  const getProgressPercentage = (cameraProgress: any) => {
    if (!cameraProgress) return 0;
    return (
      (cameraProgress.current_exposure / cameraProgress.total_exposures) * 100
    );
  };

  const getOverallProgress = () => {
    if (!hdrBurstCurrentStep) return 0;
    if (hdrBurstCurrentStep === "preparing") return 5;
    if (hdrBurstCurrentStep.includes("88us")) return 20;
    if (hdrBurstCurrentStep.includes("880us")) return 35;
    if (hdrBurstCurrentStep.includes("8800us")) return 50;
    if (hdrBurstCurrentStep.includes("88000us")) return 65;
    if (hdrBurstCurrentStep === "high_gain_burst") return 80;
    if (hdrBurstCurrentStep === "collecting_other_sensors") return 90;
    if (hdrBurstCurrentStep === "saving") return 95;
    if (hdrBurstCurrentStep === "completed") return 100;
    return 0;
  };

  const getStepDescription = (step: string) => {
    if (!step) return "Ready";
    if (step === "preparing") return "Preparing cameras...";
    if (step.includes("88us")) return "Capturing shortest exposure (88μs)";
    if (step.includes("880us")) return "Capturing short exposure (880μs)";
    if (step.includes("8800us")) return "Capturing medium exposure (8.8ms)";
    if (step.includes("88000us")) return "Capturing long exposure (88ms)";
    if (step === "high_gain_burst")
      return `High-gain burst (${hdrHighGainBurstCount}/8 frames)`;
    if (step === "collecting_other_sensors")
      return "Collecting LIDAR & RealSense data";
    if (step === "saving") return "Saving to storage...";
    if (step === "completed") return "HDR burst completed!";
    if (step.includes("error")) return `Error: ${step}`;
    return step;
  };

  // Count available sensors
  const lucidCameras = sensorsStore.sensors.filter(
    (s) =>
      s.name.toLowerCase().includes("lucid") &&
      s.state.device_status === "connected"
  );
  const lidarSensors = sensorsStore.sensors.filter(
    (s) =>
      s.name.toLowerCase().includes("ouster") &&
      s.state.device_status === "connected"
  );
  const realsenseCameras = sensorsStore.sensors.filter(
    (s) =>
      s.name.toLowerCase().includes("realsense") &&
      s.state.device_status === "connected"
  );

  return (
    <Card p={4} bg="white" borderRadius="md" shadow="sm" minW="24rem">
      <VStack align="start" spacing={3}>
        <HStack justifyContent="space-between" w="100%">
          <H4>HDR Burst Imaging</H4>
          <div
            style={{
              width: "0.75rem",
              height: "0.75rem",
              borderRadius: "50%",
              backgroundColor: hdrBurstMode
                ? hdrBurstInProgress
                  ? "orange"
                  : "green"
                : "gray",
            }}
          />
        </HStack>

        {/* Sensor Status */}
        <VStack align="start" spacing={2} w="100%">
          <Body3 fontSize="sm" fontWeight="bold">
            Available Sensors:
          </Body3>
          <HStack spacing={4} fontSize="xs">
            <HStack>
              <div
                style={{
                  width: "0.5rem",
                  height: "0.5rem",
                  borderRadius: "50%",
                  backgroundColor:
                    lucidCameras.length >= 2
                      ? "green"
                      : lucidCameras.length > 0
                      ? "orange"
                      : "red",
                }}
              />
              <Body3 fontSize="xs">Lucid ({lucidCameras.length}/2)</Body3>
            </HStack>
            <HStack>
              <div
                style={{
                  width: "0.5rem",
                  height: "0.5rem",
                  borderRadius: "50%",
                  backgroundColor: lidarSensors.length > 0 ? "green" : "red",
                }}
              />
              <Body3 fontSize="xs">LIDAR ({lidarSensors.length}/1)</Body3>
            </HStack>
            <HStack>
              <div
                style={{
                  width: "0.5rem",
                  height: "0.5rem",
                  borderRadius: "50%",
                  backgroundColor:
                    realsenseCameras.length > 0 ? "green" : "red",
                }}
              />
              <Body3 fontSize="xs">
                RealSense ({realsenseCameras.length}/1)
              </Body3>
            </HStack>
          </HStack>
        </VStack>

        {/* Storage Mode Information */}
        {storageEnabled && (
          <Alert
            status={hdrBurstMode ? "info" : "success"}
            borderRadius="md"
            fontSize="sm"
          >
            <AlertIcon />
            <VStack align="start" spacing={0} flex="1">
              <Body3 fontSize="xs" fontWeight="bold">
                Storage: {hdrBurstMode ? "HDR Burst Mode" : "Regular Mode"}
              </Body3>
              <Body3 fontSize="xs">
                {hdrBurstMode
                  ? "Only HDR burst frames will be saved. Regular frame collection is paused."
                  : "Regular frame collection is being saved continuously."}
              </Body3>
            </VStack>
          </Alert>
        )}
        <VStack spacing={2} w="100%">
          <HStack>
            <Body3 fontSize="sm">Mode:</Body3>
            <Code fontSize="xs">{hdrBurstMode ? "Enabled" : "Disabled"}</Code>
          </HStack>
        </VStack>
        <HStack>
          {hdrBurstInProgress && (
            <>
              <Body3 fontSize="sm">Status:</Body3>
              <Code fontSize="xs" maxW="12rem" isTruncated>
                {getStepDescription(hdrBurstCurrentStep)}
              </Code>
            </>
          )}
        </HStack>

        {hdrBurstInProgress && (
          <VStack w="100%" spacing={2}>
            <HStack justifyContent="space-between" w="100%">
              <Body3 fontSize="sm">Overall Progress</Body3>
              <Body3 fontSize="sm">{getOverallProgress()}%</Body3>
            </HStack>
            <div
              style={{
                width: "100%",
                height: "0.5rem",
                backgroundColor: "#e0e0e0",
                borderRadius: "0.25rem",
                overflow: "hidden",
              }}
            >
              <div
                style={{
                  width: `${getOverallProgress()}%`,
                  height: "100%",
                  backgroundColor: hdrBurstCurrentStep.includes("error")
                    ? "#dc3545"
                    : "#4A90E2",
                  transition: "width 0.3s ease",
                }}
              />
            </div>

            {Object.keys(hdrBurstProgress).length > 0 && (
              <VStack w="100%" spacing={1}>
                <Body3 fontSize="sm" alignSelf="start">
                  Camera Progress:
                </Body3>
                {Object.entries(hdrBurstProgress).map(
                  ([cameraName, progress]: [string, any]) => (
                    <HStack
                      key={cameraName}
                      justifyContent="space-between"
                      w="100%"
                    >
                      <Body3 fontSize="xs">
                        {cameraName.replace("lucid_", "")}:
                      </Body3>
                      <HStack spacing={1}>
                        <Body3 fontSize="xs">
                          {progress?.current_exposure || 0}/
                          {progress?.total_exposures || 4}
                        </Body3>
                        <div
                          style={{
                            width: "3rem",
                            height: "0.3rem",
                            backgroundColor: "#e0e0e0",
                            borderRadius: "0.15rem",
                            overflow: "hidden",
                          }}
                        >
                          <div
                            style={{
                              width: `${getProgressPercentage(progress)}%`,
                              height: "100%",
                              backgroundColor: "#28a745",
                              transition: "width 0.3s ease",
                            }}
                          />
                        </div>
                      </HStack>
                    </HStack>
                  )
                )}
              </VStack>
            )}

            {/* High-gain Processing Progress */}
            {hdrHighGainProcessing &&
              Object.keys(hdrHighGainProcessingProgress).length > 0 && (
                <VStack w="100%" spacing={2} pt={2}>
                  <HStack w="100%" justifyContent="space-between">
                    <Body3 fontSize="sm" fontWeight="bold">
                      High-gain Processing:
                    </Body3>
                    <Badge colorScheme="purple" size="sm">
                      Merging Images
                    </Badge>
                  </HStack>

                  {Object.entries(hdrHighGainProcessingProgress).map(
                    ([cameraName, progress]: [string, any]) => (
                      <VStack
                        key={cameraName}
                        w="100%"
                        spacing={1}
                        bg="gray.50"
                        p={2}
                        borderRadius="md"
                      >
                        <HStack justifyContent="space-between" w="100%">
                          <Body3 fontSize="xs" fontWeight="bold">
                            {cameraName.replace("lucid_", "")}:
                          </Body3>
                          <Body3 fontSize="xs" color="gray.600">
                            {progress?.progress_percent || 0}%
                          </Body3>
                        </HStack>

                        <div
                          style={{
                            width: "100%",
                            height: "0.4rem",
                            backgroundColor: "#e0e0e0",
                            borderRadius: "0.2rem",
                            overflow: "hidden",
                          }}
                        >
                          <div
                            style={{
                              width: `${progress?.progress_percent || 0}%`,
                              height: "100%",
                              backgroundColor: "#8B5CF6",
                              transition: "width 0.3s ease",
                            }}
                          />
                        </div>

                        <HStack
                          justifyContent="space-between"
                          w="100%"
                          fontSize="xs"
                        >
                          <Body3 fontSize="xs" color="gray.600">
                            {progress?.current_step === "merging" &&
                              "Starting merge..."}
                            {progress?.current_step === "extracting_images" &&
                              "Extracting images..."}
                            {progress?.current_step === "format_conversion" &&
                              "Converting format..."}
                            {progress?.current_step === "outlier_filtering" &&
                              "Filtering outliers..."}
                            {progress?.current_step === "denoising" &&
                              "Denoising..."}
                            {progress?.current_step === "finalizing" &&
                              "Finalizing..."}
                            {progress?.current_step === "completed" &&
                              "Completed ✓"}
                          </Body3>

                          {progress?.total_frames && (
                            <Body3 fontSize="xs" color="gray.500">
                              {progress.total_frames} frames
                              {progress?.kept_frames !== undefined &&
                                progress?.dropped_frames !== undefined && (
                                  <span>
                                    {" "}
                                    ({progress.kept_frames} kept,{" "}
                                    {progress.dropped_frames} dropped)
                                  </span>
                                )}
                            </Body3>
                          )}
                        </HStack>
                      </VStack>
                    )
                  )}
                </VStack>
              )}
          </VStack>
        )}

        <HStack spacing={2} w="100%">
          {!hdrBurstMode ? (
            <Btn
              icon={<CameraPlus />}
              color={Color.Blue}
              onClick={
                hdrBurstInProgress ? undefined : lucidStore.fetchEnableHDRBurst
              }
              varient="filled"
              size="sm"
            >
              Enable HDR Mode
            </Btn>
          ) : (
            <Btn
              icon={<Pause />}
              color={Color.Orange}
              onClick={
                hdrBurstInProgress ? undefined : lucidStore.fetchDisableHDRBurst
              }
              varient="filled"
              size="sm"
            >
              Disable HDR Mode
            </Btn>
          )}

          {hdrBurstMode && (
            <Btn
              icon={<Record />}
              color={hdrBurstInProgress ? Color.Orange : Color.Green}
              onClick={
                hdrBurstInProgress || !storageEnabled
                  ? undefined
                  : lucidStore.fetchCaptureHDRBurst
              }
              varient="filled"
              size="sm"
            >
              {hdrBurstInProgress ? "Capturing..." : "Capture HDR"}
            </Btn>
          )}
        </HStack>

        {/* High-Gain Mode Control */}
        {hdrBurstMode && (
          <VStack align="start" spacing={2} w="100%">
            <HStack justifyContent="space-between" w="100%">
              <VStack align="start" spacing={0}>
                <Body3 fontSize="sm" fontWeight="bold">
                  High-Gain Burst
                </Body3>
                <Body3 fontSize="xs" color="gray.600">
                  Captures 8 additional frames at gain 4.0 for dark scenes
                </Body3>
              </VStack>
              <HStack>
                <Body3 fontSize="xs">
                  {hdrHighGainMode ? "Enabled" : "Disabled"}
                </Body3>
                <div
                  style={{
                    width: "0.5rem",
                    height: "0.5rem",
                    borderRadius: "50%",
                    backgroundColor: hdrHighGainMode ? "green" : "gray",
                  }}
                />
              </HStack>
            </HStack>

            <HStack spacing={2} w="100%">
              {!hdrHighGainMode ? (
                <Btn
                  color={Color.Green}
                  onClick={
                    hdrBurstInProgress
                      ? undefined
                      : lucidStore.fetchEnableHDRHighGain
                  }
                  varient="outline"
                  size="sm"
                >
                  Enable High-Gain
                </Btn>
              ) : (
                <Btn
                  color={Color.Orange}
                  onClick={
                    hdrBurstInProgress
                      ? undefined
                      : lucidStore.fetchDisableHDRHighGain
                  }
                  varient="outline"
                  size="sm"
                >
                  Disable High-Gain
                </Btn>
              )}

              {hdrHighGainMode && hdrBurstCurrentStep === "high_gain_burst" && (
                <Body3 fontSize="xs" color="blue.600">
                  Capturing frame {hdrHighGainBurstCount}/8
                </Body3>
              )}
            </HStack>
          </VStack>
        )}

        {!storageEnabled && hdrBurstMode && (
          <VStack
            p={2}
            bg="yellow.100"
            borderRadius="md"
            align="start"
            spacing={1}
            w="100%"
          >
            <Body3 fontSize="sm" color="orange.600">
              ⚠️ Storage must be enabled for HDR capture
            </Body3>
          </VStack>
        )}

        {hdrBurstMode && lucidCameras.length < 2 && (
          <VStack
            p={2}
            bg="red.100"
            borderRadius="md"
            align="start"
            spacing={1}
            w="100%"
          >
            <Body3 fontSize="sm" color="red.600">
              ⚠️ At least 2 Lucid cameras must be connected
            </Body3>
          </VStack>
        )}

        <VStack
          p={3}
          bg="gray.50"
          borderRadius="md"
          align="start"
          spacing={2}
          w="100%"
        >
          <Body3 fontSize="xs" fontWeight="bold" color="gray.700">
            HDR Burst Imaging Info:
          </Body3>
          <Body3 fontSize="xs" color="gray.600">
            • Captures 4 different exposures: 88μs, 880μs, 8.8ms, 88ms
          </Body3>
          <Body3 fontSize="xs" color="gray.600">
            • Synchronized capture from both Lucid cameras
          </Body3>
          <Body3 fontSize="xs" color="gray.600">
            • Simultaneously collects LIDAR and RealSense data
          </Body3>
          <Body3 fontSize="xs" color="gray.600">
            • Offline sensors are automatically skipped
          </Body3>
          <Body3 fontSize="xs" color="gray.600">
            • Designed for static scenes (no time sync required)
          </Body3>
        </VStack>

        {/* Brightness Validation Monitor */}
        {hdrBurstMode &&
          (lucidStore.brightnessAnalysisList.length > 0 ||
            lucidStore.hdrFinalVerification) && (
            <VStack
              p={3}
              bg="blue.50"
              borderRadius="md"
              align="start"
              spacing={3}
              w="100%"
            >
              <HStack spacing={2} w="100%">
                <Icon as={InfoIcon} color="blue.600" />
                <Body3 fontSize="sm" fontWeight="bold" color="blue.700">
                  Brightness Validation Monitor
                </Body3>
              </HStack>

              {/* Overall Progress */}
              {lucidStore.hdrBurstProgress && (
                <VStack spacing={1} w="100%" align="start">
                  <HStack justify="space-between" w="100%">
                    <Body3 fontSize="xs" color="gray.700">
                      Step: {lucidStore.hdrBurstProgress.step}
                    </Body3>
                    <Body3 fontSize="xs" color="gray.700">
                      {lucidStore.hdrBurstProgress.completed}/
                      {lucidStore.hdrBurstProgress.total}
                    </Body3>
                  </HStack>
                  <Progress
                    value={
                      (lucidStore.hdrBurstProgress.completed /
                        lucidStore.hdrBurstProgress.total) *
                      100
                    }
                    size="sm"
                    colorScheme="blue"
                    w="100%"
                  />
                </VStack>
              )}

              {/* Latest Brightness Analysis */}
              {lucidStore.brightnessAnalysisList.length > 0 && (
                <VStack spacing={2} w="100%" align="start">
                  <Body3 fontSize="xs" fontWeight="medium" color="gray.700">
                    Latest Analysis:
                  </Body3>
                  <SimpleGrid columns={2} spacing={3} w="100%">
                    {Object.entries(
                      lucidStore.brightnessAnalysisList[
                        lucidStore.brightnessAnalysisList.length - 1
                      ]?.camera_results || {}
                    ).map(([cameraId, result]) => (
                      <VStack
                        key={cameraId}
                        p={2}
                        bg="white"
                        borderRadius="md"
                        align="start"
                        spacing={2}
                        border="1px solid"
                        borderColor="gray.200"
                      >
                        <HStack spacing={2} w="100%">
                          <Badge colorScheme="gray" fontSize="xs">
                            {cameraId}
                          </Badge>
                          <Badge
                            colorScheme={result.valid ? "green" : "red"}
                            fontSize="xs"
                          >
                            {result.valid ? "Valid" : "Invalid"}
                          </Badge>
                        </HStack>

                        {/* Current Exposure Info */}
                        <HStack spacing={2}>
                          <Body3 fontSize="xs" color="gray.600">
                            Exposure:
                          </Body3>
                          <Badge colorScheme="blue" fontSize="xs">
                            {result.exposure_us}μs
                          </Badge>
                        </HStack>

                        {/* Brightness Values */}
                        <VStack spacing={1} align="start" w="100%">
                          <Body3
                            fontSize="xs"
                            fontWeight="medium"
                            color="gray.700"
                          >
                            Brightness:
                          </Body3>
                          <HStack spacing={2} fontSize="xs">
                            <Text color="gray.500">Mean:</Text>
                            <Text color="gray.700" fontFamily="mono">
                              {result.mean.toFixed(4)}
                            </Text>
                          </HStack>
                          <HStack spacing={2} fontSize="xs">
                            <Text color="gray.500">Std:</Text>
                            <Text color="gray.700" fontFamily="mono">
                              {result.std.toFixed(4)}
                            </Text>
                          </HStack>
                        </VStack>

                        {/* Validation Details */}
                        <VStack spacing={1} align="start" w="100%">
                          <Body3
                            fontSize="xs"
                            fontWeight="medium"
                            color="gray.700"
                          >
                            Status:
                          </Body3>
                          <Text
                            fontSize="xs"
                            color={result.valid ? "green.600" : "red.600"}
                          >
                            {result.reason}
                          </Text>
                        </VStack>
                      </VStack>
                    ))}
                  </SimpleGrid>
                </VStack>
              )}

              {/* Final Verification Status */}
              {lucidStore.hdrFinalVerification && (
                <VStack
                  p={2}
                  bg={
                    lucidStore.hdrFinalVerification.overall_success
                      ? "green.50"
                      : "red.50"
                  }
                  borderRadius="md"
                  align="start"
                  spacing={1}
                  w="100%"
                  border="1px solid"
                  borderColor={
                    lucidStore.hdrFinalVerification.overall_success
                      ? "green.200"
                      : "red.200"
                  }
                >
                  <HStack spacing={2}>
                    <Icon
                      as={
                        lucidStore.hdrFinalVerification.overall_success
                          ? CheckCircleIcon
                          : WarningIcon
                      }
                      color={
                        lucidStore.hdrFinalVerification.overall_success
                          ? "green.600"
                          : "red.600"
                      }
                    />
                    <Body3
                      fontSize="sm"
                      fontWeight="bold"
                      color={
                        lucidStore.hdrFinalVerification.overall_success
                          ? "green.700"
                          : "red.700"
                      }
                    >
                      Final Verification:{" "}
                      {lucidStore.hdrFinalVerification.overall_success
                        ? "Passed"
                        : "Failed"}
                    </Body3>
                  </HStack>

                  {/* Show camera verification details */}
                  <VStack spacing={1} align="start" w="100%">
                    {Object.entries(
                      lucidStore.hdrFinalVerification.camera_results
                    ).map(([cameraId, result]) => (
                      <HStack key={cameraId} spacing={2}>
                        <Text fontSize="xs" color="gray.600">
                          {cameraId}:
                        </Text>
                        <Badge
                          colorScheme={result.is_valid ? "green" : "red"}
                          fontSize="xs"
                        >
                          {result.is_valid ? "Valid" : "Invalid"}
                        </Badge>
                        <Text fontSize="xs" color="gray.500">
                          (Monotonic: {result.is_monotonic ? "✓" : "✗"})
                        </Text>
                      </HStack>
                    ))}
                  </VStack>
                </VStack>
              )}
            </VStack>
          )}
      </VStack>
    </Card>
  );
});

export const SensorsPage = observer(() => {
  useEffect(() => {
    sensorsStore.fetchGetSensorsList();
  }, []);

  return (
    <PageRoot title="Sensors">
      <Flex>
        {sensorsStore.sensors.map((sensor, i) => {
          return <SensorView key={i} sensor={sensor} />;
        })}
      </Flex>
      <SensorGroupView groups={sensorsStore.sensorGroups} />
      <Flex gap={4}>
        <StorageControl />
        <HDRBurstControl />
      </Flex>
    </PageRoot>
  );
});
