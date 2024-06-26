import {
  Button,
  Flex,
  HStack,
  Input,
  Menu,
  MenuButton,
  MenuItem,
  MenuList,
  Popover,
  PopoverArrow,
  PopoverBody,
  PopoverCloseButton,
  PopoverContent,
  PopoverHeader,
  PopoverTrigger,
  Progress,
  ProgressLabel,
  Slider,
  SliderFilledTrack,
  SliderThumb,
  SliderTrack,
  Switch,
  VStack,
  useDisclosure,
} from "@chakra-ui/react";
import { Btn } from "../../design/button/button";
import { Body3, H4 } from "../../design/text/textsystem";
import { captureStore } from "../../stores/CaptureStore";
import { observer } from "mobx-react";
import { slamStore } from "../../stores/SlamStore";
import { CaptureSpaceInitModal, CaptureSpaceModal } from "./CaptureSpaceModal";
import { Color } from "../../design/color";
import { useEffect, useState } from "react";
import { httpGet, httpPost } from "../../connect/http/request";
import { jaiStore } from "../../stores/JaiStore";
import { ParameterEnum } from "../../public/proto/jai";
import {
  FloppyDisk,
  Newspaper,
  Parallelogram,
  RocketLaunch,
  SunDim,
} from "@phosphor-icons/react";
import { ChevronDownIcon, CloseIcon } from "@chakra-ui/icons";
import {
  CaptureScenarioHyperparameter_HyperParameter,
  CaptureScenarioHyperparameter_ParameterType,
} from "../../public/proto/capture";

const CaptureSourceSwitch = observer(() => {
  return (
    <VStack alignItems="flex-start">
      {Object.values(captureStore.def_switchs).map((switchItem, index) => {
        return (
          <Flex
            key={index}
            style={{
              alignItems: "center",
              flexDirection: "row",
            }}
            gap="2"
          >
            <Switch
              isChecked={switchItem.enabled}
              onChange={(e) => {
                if (e.target.checked) {
                  captureStore.topicSwitchOn(switchItem.name);
                } else {
                  captureStore.topicSwitchOff(switchItem.name);
                }
              }}
            />
            <H4>{switchItem.name}</H4>
          </Flex>
        );
      })}
    </VStack>
  );
});

export const CaptureSpaceControl = observer(() => {
  const {
    isOpen: isSpaceModalOpen,
    onOpen: onSpaceModalOpen,
    onClose: onSpaceModalClose,
  } = useDisclosure();
  const {
    isOpen: isSpaceCreateModalOpen,
    onOpen: onSpaceCreateModalOpen,
    onClose: onSpaceCreateModalClose,
  } = useDisclosure();
  const {
    isOpen: isInitPopoverOpen,
    onOpen: onInitPopoverOpen,
    onClose: onInitPopoverClose,
  } = useDisclosure();

  return (
    <VStack>
      <H4>
        Slam is
        {[" Success", " Error", " Offline"][slamStore.slamStatus?.status || 0]}
      </H4>
      <VStack alignItems="flex-start">
        <HStack>
          <Popover
            isOpen={isInitPopoverOpen}
            onOpen={onInitPopoverOpen}
            onClose={onInitPopoverClose}
          >
            <PopoverTrigger>
              <Button size="sm" variant={"outline"}>
                {captureStore.space_id && captureStore.space_id !== null
                  ? captureStore.space_name
                    ? captureStore.space_name
                    : "Space ID : " + captureStore.space_id
                  : "Initiate Space"}
              </Button>
            </PopoverTrigger>
            <PopoverContent>
              <PopoverArrow />
              <PopoverCloseButton />
              <PopoverHeader>
                {captureStore.space_id === undefined && <H4>Initiate Space</H4>}
                {captureStore.space_id !== undefined &&
                  !captureStore.is_capture_running && <H4>Load other Space</H4>}
                {captureStore.space_id !== undefined &&
                  captureStore.is_capture_running && (
                    <H4>Space is on capture now</H4>
                  )}
              </PopoverHeader>
              <PopoverBody>
                <HStack>
                  {captureStore.is_capture_running && (
                    <>
                      <Body3>Back to the control pannel</Body3>
                    </>
                  )}
                  {!captureStore.is_capture_running && (
                    <>
                      <Btn
                        size="sm"
                        varient="outline"
                        onClick={onSpaceCreateModalOpen}
                      >
                        Create new Space
                      </Btn>
                      <Btn
                        size="sm"
                        varient="outline"
                        onClick={onSpaceModalOpen}
                      >
                        Load Saved Space
                      </Btn>
                    </>
                  )}
                </HStack>
              </PopoverBody>
            </PopoverContent>
          </Popover>
          {isSpaceModalOpen && (
            <CaptureSpaceModal
              onClose={onSpaceModalClose}
              isOpen={isSpaceModalOpen}
            />
          )}
          {isSpaceCreateModalOpen && (
            <CaptureSpaceInitModal
              onClose={onSpaceCreateModalClose}
              isOpen={isSpaceCreateModalOpen}
            />
          )}
        </HStack>
        <HStack>
          <H4>Map name</H4>
          <Btn size="sm" varient="outline">
            {captureStore.map_name}
          </Btn>
        </HStack>
      </VStack>
    </VStack>
  );
});

export const MultiSpectralCameraControl = observer(() => {
  return (
    <VStack
      style={{
        alignItems: "flex-start",
      }}
    >
      <H4>JAI Master Control</H4>
      <Btn
        onClick={() => captureStore.fetchMultiSpectralInit()}
        color={Color.Cyan}
        size="sm"
        icon={<Newspaper />}
      >
        CameraInit
      </Btn>
      <Btn
        onClick={() => captureStore.fetchMultiSpectralOpen()}
        color={Color.Cyan}
        size="sm"
        icon={<RocketLaunch />}
      >
        StreamOpen
      </Btn>
      <Btn
        onClick={() => captureStore.fetchMultiSpectralClose()}
        size="sm"
        color={Color.Orange}
        icon={<CloseIcon />}
      >
        StreamClose
      </Btn>{" "}
      <Btn
        size="sm"
        icon={<SunDim />}
        onClick={() => {
          jaiStore.fetchJaiCameraHoldAutoExposureAll(false);
        }}
      >
        AutoExposureOn
      </Btn>
    </VStack>
  );
});

export const CaptureScenarioHyperparameterControl = observer(() => {
  const parameter = captureStore.scenario_hyperparameters;

  return parameter ? (
    <VStack>
      {parameter.hyperparameters.map((param, index) => (
        <HStack
          style={{
            width: "20rem",
          }}
        >
          <Body3>{param.name}</Body3>
          {param.type ===
          CaptureScenarioHyperparameter_ParameterType.BOOLEAN ? (
            <>
              <div style={{ width: "100%" }} />
              <Switch
                isChecked={param.value === 1}
                onChange={(e) => {
                  captureStore.fetchScenarioHyperparameterUpdate(
                    param.name,
                    e.target.checked ? 1 : 0
                  );
                }}
              />
            </>
          ) : param.type ===
            CaptureScenarioHyperparameter_ParameterType.DOUBLE ? (
            <>
              <Slider
                min={param.range[0]}
                max={param.range[1]}
                size={"md"}
                style={{
                  width: "100%",

                  height: "1rem",
                }}
                step={param.gap}
                value={param.value}
                onChange={(value) => {
                  captureStore.fetchScenarioHyperparameterUpdate(
                    param.name,
                    value
                  );
                }}
              >
                <SliderTrack>
                  <SliderFilledTrack />
                </SliderTrack>
                <SliderThumb />
              </Slider>
              <H4>{param.value}</H4>
            </>
          ) : param.type ===
            CaptureScenarioHyperparameter_ParameterType.DOUBLE_ARRAY ? (
            <HyperParameterDoubleArray paramter={param} />
          ) : (
            <>
              <Menu placement="bottom-end">
                <MenuButton as={Button} rightIcon={<ChevronDownIcon />}>
                  {param.enumValues[param.value]}
                </MenuButton>
                <MenuList>
                  {param.enumValues.map((def, index) => {
                    return (
                      <MenuItem
                        key={index}
                        fontSize="small"
                        onClick={() => {
                          captureStore.fetchScenarioHyperparameterUpdate(
                            param.name,
                            index
                          );
                        }}
                      >
                        {def}
                      </MenuItem>
                    );
                  })}
                </MenuList>
              </Menu>
            </>
          )}
        </HStack>
      ))}
    </VStack>
  ) : (
    <></>
  );
});

const HyperParameterDoubleArray = (props: {
  paramter: CaptureScenarioHyperparameter_HyperParameter;
}) => {
  const [valueArray, setValueArray] = useState(props.paramter.valueArray);
  return (
    <HStack>
      <Input
        value={valueArray.join(",")}
        onChange={(e) => {
          setValueArray(
            e.target.value.split(",").map((angle) => parseInt(angle))
          );
        }}
      />
      <Btn
        size="sm"
        onClick={() => {
          captureStore.fetchScenarioHyperparameterUpdate(
            props.paramter.name,
            undefined,
            valueArray
          );
        }}
        icon={<FloppyDisk style={{ width: "1rem", height: "1rem" }} />}
      ></Btn>
    </HStack>
  );
};

export const CaptureControl = observer(() => {
  return (
    <Flex wrap="wrap" gap="2" width="100%">
      <CaptureSpaceControl />
      <HStack>
        {captureStore.is_capture_running && (
          <>
            <Btn
              color={Color.Red}
              onClick={() => captureStore.disableSpaceCapture()}
            >
              Quit
            </Btn>
          </>
        )}
        {!captureStore.is_capture_running && (
          <>
            <Btn
              color={Color.Indigo}
              onClick={() => captureStore.enableSpaceCapture()}
            >
              GO LIVE
            </Btn>
          </>
        )}
      </HStack>

      {captureStore.is_capture_running && (
        <>
          {captureStore.progress.length > 0 && (
            <>
              <Btn onClick={() => captureStore.fetchAbortCapture()}>Abort</Btn>
              {captureStore.progress.map((progress, index) => (
                <VStack>
                  <Progress
                    value={progress.progress}
                    style={{
                      width: "20rem",
                      height: "5rem",
                    }}
                  >
                    <ProgressLabel>{progress.progress}%</ProgressLabel>
                  </Progress>
                  <Body3>{progress.message}</Body3>
                </VStack>
              ))}
            </>
          )}
          {captureStore.progress.length < 1 && (
            <>
              <HStack>
                <Btn onClick={() => captureStore.fetchPostCaptureQueue()}>
                  BeginCapture
                </Btn>{" "}
              </HStack>

              <CaptureSourceSwitch />

              <MultiSpectralCameraControl />

              <CaptureScenarioHyperparameterControl />
            </>
          )}
        </>
      )}
      {captureStore.is_capture_running && captureStore.captureTimestampLog && (
        <VStack
          style={{
            alignItems: "flex-start",
            height: "15rem",
            overflowY: "scroll",
          }}
        >
          <H4>TimeStamp Logs</H4>
          {captureStore.captureTimestampLog.logs.map((log, index) => (
            <HStack>
              <Body3>{log.topic}</Body3>
              <Body3>{log.delayToSystem.toFixed(4)} ms</Body3>
            </HStack>
          ))}
        </VStack>
      )}
    </Flex>
  );
});
