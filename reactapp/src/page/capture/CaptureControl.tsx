import {
  Button,
  Flex,
  HStack,
  Popover,
  PopoverArrow,
  PopoverBody,
  PopoverCloseButton,
  PopoverContent,
  PopoverHeader,
  PopoverTrigger,
  Progress,
  ProgressLabel,
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
import { SlamState_Status } from "../../public/proto/slam";

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
    <HStack>
      <Btn
        onClick={() => captureStore.fetchMultiSpectralInit()}
        color={Color.Cyan}
        size="sm"
      >
        MultiSpectralInit
      </Btn>
      <Btn
        onClick={() => captureStore.fetchMultiSpectralClose()}
        size="sm"
        color={Color.Orange}
      >
        MultiSpectralClose
      </Btn>{" "}
    </HStack>
  );
});

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
                <Btn onClick={() => captureStore.fetchPostCaptureSingle()}>
                  Single
                </Btn>
                <Btn onClick={() => captureStore.fetchPostCaptureQueue()}>
                  Queue
                </Btn>{" "}
              </HStack>

              <CaptureSourceSwitch />

              <MultiSpectralCameraControl />
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
