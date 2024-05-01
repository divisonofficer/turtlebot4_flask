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
  Switch,
  VStack,
  useDisclosure,
} from "@chakra-ui/react";
import { Btn } from "../../design/button/button";
import { Body3, H4 } from "../../design/text/textsystem";
import { captureStore } from "../../stores/CaptureStore";
import { observer } from "mobx-react";
import { slamStore } from "../../stores/SlamStore";
import { CaptureSpaceModal } from "./CaptureSpaceModal";
import { Color } from "../../design/color";

const CaptureSourceSwitch = observer(() => {
  return (
    <VStack alignItems="flex-start">
      {captureStore.image_topic_switches.map((switchItem, index) => {
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
              isChecked={switchItem.checked}
              onChange={(e) => {
                if (e.target.checked) {
                  captureStore.topicSwitchOn(switchItem.topic);
                } else {
                  captureStore.topicSwitchOff(switchItem.topic);
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
    isOpen: isInitPopoverOpen,
    onOpen: onInitPopoverOpen,
    onClose: onInitPopoverClose,
  } = useDisclosure();

  return (
    <VStack>
      <H4>Slam is {slamStore.slamStatus?.status}</H4>
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
                  ? "Space ID : " + captureStore.space_id
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
                        onClick={() => {
                          captureStore.initSpace();
                          onInitPopoverClose();
                        }}
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
          {" "}
          <HStack>
            <Btn onClick={() => captureStore.fetchPostCaptureSingle()}>
              Single
            </Btn>
            <Btn>Queue</Btn>{" "}
          </HStack>
          <CaptureSourceSwitch />
        </>
      )}
    </Flex>
  );
});
