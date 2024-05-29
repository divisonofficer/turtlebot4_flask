import {
  HStack,
  IconButton,
  Slider,
  SliderFilledTrack,
  SliderThumb,
  SliderTrack,
  VStack,
} from "@chakra-ui/react";
import { PageRoot } from "../../design/other/flexs";
import { jaiStore } from "../../stores/JaiStore";
import {
  DeviceInfo,
  ParameterInfo,
  ParameterInfo_Source,
} from "../../public/proto/jai";
import { Body3, H4 } from "../../design/text/textsystem";
import { VideoStream } from "../../design/other/video";
import { observer } from "mobx-react";
import { BookOpen, FloppyDisk, Pause } from "@phosphor-icons/react";
import { PolarizerControl } from "./PolarizerControl";
import { Play } from "@phosphor-icons/react/dist/ssr";
import { CloseIcon, ViewIcon } from "@chakra-ui/icons";
import { Btn } from "../../design/button/button";
import { Color } from "../../design/color";
import { captureStore } from "../../stores/CaptureStore";

export const SourceDeviceParamSlide = observer(
  (props: { device: DeviceInfo; sourceIndex: number; paramIndex: number }) => {
    const deviceName = props.device.name;
    const paramInfo = jaiStore.jaiCameraParams[deviceName]
      ? jaiStore.jaiCameraParams[deviceName][props.sourceIndex]
      : undefined;

    const paramDefine = props.device.configurable[props.paramIndex];
    const value =
      paramInfo && paramInfo[paramDefine.name as keyof typeof paramInfo]
        ? paramInfo[paramDefine.name as keyof typeof paramInfo].value
        : 0;
    return (
      <VStack
        style={{
          width: "100%",
        }}
      >
        <HStack
          style={{
            width: "100%",
            justifyContent: "space-between",
          }}
        >
          <Body3>{paramDefine.name}</Body3>
          <Body3>{value}</Body3>
        </HStack>
        <Slider
          min={paramDefine.min}
          max={paramDefine.max}
          size={"md"}
          style={{
            width: "100%",

            height: "1rem",
          }}
          step={1}
          value={value}
          onChange={(value) => {
            jaiStore.fetchJaiCameraUpdateParam(
              deviceName,
              props.sourceIndex,
              paramDefine.name,
              paramDefine.type,
              value
            );
          }}
        >
          <SliderTrack>
            <SliderFilledTrack />
          </SliderTrack>
          <SliderThumb />
        </Slider>
      </VStack>
    );
  }
);

export const JaiDeviceSource = (props: {
  device: DeviceInfo;
  sourceIndex: number;
}) => {
  return (
    <VStack>
      <H4>
        {props.device.name} {props.sourceIndex}
      </H4>

      <VideoStream
        url={`/jai/preview/${props.device.name}/${props.sourceIndex}`}
        style={{
          width: "24rem",
          height: "18rem",
        }}
      />
      {props.device.configurable.map((conf, index) => {
        return (
          !(
            conf.source === ParameterInfo_Source.DEVICE &&
            props.sourceIndex === 1
          ) && (
            <SourceDeviceParamSlide
              device={props.device}
              sourceIndex={props.sourceIndex}
              paramIndex={index}
              key={index}
            />
          )
        );
      })}
    </VStack>
  );
};

export const JaiDeviceControl = (props: { device: DeviceInfo }) => {
  return (
    <VStack>
      <H4>{props.device.name}</H4>
      <HStack>
        <IconButton
          icon={<ViewIcon />}
          onClick={() => jaiStore.fetchJaiDeviceParam()}
          aria-label="Refresh Parameters"
        />
        <IconButton
          icon={<Play />}
          onClick={() =>
            jaiStore.fetchJaiCameraStreamControl(props.device.name, true)
          }
          aria-label="Open Camera Stream"
        />
        <IconButton
          icon={<Pause />}
          onClick={() =>
            jaiStore.fetchJaiCameraStreamControl(props.device.name, false)
          }
          aria-label="Close Camera Stream"
        />
      </HStack>

      <HStack
        style={{
          alignItems: "flex-start",
        }}
      >
        {Array.from({ length: props.device.sourceCount }).map((_, index) => {
          return (
            <JaiDeviceSource
              device={props.device}
              sourceIndex={index}
              key={index}
            />
          );
        })}
      </HStack>
    </VStack>
  );
};

export const JAiBridgeControl = observer(() => {
  return (
    <VStack>
      <HStack>
        <Btn
          icon={<FloppyDisk />}
          onClick={() => {
            jaiStore.fetchJaiParameterSave();
          }}
          size="sm"
          color={Color.Indigo}
        >
          Save Parameters
        </Btn>
        <Btn
          icon={<BookOpen />}
          onClick={() => {
            captureStore.fetchMultiSpectralInit();
          }}
          size="sm"
          color={Color.Green}
        >
          Force Parameters to eBUS
        </Btn>
        <Btn
          icon={<CloseIcon />}
          onClick={() => {
            captureStore.fetchMultiSpectralClose();
          }}
          size="sm"
          color={Color.Red}
        >
          Shutdown All Streams
        </Btn>
      </HStack>

      <HStack>
        {jaiStore.jaiDeviceInfo.map((device, index) => {
          return <JaiDeviceControl device={device} key={index} />;
        })}
      </HStack>
      <VideoStream
        url={`/jai/preview/oakd/0`}
        style={{
          width: "24rem",
          height: "18rem",
        }}
      />
    </VStack>
  );
});

export const JaiBridgePage = () => {
  return (
    <PageRoot title="Jai Bridge">
      <PolarizerControl />
      <JAiBridgeControl />
    </PageRoot>
  );
};
