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
import { JAIDeviceInfo, jaiStore } from "../../stores/JaiStore";
import { Body3, H4 } from "../../design/text/textsystem";
import { VideoStream } from "../../design/other/video";
import { observer } from "mobx-react";
import { ArrowArcRight } from "@phosphor-icons/react";

export const SourceDeviceParamSlide = observer(
  (props: {
    device: JAIDeviceInfo;
    sourceIndex: number;
    paramIndex: number;
  }) => {
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
  device: JAIDeviceInfo;
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
      {props.device.configurable.map((_, index) => {
        return (
          <SourceDeviceParamSlide
            device={props.device}
            sourceIndex={props.sourceIndex}
            paramIndex={index}
            key={index}
          />
        );
      })}
    </VStack>
  );
};

export const JaiDeviceControl = (props: { device: JAIDeviceInfo }) => {
  return (
    <VStack>
      <H4>{props.device.name}</H4>
      <IconButton
        icon={<ArrowArcRight />}
        onClick={() => jaiStore.fetchJaiDeviceParam()}
        aria-label=""
      />
      <HStack>
        {Array.from({ length: props.device.source_count }).map((_, index) => {
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
    <HStack>
      {jaiStore.jaiDeviceInfo.map((device, index) => {
        return <JaiDeviceControl device={device} key={index} />;
      })}
    </HStack>
  );
});

export const JaiBridgePage = () => {
  return (
    <PageRoot title="Jai Bridge">
      <JAiBridgeControl />
    </PageRoot>
  );
};