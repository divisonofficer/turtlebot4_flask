import {
  Button,
  HStack,
  IconButton,
  Menu,
  MenuButton,
  MenuItem,
  MenuList,
  Slider,
  SliderFilledTrack,
  SliderThumb,
  SliderTrack,
  Switch,
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
import {
  BookOpen,
  FloppyDisk,
  Pause,
  Sun,
  SunDim,
  SunHorizon,
} from "@phosphor-icons/react";
import { PolarizerControl } from "./PolarizerControl";
import { Play } from "@phosphor-icons/react/dist/ssr";
import { ChevronDownIcon, CloseIcon, ViewIcon } from "@chakra-ui/icons";
import { Btn } from "../../design/button/button";
import { Color } from "../../design/color";
import { captureStore } from "../../stores/CaptureStore";
import { useEffect } from "react";
import { CalibrateView } from "./CalibrationView";

export const SourceDeviceParamSlide = observer(
  (props: { device: DeviceInfo; sourceIndex: number; paramIndex: number }) => {
    const deviceName = props.device.name;
    const paramInfo = jaiStore.jaiCameraParams[deviceName]
      ? jaiStore.jaiCameraParams[deviceName][props.sourceIndex]
      : undefined;

    const paramDefine = props.device.configurable[props.paramIndex];
    const value =
      paramInfo && paramInfo[paramDefine.name as keyof typeof paramInfo]
        ? Number.parseFloat(
            paramInfo[paramDefine.name as keyof typeof paramInfo].value
          )
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

export const SourceDeviceParamEnum = observer(
  (props: { device: DeviceInfo; sourceIndex: number; paramIndex: number }) => {
    const deviceName = props.device.name;
    const paramInfo = jaiStore.jaiCameraParams[deviceName]
      ? jaiStore.jaiCameraParams[deviceName][props.sourceIndex]
      : undefined;

    const paramDefine = props.device.configurable[props.paramIndex];
    const valueIdx =
      paramInfo && paramInfo[paramDefine.name as keyof typeof paramInfo]
        ? Number.parseInt(
            paramInfo[paramDefine.name as keyof typeof paramInfo].value
          )
        : -1;
    const value = paramDefine.enumDefs.find(
      (def) => def.index === valueIdx
    )?.value;
    return (
      <HStack
        style={{
          width: "100%",
          justifyContent: "space-between",
        }}
      >
        <HStack
          style={{
            width: "100%",
            justifyContent: "space-between",
          }}
        >
          <Body3>{paramDefine.name}</Body3>
        </HStack>
        <VStack
          style={{
            width: "6rem",
          }}
        >
          <Menu placement="bottom-end">
            <MenuButton as={Button} rightIcon={<ChevronDownIcon />}>
              {value}
            </MenuButton>
            <MenuList>
              {paramDefine.enumDefs.map((def, index) => {
                return (
                  <MenuItem
                    key={index}
                    fontSize="small"
                    onClick={() => {
                      jaiStore.fetchJaiCameraUpdateParam(
                        deviceName,
                        props.sourceIndex,
                        paramDefine.name,
                        paramDefine.type,
                        def.index
                      );
                    }}
                  >
                    {def.value}
                  </MenuItem>
                );
              })}
            </MenuList>
          </Menu>
        </VStack>
      </HStack>
    );
  }
);
export const SourceDeviceParamBool = observer(
  (props: { device: DeviceInfo; sourceIndex: number; paramIndex: number }) => {
    const deviceName = props.device.name;
    const paramInfo = jaiStore.jaiCameraParams[deviceName]
      ? jaiStore.jaiCameraParams[deviceName][props.sourceIndex]
      : undefined;

    const paramDefine = props.device.configurable[props.paramIndex];
    const valueStr =
      paramInfo && paramInfo[paramDefine.name as keyof typeof paramInfo]
        ? paramInfo[paramDefine.name as keyof typeof paramInfo].value
        : "false";
    const value =
      valueStr === "true" || valueStr === "1" || valueStr === "True";
    return (
      <HStack
        style={{
          width: "100%",
          justifyContent: "space-between",
        }}
      >
        <HStack
          style={{
            width: "100%",
            justifyContent: "space-between",
          }}
        >
          <Body3>{paramDefine.name}</Body3>
          <Body3>{valueStr}</Body3>
        </HStack>
        <Switch
          isChecked={value}
          onChange={(value) => {
            jaiStore.fetchJaiCameraUpdateParam(
              deviceName,
              props.sourceIndex,
              paramDefine.name,
              paramDefine.type,
              value.target.checked
            );
          }}
        />
      </HStack>
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
            <>
              {conf.type === "float" && (
                <SourceDeviceParamSlide
                  device={props.device}
                  sourceIndex={props.sourceIndex}
                  paramIndex={index}
                  key={index}
                />
              )}
              {conf.type === "bool" && (
                <SourceDeviceParamBool
                  device={props.device}
                  sourceIndex={props.sourceIndex}
                  paramIndex={index}
                  key={index}
                />
              )}
              {conf.type === "enum" && (
                <SourceDeviceParamEnum
                  device={props.device}
                  sourceIndex={props.sourceIndex}
                  paramIndex={index}
                  key={index}
                />
              )}
            </>
          )
        );
      })}
    </VStack>
  );
};

export const JaiDeviceControl = observer((props: { device: DeviceInfo }) => {
  const autoExposureRunning =
    jaiStore.jaiCameraParams[props.device.name] &&
    jaiStore.jaiCameraParams[props.device.name][0] &&
    jaiStore.jaiCameraParams[props.device.name][1] &&
    jaiStore.jaiCameraParams[props.device.name][0]["ExposureAuto"] &&
    jaiStore.jaiCameraParams[props.device.name][1]["ExposureAuto"] &&
    jaiStore.jaiCameraParams[props.device.name][0]["ExposureAuto"].value ===
      "2" &&
    jaiStore.jaiCameraParams[props.device.name][1]["ExposureAuto"].value ===
      "2";

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

        {autoExposureRunning && (
          <IconButton
            icon={<SunHorizon />}
            onClick={() =>
              jaiStore.fetchJaiCameraHoldAutoExposure(props.device.name, true)
            }
            aria-label="Fix Auto Exposure"
            colorScheme="cyan"
          />
        )}

        {!autoExposureRunning && (
          <IconButton
            icon={<Sun />}
            onClick={() =>
              jaiStore.fetchJaiCameraHoldAutoExposure(props.device.name, false)
            }
            aria-label="Release AutoExposure"
            colorScheme="orange"
          />
        )}
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
});

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
  useEffect(() => {
    jaiStore.fetchSubscribeJaiStream();

    return () => {
      jaiStore.fetchUnsubscribeJaiStream();
    };
  }, []);

  return (
    <PageRoot title="Jai Bridge">
      <PolarizerControl />
      <JAiBridgeControl />
      <CalibrateView />
    </PageRoot>
  );
};
