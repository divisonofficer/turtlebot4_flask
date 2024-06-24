import { observer } from "mobx-react";
import { PageRoot } from "../../design/other/flexs";
import { captureStore } from "../../stores/CaptureStore";
import {
  Button,
  Flex,
  HStack,
  IconButton,
  Input,
  Menu,
  MenuButton,
  MenuGroup,
  MenuItem,
  MenuList,
  Slider,
  SliderFilledTrack,
  SliderThumb,
  SliderTrack,
  VStack,
} from "@chakra-ui/react";
import { useEffect, useRef, useState } from "react";
import { Body3, H4 } from "../../design/text/textsystem";
import { ArrowBendDoubleUpLeft } from "@phosphor-icons/react";
import {
  CHANNEL,
  PROPERTIES,
  polarizationStore,
} from "../../stores/PolarizationStore";

export const PolarizationView = observer((props: { randomTime: number }) => {
  return (
    <VStack style={{ width: "100%" }}>
      {PROPERTIES.map((property, index_sub) => (
        <HStack
          style={{
            width: "90%",
            overflowX: "auto",
          }}
        >
          {CHANNEL.map((channel, index) => (
            <VStack
              flexGrow="1"
              onClick={() => {
                polarizationStore.viewFocusProperty = `${channel}_${property}`;
              }}
            >
              <img
                src={`/polarization/view/${channel}_${property}?timestamp=${
                  new Date().getTime() + props.randomTime
                }`}
                style={{ minWidth: "30rem", flexGrow: "1", height: "auto" }}
                alt={property}
              />
              <Body3>
                {channel}_{property}
              </Body3>
            </VStack>
          ))}{" "}
        </HStack>
      ))}
    </VStack>
  );
});

export const ConfigUpdateSlider = observer(
  (props: {
    title: string;
    value: number;
    range: [number, number];
    step: number;
    onChange: (value: number, onSuccess: () => void) => void;
  }) => {
    const [valueDisplay, setValueDisplay] = useState(props.value);

    useEffect(() => {
      setValueDisplay(props.value);
    }, [props.value]);

    return (
      <VStack>
        <H4>{props.title}</H4>
        <Slider
          min={props.range[0]}
          max={props.range[1]}
          size={"md"}
          style={{
            width: "100%",

            height: "1rem",
          }}
          step={props.step}
          value={props.value}
          onChange={(value) => {
            props.onChange(value, () => {
              setValueDisplay(value);
            });
          }}
        >
          <SliderTrack>
            <SliderFilledTrack />
          </SliderTrack>
          <SliderThumb />
        </Slider>
        <Input
          type="number"
          value={valueDisplay}
          onChange={(e) => setValueDisplay(parseFloat(e.target.value))}
          onKeyDownCapture={(e) => {
            if (e.key === "Enter") {
              props.onChange(valueDisplay, () => {});
            }
          }}
        />
      </VStack>
    );
  }
);

export const PolarizationLinearCalibration = observer(() => {
  useEffect(() => {
    captureStore.fetchGetCaptureSpaceList();
  }, []);

  const PolarizationAngleConfig = () => {
    return (
      <HStack>
        <ConfigUpdateSlider
          title="Linear Polarizer Angle"
          value={polarizationStore.mmLinearAngle}
          range={[-45, 45]}
          step={0.0001}
          onChange={(value, onSuccess) => {
            polarizationStore.fetchComputePolarization({
              angle: value,
              doOnSuccess: () => {
                polarizationStore.mmLinearAngle = value;
                onSuccess();
              },
            });
          }}
        />

        <ConfigUpdateSlider
          title="QWP Polarizer Angle"
          value={polarizationStore.mmQwpAngle}
          range={[-45, 45]}
          step={0.0001}
          onChange={(value, onSuccess) => {
            polarizationStore.fetchComputePolarization({
              qwpAngle: value,
              doOnSuccess: () => {
                polarizationStore.mmQwpAngle = value;
                onSuccess();
              },
            });
          }}
        />
      </HStack>
    );
  };

  const PolarizationSceneMenu = observer(() => {
    return (
      <HStack
        style={{
          width: "100%",
          justifyContent: "flex-start",
        }}
      >
        <Menu
          placement="bottom-start"
          onOpen={() => {
            captureStore.fetchGetCaptureSpaceList();
          }}
        >
          <MenuButton as={Button} size="sm">
            {polarizationStore.space?.spaceName || "Space"}
          </MenuButton>
          <MenuList maxHeight={"20rem"} overflowY="scroll">
            {captureStore.captureSpaceList.map((space, index) => (
              <MenuItem
                key={index}
                onClick={() => (polarizationStore.space = space)}
                fontSize={"small"}
              >
                {space.spaceName} / {space.captures.length}
              </MenuItem>
            ))}
          </MenuList>
        </Menu>

        {polarizationStore.space && (
          <Menu placement="bottom-start">
            <MenuButton as={Button} size="sm">
              {polarizationStore.capture?.captureId || "capture"}
            </MenuButton>
            <MenuList maxHeight={"20rem"} overflowY="scroll">
              {polarizationStore.space.captures.map((capture, index) => (
                <MenuItem
                  key={index}
                  onClick={() => (polarizationStore.capture = capture)}
                  fontSize="small"
                >
                  {capture.captureId} / {capture.scenes.length}
                </MenuItem>
              ))}
            </MenuList>
          </Menu>
        )}
        {polarizationStore.capture && (
          <Menu>
            <MenuButton as={Button} size="sm">
              {polarizationStore.scene
                ? polarizationStore.scene.images
                    .filter((image) => image.includes("left_channel_0"))
                    .map((image) => image.split(".png")[0].split("_").pop())
                    .join(", ")
                : "scene"}
            </MenuButton>
            <MenuList overflowY="scroll" maxHeight={"20rem"}>
              {polarizationStore.capture.scenes.map((scene, index) => (
                <MenuItem
                  fontSize={"small"}
                  key={index}
                  onClick={() =>
                    polarizationStore.fetchComputePolarization({
                      sceneId: scene.sceneId,
                      doOnSuccess: () => {
                        polarizationStore.scene = scene;
                      },
                    })
                  }
                >
                  {index} __
                  {scene.images
                    .filter((image) => image.includes("left_channel_0"))
                    .map((image) => image.split(".png")[0].split("_").pop())
                    .join(", ")}
                </MenuItem>
              ))}
            </MenuList>
          </Menu>
        )}

        {polarizationStore.scene && (
          <Menu>
            <MenuButton as={Button} size="sm">
              {polarizationStore.viewFocusProperty || "ViewProperty"}
            </MenuButton>
            <MenuList overflowY="scroll" maxHeight={"20rem"}>
              <MenuItem
                onClick={() =>
                  (polarizationStore.viewFocusProperty = `rgb_s0_positive`)
                }
                fontSize="small"
              >
                RGB_S0
              </MenuItem>
              {CHANNEL.map((channel, index) => (
                <MenuGroup title={channel}>
                  {PROPERTIES.map((property, index_sub) => (
                    <MenuItem
                      onClick={() =>
                        (polarizationStore.viewFocusProperty = `${channel}_${property}`)
                      }
                      fontSize="small"
                    >
                      {property}
                    </MenuItem>
                  ))}
                </MenuGroup>
              ))}
            </MenuList>
          </Menu>
        )}
        <IconButton
          aria-label="Refresh"
          icon={<ArrowBendDoubleUpLeft />}
          onClick={() => {
            polarizationStore.fetchComputePolarization();
          }}
        />
      </HStack>
    );
  });

  return (
    <VStack
      style={{
        height: "100%",
        width: "100%",
      }}
    >
      <PolarizationSceneMenu />
      <PolarizationAngleConfig />
      <VStack
        style={{
          width: "100%",
          overflowY: "auto",
        }}
      >
        {polarizationStore.viewFocusProperty && <PolarizationDeepView />}

        <PolarizationView randomTime={polarizationStore.randomTime} />
      </VStack>
    </VStack>
  );
});

export const PolarizationDeepView = observer(() => {
  const imageRef = useRef<HTMLImageElement>(null);

  return (
    <Flex
      style={{
        position: "relative",
      }}
    >
      <img
        ref={imageRef}
        src={`/polarization/view/${
          polarizationStore.viewFocusProperty
        }?timestamp=${new Date().getTime() + polarizationStore.randomTime}`}
        style={{ width: "80%", height: "auto" }}
        alt="viewFocusProperty"
        onMouseMove={(e) => {
          imageRef.current &&
            polarizationStore.fetchColorDeepView(
              e.nativeEvent.offsetY / imageRef.current.clientHeight,
              e.nativeEvent.offsetX / imageRef.current.clientWidth
            );
        }}
      />
      {polarizationStore.colorDeepView && (
        <VStack
          style={{
            position: "absolute",
            top: polarizationStore.colorDeepView.y * 100 + "%",
            left: polarizationStore.colorDeepView.x * 100 + "%",
            background: "#AAFFFFFF",
          }}
        >
          <Body3>
            {polarizationStore.colorDeepView.x.toFixed(4)} /{" "}
            {polarizationStore.colorDeepView.y.toFixed(4)}
          </Body3>
          <HStack>
            {polarizationStore.colorDeepView.colorList &&
              Object.entries(polarizationStore.colorDeepView.colorList).map(
                (entry, index) => (
                  <Body3 key={index}>
                    {entry[0]}:{entry[1].toFixed(3)}
                  </Body3>
                )
              )}
          </HStack>
        </VStack>
      )}
    </Flex>
  );
});

export const PolarizationCalibrationPage = observer(() => {
  return (
    <PageRoot title="Polarization">
      <PolarizationLinearCalibration />
    </PageRoot>
  );
});
