import { observer } from "mobx-react";
import { PageRoot } from "../../design/other/flexs";
import { captureStore } from "../../stores/CaptureStore";
import {
  Flex,
  HStack,
  Input,
  Progress,
  Slider,
  SliderFilledTrack,
  SliderThumb,
  SliderTrack,
  VStack,
} from "@chakra-ui/react";
import { useEffect, useState } from "react";
import { InfoCardBtn } from "../../design/other/infocard";
import { Btn } from "../../design/button/button";
import {
  CaptureAppCapture,
  CaptureAppScene,
  CaptureAppSpace,
} from "../../public/proto/capture";
import { Body3, H4 } from "../../design/text/textsystem";
import { httpGet } from "../../connect/http/request";

export const PolarizationLinearCalibration = observer(() => {
  const [angle, setAngle] = useState(0);
  const [qwpAngle, setQwpAngle] = useState(0);
  const [capture, setCapture] = useState<CaptureAppCapture | undefined>(
    undefined
  );
  const [space, setSpace] = useState<CaptureAppSpace | undefined>(undefined);
  const [scene, setScene] = useState<CaptureAppScene | undefined>(undefined);

  useEffect(() => {
    captureStore.fetchGetCaptureSpaceList();
  }, []);

  return (
    <VStack>
      <HStack
        style={{
          width: "100%",
          justifyContent: "flex-start",
        }}
      >
        <VStack
          style={{
            width: "33%",
            height: "15rem",
            overflowY: "scroll",
            alignItems: "flex-start",
          }}
        >
          {captureStore.captureSpaceList.map((space, index) => (
            <Btn key={index} onClick={() => setSpace(space)} size="sm">
              {space.spaceName} / {space.captures.length}
            </Btn>
          ))}
        </VStack>

        {space && (
          <VStack
            style={{
              width: "33%",
              height: "15rem",
              overflowY: "scroll",
              alignItems: "flex-start",
            }}
          >
            {space.captures.map((capture, index) => (
              <Btn key={index} onClick={() => setCapture(capture)} size="sm">
                {capture.captureId} / {capture.scenes.length}
              </Btn>
            ))}
          </VStack>
        )}
        {capture && (
          <VStack
            style={{
              width: "33%",
              height: "15rem",
              overflowY: "scroll",
              alignItems: "flex-start",
            }}
          >
            {capture.scenes.map((scene, index) => (
              <Btn
                key={index}
                size="sm"
                onClick={() =>
                  httpGet(`
                /capture/polarization/linear/${space?.spaceId}/${capture?.captureId}/${scene.sceneId}/${angle}/${qwpAngle}
              `)
                    .onSuccess((data) => {
                      setScene(scene);
                    })
                    .fetch()
                }
              >
                {index} __
                {scene.images
                  .filter((image) => image.includes("left_channel_0"))
                  .map((image) => image.split(".png")[0].split("_").pop())
                  .join(", ")}
              </Btn>
            ))}
          </VStack>
        )}
      </HStack>

      <HStack>
        <VStack>
          <H4>Linear Polarizer Angle Adjust</H4>
          <Slider
            min={0}
            max={180}
            size={"md"}
            style={{
              width: "100%",

              height: "1rem",
            }}
            step={0.0001}
            value={angle}
            onChange={(value) => {
              httpGet(`
            /capture/polarization/linear/${space?.spaceId}/${capture?.captureId}/${scene?.sceneId}/${value}/${qwpAngle}
          `)
                .onSuccess((data) => {
                  setAngle(value);
                })
                .fetch();
            }}
          >
            <SliderTrack>
              <SliderFilledTrack />
            </SliderTrack>
            <SliderThumb />
          </Slider>
          <Input
            type="number"
            value={angle}
            onChange={(e) => setAngle(parseFloat(e.target.value))}
          />
        </VStack>
        <VStack>
          <H4>QWP Polarizer Angle Adjust</H4>
          <Slider
            min={0}
            max={180}
            size={"md"}
            style={{
              width: "100%",

              height: "1rem",
            }}
            step={0.0001}
            value={qwpAngle}
            onChange={(value) => {
              httpGet(`
            /capture/polarization/linear/${space?.spaceId}/${capture?.captureId}/${scene?.sceneId}/${angle}/${value}
          `)
                .onSuccess((data) => {
                  setQwpAngle(value);
                })
                .fetch();
            }}
          >
            <SliderTrack>
              <SliderFilledTrack />
            </SliderTrack>
            <SliderThumb />
          </Slider>
          <Input
            type="number"
            value={qwpAngle}
            onChange={(e) => setQwpAngle(parseFloat(e.target.value))}
          />
        </VStack>
      </HStack>

      {[
        [
          "s0_rgb_positive",
          "s1_rgb_positive",
          "s2_rgb_positive",
          "s3_rgb_positive",
        ],
        [
          "s0_rgb_negative",
          "s1_rgb_negative",
          "s2_rgb_negative",
          "s3_rgb_negative",
        ],
        [
          "s0_nir_positive",
          "s1_nir_positive",
          "s2_nir_positive",
          "s3_nir_positive",
        ],
        [
          "s0_nir_negative",
          "s1_nir_negative",
          "s2_nir_negative",
          "s3_nir_negative",
        ],
        ["r_aolp", "r_dolp", "g_aolp", "g_dolp", "b_aolp", "b_dolp"],
        ["nir_aolp", "nir_dolp"],
      ].map((property_group, index) => (
        <Flex wrap="wrap" width="100%">
          {property_group.map((property, index_sub) => (
            <VStack>
              <img
                src={`/capture/polarization/linear/${space?.spaceId}/${capture?.captureId}/${scene?.sceneId}/${angle}/${qwpAngle}/${property}`}
                style={{ width: "20rem", height: "12rem" }}
              />
              <Body3>{property}</Body3>
            </VStack>
          ))}{" "}
        </Flex>
      ))}
    </VStack>
  );
});

export const PolarizationCalibrationPage = observer(() => {
  return (
    <PageRoot title="Polarization">
      <PolarizationLinearCalibration />
    </PageRoot>
  );
});
