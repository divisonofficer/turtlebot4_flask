import { observer } from "mobx-react";
import { PageRoot } from "../../design/other/flexs";
import { captureStore } from "../../stores/CaptureStore";
import {
  Button,
  Flex,
  HStack,
  Input,
  Menu,
  MenuButton,
  MenuItem,
  MenuList,
  Slider,
  SliderFilledTrack,
  SliderThumb,
  SliderTrack,
  VStack,
} from "@chakra-ui/react";
import { useEffect, useState } from "react";
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

  const [viewFocusProperty, setViewFocusProperty] = useState<
    string | undefined
  >(undefined);

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
        <Menu placement="bottom-start">
          <MenuButton as={Button} size="sm">
            {space?.spaceName || "Space"}
          </MenuButton>
          <MenuList maxHeight={"20rem"} overflowY="scroll">
            {captureStore.captureSpaceList.map((space, index) => (
              <MenuItem
                key={index}
                onClick={() => setSpace(space)}
                fontSize={"small"}
              >
                {space.spaceName} / {space.captures.length}
              </MenuItem>
            ))}
          </MenuList>
        </Menu>

        {space && (
          <Menu placement="bottom-start">
            <MenuButton as={Button} size="sm">
              {capture?.captureId || "capture"}
            </MenuButton>
            <MenuList maxHeight={"20rem"} overflowY="scroll">
              {space.captures.map((capture, index) => (
                <MenuItem
                  key={index}
                  onClick={() => setCapture(capture)}
                  fontSize="small"
                >
                  {capture.captureId} / {capture.scenes.length}
                </MenuItem>
              ))}
            </MenuList>
          </Menu>
        )}
        {capture && (
          <Menu>
            <MenuButton as={Button} size="sm">
              {scene
                ? scene.images
                    .filter((image) => image.includes("left_channel_0"))
                    .map((image) => image.split(".png")[0].split("_").pop())
                    .join(", ")
                : "scene"}
            </MenuButton>
            <MenuList overflowY="scroll" maxHeight={"20rem"}>
              {capture.scenes.map((scene, index) => (
                <MenuItem
                  fontSize={"small"}
                  key={index}
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
                </MenuItem>
              ))}
            </MenuList>
          </Menu>
        )}

        {scene && (
          <Menu>
            <MenuButton as={Button} size="sm">
              {viewFocusProperty || "ViewProperty"}
            </MenuButton>
            <MenuList>
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
                ["r_aolp", "g_aolp", "b_aolp", "nir_aolp"],
                ["r_dolp", "g_dolp", "b_dolp", "nir_dolp"],
              ].map((property_group, index) =>
                property_group.map((property, index_sub) => (
                  <MenuItem
                    onClick={() => setViewFocusProperty(property)}
                    fontSize="small"
                  >
                    {property}
                  </MenuItem>
                ))
              )}
            </MenuList>
          </Menu>
        )}
      </HStack>

      {viewFocusProperty && (
        <img
          src={`/capture/polarization/view/${viewFocusProperty}?timestamp=${new Date().getTime()}`}
          style={{ width: "80%", height: "auto" }}
          alt="viewFocusProperty"
        />
      )}
      <HStack>
        <VStack>
          <H4>Linear Polarizer Angle Adjust</H4>
          <Slider
            min={-90}
            max={90}
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
            min={-90}
            max={90}
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
        ["r_aolp", "g_aolp", "b_aolp", "nir_aolp"],
        ["r_dolp", "g_dolp", "b_dolp", "nir_dolp"],
      ].map((property_group, index) => (
        <Flex wrap="wrap" width="100%">
          {property_group.map((property, index_sub) => (
            <VStack>
              <img
                src={`/capture/polarization/view/${property}?timestamp=${new Date().getTime()}`}
                style={{ width: "20rem", height: "12rem" }}
                alt={property}
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
