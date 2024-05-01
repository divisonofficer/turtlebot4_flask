import {
  CircularProgress,
  Flex,
  HStack,
  Progress,
  VStack,
} from "@chakra-ui/react";
import { observer } from "mobx-react";
import { CaptureSingle, captureStore } from "../../stores/CaptureStore";
import { Body2, Body3 } from "../../design/text/textsystem";

import { CaptureScene } from "../../stores/CaptureStore";

export const CaptureImageStorage = observer(() => {
  return (
    <Flex
      style={{
        width: "100%",
      }}
      wrap="wrap"
      gap="2"
    >
      {captureStore.captures.map((capture, index) => {
        return <CaptureItem capture={capture} key={index} />;
      })}
    </Flex>
  );
});

export const CaptureItem = ({ capture }: { capture: CaptureSingle }) => {
  return (
    <VStack
      style={{
        borderRadius: "0.5rem",
        overflow: "hidden",
        background: "#fcfcfc",
        padding: "0.5rem",
        alignItems: "flex-start",
      }}
    >
      <Body2>{capture.capture_id}</Body2>
      <HStack>
        {capture.scenes.map((scene, index) => (
          <CaptureImageItem scene={scene} key={index} />
        ))}
        {captureStore.capture_pendings_id.includes(capture.capture_id) && (
          <Flex
            style={{
              width: "12rem",
              height: "12rem",
              alignItems: "center",
              justifyContent: "center",
            }}
          >
            <CircularProgress
              size="5rem"
              isIndeterminate
              style={{
                borderRadius: "0.5rem",
              }}
            />
          </Flex>
        )}
      </HStack>
    </VStack>
  );
};

export const CaptureImageItem = ({ scene }: { scene: CaptureScene }) => {
  return (
    <VStack
      style={{
        borderRadius: "0.5rem",
        overflow: "hidden",
        background: "#ffffff",
        padding: "0.5rem",
      }}
      onClick={() => {
        captureStore.map_focused_capture_id = scene.capture_id;
        captureStore.map_focused_scene_id = scene.scene_id;
      }}
    >
      <VStack
        style={{
          width: "100%",
          alignItems: "flex-start",
        }}
      >
        <Body3>{new Date(scene.timestamp * 1000).toLocaleString()}</Body3>
      </VStack>

      <HStack>
        {scene.images.map((url, index) => {
          return (
            <VStack
              style={{
                alignItems: "flex-start",
              }}
            >
              <Body3>
                {url.split("/").pop()?.split(".jpg")[0].split(".png")[0]}
              </Body3>
              <img
                key={index}
                src={
                  url.startsWith("/capture")
                    ? url
                    : `/capture/result/${scene.space_id}/${scene.capture_id}/${scene.scene_id}/${url}/thumb`
                }
                alt=""
                style={{
                  width: "12rem",
                  height: "12rem",
                  borderRadius: "0.5rem",
                }}
              />
            </VStack>
          );
        })}
      </HStack>
    </VStack>
  );
};
