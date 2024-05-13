import { CircularProgress, Flex, HStack, VStack } from "@chakra-ui/react";
import { observer } from "mobx-react";
import { captureStore } from "../../stores/CaptureStore";
import { Body2, Body3 } from "../../design/text/textsystem";

import { CaptureAppCapture, CaptureAppScene } from "../../public/proto/capture";

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

export const CaptureItem = ({ capture }: { capture: CaptureAppCapture }) => {
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
      <Body2>{capture.captureId}</Body2>
      <HStack>
        {capture.scenes.map((scene, index) => (
          <CaptureImageItem scene={scene} key={index} />
        ))}
        {captureStore.capture_pendings_id.includes(capture.captureId) && (
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

export const CaptureImageItem = ({ scene }: { scene: CaptureAppScene }) => {
  return (
    <VStack
      style={{
        borderRadius: "0.5rem",
        overflow: "hidden",
        background: "#ffffff",
        padding: "0.5rem",
      }}
      onClick={() => {
        captureStore.map_focused_capture_id = scene.captureId;
        captureStore.map_focused_scene_id = scene.sceneId;
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

      <Flex wrap="wrap">
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
                    : `/capture/result/${scene.spaceId}/${scene.captureId}/${scene.sceneId}/${url}/thumb`
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
      </Flex>
    </VStack>
  );
};
