import {
  Button,
  Checkbox,
  CircularProgress,
  Flex,
  HStack,
  Switch,
  VStack,
} from "@chakra-ui/react";
import { observer } from "mobx-react";
import { captureStore } from "../../stores/CaptureStore";
import { Body2, Body3 } from "../../design/text/textsystem";
import { LazyLoadImage } from "react-lazy-load-image-component";
import { CaptureAppCapture, CaptureAppScene } from "../../public/proto/capture";
import { useState } from "react";
import { NavLink } from "react-router-dom";

export const CaptureImageStorage = observer(() => {
  const [open, setOpen] = useState(false);
  return (
    <>
      <HStack>
        <Switch
          defaultChecked={open}
          onChange={(e) => {
            setOpen(e.target.checked);
          }}
        >
          Storage
        </Switch>
        {captureStore.space_id && (
          <>
            <Button
              onClick={() => {
                captureStore.fetchPostCompressSpace();
              }}
            >
              Compress
            </Button>
            <NavLink to={`/capture/result/${captureStore.space_id}/gzipped`}>
              <Button>Download</Button>
            </NavLink>
          </>
        )}
      </HStack>
      {open && <CaptureImageStorageView />}
    </>
  );
});

export const CaptureImageStorageView = observer(() => {
  return (
    <VStack>
      <Flex wrap="wrap" style={{ width: "100%" }}>
        {Object.keys(captureStore.capture_view_image_filenames).map(
          (filename, index) => (
            <HStack key={filename} paddingRight={"1rem"}>
              <Checkbox
                defaultChecked={
                  captureStore.capture_view_image_filenames[filename]
                }
                onChange={(e) => {
                  captureStore.capture_view_image_filenames[filename] =
                    e.target.checked;
                }}
              />
              <Body3>{filename}</Body3>
            </HStack>
          )
        )}
      </Flex>

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
    </VStack>
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
      <Flex
        style={{
          overflowX: "auto",
          width: "100%",
        }}
        wrap={"wrap"}
      >
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
        )}{" "}
      </Flex>
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
            captureStore.capture_view_image_filenames[
              url.split("/").pop()!
            ] && (
              <VStack
                style={{
                  alignItems: "flex-start",
                }}
              >
                <Body3>
                  {url.split("/").pop()?.split(".jpg")[0].split(".png")[0]}
                </Body3>
                <LazyLoadImage
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
                    margin: "0.5rem",
                  }}
                />
              </VStack>
            )
          );
        })}
      </Flex>
    </VStack>
  );
};
