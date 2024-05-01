import { Flex } from "@chakra-ui/react";
import { PageRoot } from "../../design/other/flexs";
import { CaptureControl } from "./CaptureControl";
import { CaptureMap } from "./CaptureMap";
import { CaptureImageStorage } from "./CaptureImageStorage";
import { captureStore } from "../../stores/CaptureStore";

export const CapturePage = () => {
  return (
    <PageRoot
      title="Capture"
      runOnMount={() => {
        captureStore.fetchGetCaptureSpaceList();
        captureStore.loadStatus();
      }}
    >
      <CaptureMap />
      <CaptureControl />
      <CaptureImageStorage />
    </PageRoot>
  );
};
