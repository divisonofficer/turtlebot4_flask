import { Flex, Switch } from "@chakra-ui/react";
import { PageRoot } from "../../design/other/flexs";
import { CaptureControl } from "./CaptureControl";
import { CaptureMap } from "./CaptureMap";
import { CaptureImageStorage } from "./CaptureImageStorage";
import { captureStore } from "../../stores/CaptureStore";
import { useState } from "react";

export const CapturePage = () => {
  const [openStorage, setOpenStorage] = useState(false);
  return (
    <PageRoot
      title="Capture"
      runOnMount={() => {
        captureStore.fetchGetCaptureSpaceList();
        captureStore.loadStatus();
      }}
    >
      <CaptureControl />

      <CaptureMap />
      <Switch
        defaultChecked={openStorage}
        onChange={(e) => setOpenStorage(e.target.checked)}
      >
        OpenStorage
      </Switch>
      {openStorage && <CaptureImageStorage />}
    </PageRoot>
  );
};
