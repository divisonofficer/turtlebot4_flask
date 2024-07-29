import { HStack, Switch, VStack } from "@chakra-ui/react";
import { PageRoot } from "../../design/other/flexs";
import { VideoStream } from "../../design/other/video";
import { Body3, H3, H4 } from "../../design/text/textsystem";
import { observer } from "mobx-react";
import { jaiStore } from "../../stores/JaiStore";
import { useEffect } from "react";

const DepthVideoStream = ({ url }: { url: string }) => {
  return (
    <VStack width="100%">
      <HStack width="100%" justifyContent="space-evenly">
        <H4>Left</H4>
        <H4>Right</H4>
        <H4>Disparity</H4>
      </HStack>
      <VideoStream url={url} style={{ width: "100%", height: "auto" }} />

      <HStack>
        <H4>0</H4>
        <img
          src="https://kr.mathworks.com/help/matlab/ref/colormap_jet.png"
          alt="jet"
        />
        <H4>255</H4>
      </HStack>
    </VStack>
  );
};

const StorageEnableButton = observer(() => {
  return (
    <VStack>
      <HStack>
        <H3>Storage</H3>
        <Switch
          isChecked={jaiStore.depth_storage_id ? true : false}
          onChange={(e) => {
            if (e.target.checked) {
              jaiStore.fetchEnableStereoStorage();
            } else {
              jaiStore.fetchDisableStereoStorage();
            }
          }}
        />
        <Body3>{jaiStore.depth_storage_id}</Body3>
      </HStack>
    </VStack>
  );
});

export const DepthViewPage = () => {
  useEffect(() => {
    jaiStore.fetchGetStereoNodeStatus();
  }, []);

  return (
    <PageRoot title="Depth">
      <StorageEnableButton />
      <DepthVideoStream url="/jai/stereo/stream/stream_disparity_viz" />
      <DepthVideoStream url="/jai/stereo/stream/stream_disparity_nir" />
    </PageRoot>
  );
};
