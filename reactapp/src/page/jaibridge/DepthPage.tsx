import { HStack, VStack } from "@chakra-ui/react";
import { PageRoot } from "../../design/other/flexs";
import { VideoStream } from "../../design/other/video";
import { H4 } from "../../design/text/textsystem";

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

export const DepthViewPage = () => {
  return (
    <PageRoot title="Depth">
      <DepthVideoStream url="/jai/stereo/stream/stream_disparity_viz" />
      <DepthVideoStream url="/jai/stereo/stream/stream_disparity_nir" />
    </PageRoot>
  );
};
