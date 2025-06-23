import { observer } from "mobx-react";
import { PageRoot } from "../design/other/flexs";
import { useEffect } from "react";
import { jaiSocket } from "../connect/socket/subscribe";
import {
  Button,
  HStack,
  Slider,
  SliderFilledTrack,
  SliderThumb,
  SliderTrack,
} from "@chakra-ui/react";
import { jaiStore } from "../stores/JaiStore";
import { H4 } from "../design/text/textsystem";

const ImageSocketStream = (props: { topic: string; tag?: string }) => {
  useEffect(() => {
    jaiSocket.subscribe(props.topic, (data) => {
      console.log("Received image data:", data.dtype, props.topic);
      const database64 = data.data;
      const img = new Image();
      img.src = `data:image/jpeg;base64,${database64}`;
      img.onload = () => {
        const canvas = document.getElementById(
          props.topic.replace(/\//g, "_")
        ) as HTMLCanvasElement;
        const ctx = canvas.getContext("2d");
        if (ctx) {
          canvas.width = img.width;
          canvas.height = img.height;
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          ctx.drawImage(img, 0, 0);
        }
      };
    });
    return () => {
      jaiSocket.unsubscribe(props.topic);
    };
  }, []);
  return (
    <div style={{ position: "relative", width: "100%", height: "auto" }}>
      <canvas
        style={{ width: "100%", height: "auto" }}
        id={props.topic.replace(/\//g, "_")}
        width="640"
        height="480"
      ></canvas>
      {props.tag && (
        <div
          style={{
            position: "absolute",
            bottom: "10px",
            left: "10px",
            backgroundColor: "rgba(0, 0, 0, 0.5)",
            color: "white",
            padding: "3px 8px",
            borderRadius: "4px",
            fontSize: "0.5em",
          }}
        >
          {props.tag}
        </div>
      )}
    </div>
  );
};

const DemoModeName = (props: { mode: "RGB" | "NIR" | "IF" | "FF" }) => {
  switch (props.mode) {
    case "RGB":
      return "RGB";
    case "NIR":
      return "NIR";
    case "IF":
      return "Image Fusion";
    case "FF":
      return "Feature Fusion";
    default:
      return "Unknown Mode";
  }
};

export const DemoPage = observer(() => {
  return (
    <PageRoot title="Demo">
      <HStack justify="flex-start" width="100%">
        {["RGB", "NIR", "IF", "FF"].map((tag) => (
          <Button
            size="lg"
            key={tag}
            colorScheme={jaiStore.demo_status.mode === tag ? "blue" : "gray"}
            onClick={() => {
              jaiStore.fetchUpdateDemoConfig(
                "mode",
                tag as "RGB" | "NIR" | "IF" | "FF"
              );
            }}
          >
            {DemoModeName({ mode: tag as "RGB" | "NIR" | "IF" | "FF" })}
          </Button>
        ))}

        <HStack width="20rem">
          <H4>Max Disparity</H4>
          <Slider
            min={0}
            max={200}
            size={"md"}
            style={{
              width: "20%",
              height: "5rem",
            }}
            step={10}
            value={jaiStore.demo_status.max_disp}
            onChange={(value) => {
              jaiStore.fetchUpdateDemoConfig("max_disp", value);
            }}
          >
            <SliderTrack>
              <SliderFilledTrack />
            </SliderTrack>
            <SliderThumb />
          </Slider>
          <H4>{jaiStore.demo_status.max_disp}</H4>
        </HStack>
      </HStack>
      <HStack>
        <ImageSocketStream
          topic="demo/img/stereo_rgb_left_rectified"
          tag="RGB Left"
        />
        <ImageSocketStream
          topic="demo/img/stereo_rgb_right_rectified"
          tag="RGB Right"
        />
        <ImageSocketStream
          topic="demo/img/stereo_nir_left_rectified"
          tag="NIR Left"
        />
        <ImageSocketStream
          topic="demo/img/stereo_nir_right_rectified"
          tag="NIR Right"
        />
        {jaiStore.demo_status.mode === "IF" && (
          <>
            <ImageSocketStream
              topic="demo/img/stereo_fusion_left"
              tag="Image Fusion Left"
            />
            <ImageSocketStream
              topic="demo/img/stereo_fusion_right"
              tag="Image Fusion Right"
            />
          </>
        )}
      </HStack>

      <HStack width="100%">
        {jaiStore.demo_status.mode !== "FF" && (
          <ImageSocketStream
            topic="demo/img/yolo_detection"
            tag="Object Detection"
          />
        )}
        {jaiStore.demo_status.mode === "FF" && (
          <ImageSocketStream
            topic="demo/img/stereo_attention"
            tag="Feature Fusion Attention"
          />
        )}
        <ImageSocketStream topic="demo/img/stereo_depth" tag="Stereo Depth" />
      </HStack>
    </PageRoot>
  );
});
