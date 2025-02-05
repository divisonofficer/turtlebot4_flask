import {
  HStack,
  Switch,
  VStack,
  CircularProgress,
  Flex,
  Slider,
  SliderTrack,
  SliderFilledTrack,
  SliderThumb,
} from "@chakra-ui/react";
import { PageRoot } from "../../design/other/flexs";

import { H3, H4 } from "../../design/text/textsystem";
import { observer } from "mobx-react";

import { useEffect } from "react";
import { InfoCard, InfoCardBtn } from "../../design/other/infocard";
import { jaiHDRStore, JaiHDRLog } from "../../stores/JaiHDRStore";
import { Camera } from "@phosphor-icons/react/dist/ssr";
import { Log, Steps } from "@phosphor-icons/react";
import { Color } from "../../design/color";

const ErrorCard = ({ error }: { error: JaiHDRLog.Error }) => {
  if (error.type === "error_process_stream_expire") {
    return (
      <InfoCard
        title={"Expired"}
        value={
          error.data.device +
          "/" +
          error.data.stream +
          ":" +
          error.data.expired_time
        }
        color={"yellow"}
      />
    );
  }

  if (error.type === "error_retrieve_buffer") {
    return (
      <InfoCard
        title={"Retrieve fail"}
        value={
          error.data.lResult === "OK" ? error.data.aResult : error.data.lResult
        }
        color={"yellow"}
      />
    );
  }

  if (error.type === "error_crash") {
    return <InfoCard title={"Crash"} value={error.data.cause} color={"red"} />;
  }

  return (
    <InfoCard
      title={error.type}
      value={JSON.stringify(error.data)}
      color={"gray"}
    />
  );
};

const JaiOptionSwitch = ({
  option_id,
  option_name,
  checked,
}: {
  option_id: string;
  option_name: string;
  checked: boolean | undefined;
}) => {
  return (
    <HStack>
      <H3> {option_name}</H3>
      <Switch
        isChecked={checked}
        onChange={(e) => {
          jaiHDRStore.fetchUpdateConfig(option_id, e.target.checked);
        }}
      />
    </HStack>
  );
};

const HDRProgressView = observer(() => {
  const {
    progress_root: pr,
    progress_sub: ps,
    hdr_error_msgs,
  } = jaiHDRStore.hdr_log || {
    progress_root: {
      idx: 0,
      status: "ready",
      task: "rotate",
    },
    progress_sub: {
      idx: 0,
      type: "hdr",
    },
    hdr_error_msgs: [],
  };

  return (
    <Flex width="100%" justifyContent="space-between" wrap="wrap">
      <VStack width="20rem">
        {[
          {
            range: [15, 180],
            step: 1,
            name: "rotate_angle",
            value: jaiHDRStore.hdr_config.rotate_angle,
          },
          {
            range: [1, 20],
            step: 1,
            name: "capture_cnt",
            value: jaiHDRStore.hdr_config.capture_cnt,
          },
          {
            range: [1, 10],
            step: 1,
            name: "side_move_cnt",
            value: jaiHDRStore.hdr_config.side_move_cnt,
          },
          {
            range: [-0.2, 0.2],
            step: 0.01,
            name: "side_move_distance",
            value: jaiHDRStore.hdr_config.side_move_distance,
          },
        ].map((param) => (
          <>
            <Slider
              min={param.range[0]}
              max={param.range[1]}
              size={"md"}
              style={{
                width: "100%",

                height: "1rem",
              }}
              step={param.step}
              value={param.value}
              onChange={(value) => {
                jaiHDRStore.fetchUpdateConfig(param.name, value);
              }}
            >
              <SliderTrack>
                <SliderFilledTrack />
              </SliderTrack>
              <SliderThumb />
            </Slider>
            <HStack justifyContent="space-between" width="100%">
              <H4>{param.name}</H4>
              <H4>{param.value}</H4>
            </HStack>
          </>
        ))}
        <JaiOptionSwitch
          option_id="lidar"
          option_name="Enable LiDAR acquire"
          checked={jaiHDRStore.hdr_config.lidar}
        />
      </VStack>

      {jaiHDRStore.hdr_log?.progress_root.status !== "running" && (
        <InfoCardBtn
          title="Capture"
          Icon={Camera}
          onClick={() => jaiHDRStore.triggerHDR()}
        />
      )}
      {jaiHDRStore.hdr_log?.progress_root.status === "running" && (
        <InfoCardBtn title={pr.task} Icon={Steps} color={Color.Cyan} />
      )}
      <InfoCard
        title={pr.status}
        value={pr.idx}
        progressBar={
          <CircularProgress
            value={
              (pr.idx /
                (jaiHDRStore.hdr_config.capture_cnt *
                  jaiHDRStore.hdr_config.side_move_cnt)) *
              100
            }
          />
        }
      />

      {(pr.task === "hdr" || pr.task === "ambient") && (
        <InfoCard
          title={"Exposure"}
          value={pr.task}
          progressBar={<CircularProgress value={(ps.idx / 4) * 100} />}
        />
      )}
      {hdr_error_msgs.length < 1 && <InfoCard title="Error" value="None" />}
      {hdr_error_msgs.length > 0 &&
        hdr_error_msgs.map((error) => <ErrorCard error={error} />)}
    </Flex>
  );
});

export const HDRPage = observer(() => {
  useEffect(() => {
    //jaiStore.fetchGetStereoNodeStatus();
  }, []);

  return (
    <PageRoot title="HDR">
      <HDRProgressView />
    </PageRoot>
  );
});
