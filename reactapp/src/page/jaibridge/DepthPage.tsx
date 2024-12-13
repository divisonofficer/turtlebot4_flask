import {
  HStack,
  Switch,
  VStack,
  Menu,
  MenuButton,
  MenuItem,
  MenuList,
  Button,
} from "@chakra-ui/react";
import { PageRoot } from "../../design/other/flexs";
import { VideoStream } from "../../design/other/video";
import { Body3, H3, H4 } from "../../design/text/textsystem";
import { observer } from "mobx-react";
import { JaiStereoQueueStatus, jaiStore } from "../../stores/JaiStore";
import { useEffect } from "react";
import { ChevronDownIcon } from "@chakra-ui/icons";
import { InfoCard } from "../../design/other/infocard";

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
          jaiStore.fetchUpdateStereoOption(option_id, e.target.checked);
        }}
      />
    </HStack>
  );
};

const StorageEnableButton = observer(() => {
  return (
    <VStack>
      <HStack>
        <H3>Storage</H3>
        <Switch
          isChecked={jaiStore.stereo_status?.storage_id ? true : false}
          onChange={(e) => {
            if (e.target.checked) {
              jaiStore.fetchEnableStereoStorage();
            } else {
              jaiStore.fetchDisableStereoStorage();
            }
          }}
        />
        <Body3>{jaiStore.stereo_status?.storage_id}</Body3>
        <VStack
          style={{
            width: "20rem",
          }}
        >
          <JaiOptionSwitch
            option_id="single_frame_mode"
            option_name="Single Capture"
            checked={jaiStore.stereo_status?.option_status.single_frame_mode}
          />
          <JaiOptionSwitch
            option_id="reset_on_lidar_error"
            option_name="Reset on Lidar Error"
            checked={jaiStore.stereo_status?.option_status.reset_on_lidar_error}
          />
        </VStack>
        <VStack
          style={{
            width: "20rem",
          }}
        >
          <JaiOptionSwitch
            option_id="lidar_collect"
            option_name="LiDAR"
            checked={jaiStore.stereo_status?.option_status.lidar_collect}
          />
          <JaiOptionSwitch
            option_id="oakd_collect"
            option_name="OAK-Depth"
            checked={jaiStore.stereo_status?.option_status.oakd_collect}
          />
          {/**
           * <JaiOptionSwitch
           *  option_id="oakd_nir_collect"
           */}
        </VStack>
      </HStack>
    </VStack>
  );
});

export const QueueStatusView = ({ queue }: { queue: JaiStereoQueueStatus }) => {
  return (
    <VStack>
      <Body3>{queue.merge_interval?.toFixed(4)}</Body3>
      <Body3>{queue.diff?.toFixed(4)}</Body3>
      <Body3>Left Queue Size</Body3>
      <Body3>{queue.left_count}</Body3>
      <Body3>Left Last Delay</Body3>
      <Body3>
        {queue.left_last_delay?.toFixed(4)} _ {queue.left_last_time}
      </Body3>
      <Body3>Right Queue Size</Body3>
      <Body3>{queue.right_count}</Body3>
      <Body3>Right Last Delay</Body3>
      <Body3>
        {queue.right_last_delay?.toFixed(4)} _ {queue.right_last_time}
      </Body3>
    </VStack>
  );
};
export const DepthStatusView = observer(() => {
  const queue_viz = jaiStore.stereo_status?.queue_status.viz;
  const queue_nir = jaiStore.stereo_status?.queue_status.nir;
  const queue_merged = jaiStore.stereo_status?.queue_status.merged;

  return (
    <HStack>
      <VStack>
        <H4>Viz</H4>
        {queue_viz && <QueueStatusView queue={queue_viz} />}
      </VStack>
      <VStack>
        <H4>Nir</H4>
        {queue_nir && <QueueStatusView queue={queue_nir} />}
      </VStack>
      <VStack>
        <H4>Merged</H4>
        {queue_merged && <QueueStatusView queue={queue_merged} />}
      </VStack>

      {jaiStore.stereo_status && (
        <HStack>
          <InfoCard
            title="Stored Frame Count"
            value={jaiStore.stereo_status.storage_status.stored_frame_cnt}
          />
          <InfoCard
            title="Framerate"
            value={jaiStore.stereo_status.storage_status.frame_rate.toFixed(4)}
          />
          <InfoCard
            title="Queue Len"
            value={jaiStore.stereo_status.storage_status.queue_length}
          />
          <InfoCard
            title="Store Time"
            value={jaiStore.stereo_status.storage_status.item_store_time.toFixed(
              4
            )}
          />
        </HStack>
      )}
    </HStack>
  );
});

const LoadCalibrationButton = observer(() => {
  useEffect(() => {
    jaiStore.fetchGetCalibrationList();
  }, []);

  return (
    <Menu placement="bottom-end">
      <MenuButton as={Button} rightIcon={<ChevronDownIcon />} fontSize="md">
        Load Calibration
      </MenuButton>
      <MenuList>
        {jaiStore.calibrationList.map((meta, index) => {
          return (
            <MenuItem
              key={index}
              fontSize="small"
              onClick={() => {
                jaiStore.fetchDepthSetCurrentCalibration(meta.id);
              }}
            >
              {meta.id} : {meta.month}/{meta.day}/{meta.hour}
            </MenuItem>
          );
        })}
      </MenuList>
    </Menu>
  );
});

export const DepthViewPage = () => {
  useEffect(() => {
    //jaiStore.fetchGetStereoNodeStatus();
  }, []);

  return (
    <PageRoot title="Depth">
      <LoadCalibrationButton />
      <StorageEnableButton />
      <DepthStatusView />
      <DepthVideoStream url="/jai/stereo/stream/stream_disparity_viz" />
    </PageRoot>
  );
};
