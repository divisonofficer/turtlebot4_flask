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

import { Body3, H3, H4 } from "../../design/text/textsystem";
import { Observer, observer } from "mobx-react";

import { useEffect } from "react";
import { ChevronDownIcon } from "@chakra-ui/icons";
import { InfoCard } from "../../design/other/infocard";
import { jaiHDRStore } from "../../stores/JaiHDRStore";
import { Btn } from "../../design/button/button";

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
      <Switch isChecked={checked} onChange={(e) => {}} />
    </HStack>
  );
};

export const HDRPage = observer(() => {
  useEffect(() => {
    //jaiStore.fetchGetStereoNodeStatus();
  }, []);

  return (
    <PageRoot title="Depth">
      <Body3>{jaiHDRStore.hdr_log?.progress_root.status}</Body3>
      {jaiHDRStore.hdr_log?.progress_root.status !== "running" && (
        <Btn onClick={() => jaiHDRStore.triggerHDR()}>Trigger HDR</Btn>
      )}
      <HStack>
        <Body3>{jaiHDRStore.hdr_log?.progress_sub.idx}</Body3>
        <Body3>{jaiHDRStore.hdr_log?.progress_sub.type}</Body3>
        <Body3>{jaiHDRStore.hdr_log?.progress_root.task}</Body3>
      </HStack>
    </PageRoot>
  );
});
