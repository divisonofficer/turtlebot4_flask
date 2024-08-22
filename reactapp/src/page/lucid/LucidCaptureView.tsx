import { observer } from "mobx-react-lite";
import { PageRoot } from "../../design/other/flexs";
import { lucidStore } from "../../stores/LucidStore";
import { Btn } from "../../design/button/button";
import { Body3 } from "../../design/text/textsystem";
import { Switch, VStack } from "@chakra-ui/react";

export const LucidCaptureView = observer(() => {
  return (
    <PageRoot title="Lucid">
      {lucidStore.storageEnabled ? (
        <Btn onClick={lucidStore.fetchDisableStorage}>Disable Storage</Btn>
      ) : (
        <Btn onClick={lucidStore.fetchEnableStorage}>Enable Storage</Btn>
      )}
      {lucidStore.lucidStatus.hasOwnProperty("single_storage_mode") && (
        <Switch
          isChecked={lucidStore.lucidStatus["single_storage_mode"]}
          onChange={(e) => {
            lucidStore.fetchUpdateStatusAttributes({
              single_storage_mode: e.target.checked,
            });
          }}
        />
      )}

      <VStack
        style={{
          width: "100%",
          height: "100vh",
        }}
      >
        {lucidStore.lucidStatusJson.split(",").map((v, i) => (
          <Body3
            key={i}
            style={{
              width: "100%",
            }}
          >
            {"," + v}
          </Body3>
        ))}
      </VStack>
    </PageRoot>
  );
});
