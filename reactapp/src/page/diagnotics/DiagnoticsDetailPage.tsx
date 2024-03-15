import { observer } from "mobx-react-lite";
import { PageRoot } from "../../design/other/flexs";
import { Body1, Body3, H1, H2 } from "../../design/text/textsystem";
import { diagnoticsStore } from "../../stores/DiagnoticsStore";
import { Flex, HStack, VStack } from "@chakra-ui/react";
import { Color } from "../../design/color";

const StringValueView = ({
  name,
  value,
  idx,
}: {
  name: string;
  value: string;
  idx: number;
}) => {
  const _name = name.split("(")[0];
  const _dimention =
    name.split("(").length > 1 ? name.split("(")[1].split(")")[0] : "";
  const color = Object.values(Color)[idx % 8];
  return (
    <VStack
      style={{
        width: "12rem",
        height: "12rem",
        background: color,
        borderRadius: "1.5rem",
        padding: "1rem",
        justifyContent: "space-between",
      }}
    >
      <Body3
        style={{
          color: "white",
        }}
      >
        {_name}
      </Body3>
      <H2
        style={{
          color: "white",
        }}
      >
        {value} {_dimention}
      </H2>
    </VStack>
  );
};
const NumValueView = ({
  name,
  value,
  idx,
}: {
  name: string;
  value: number;
  idx: number;
}) => {
  const _name = name.split("(")[0];
  const _dimention =
    name.split("(").length > 1 ? name.split("(")[1].split(")")[0] : "";
  const color = Object.values(Color)[idx % 8];
  return (
    <VStack
      style={{
        width: "12rem",
        height: "12rem",
        background: color,
        borderRadius: "1.5rem",
        padding: "1rem",
        justifyContent: "space-between",
      }}
    >
      <Body3
        style={{
          color: "white",
        }}
      >
        {_name}
      </Body3>
      <H2
        style={{
          color: "white",
        }}
      >
        {value.toPrecision(4)} {_dimention}
      </H2>
    </VStack>
  );
};
export const DiagnoticsDetailPage = observer(() => {
  const msgKey = diagnoticsStore.detailMessageKey;
  return (
    <PageRoot
      title={diagnoticsStore.message[msgKey]?.name.split(":")[1] || "Detail"}
      backPage={{
        title: "Diagnotics",
        path: "/diagnostics",
      }}
    >
      <HStack
        style={{
          paddingLeft: "1rem",
          marginTop: "2rem",
        }}
      >
        {Object.entries(diagnoticsStore.message[msgKey]?.values || {}).map(
          ([key, value], idx) =>
            typeof value === "string" && !isNaN(Number(value)) ? (
              <NumValueView
                key={key}
                name={key}
                value={Number(value)}
                idx={idx}
              />
            ) : (
              <></>
            )
        )}
      </HStack>

      <HStack
        style={{
          paddingLeft: "1rem",
          marginTop: "2rem",
        }}
      >
        {Object.entries(diagnoticsStore.message[msgKey]?.values || {}).map(
          ([key, value], idx) =>
            typeof value === "string" && isNaN(Number(value)) ? (
              <StringValueView key={key} name={key} value={value} idx={idx} />
            ) : (
              <></>
            )
        )}
      </HStack>

      <Flex
        style={{
          width: "100%",
          height: "15rem",
          marginTop: "2rem",
          padding: "2rem",
        }}
      >
        <VStack
          style={{
            background: "#F7F9FB",
            borderRadius: "1rem",
            flex: 1,
            alignItems: "flex-start",
            paddingLeft: "2rem",
            paddingTop: "2rem",
          }}
        >
          <H1>Message</H1>
          <Body1>{diagnoticsStore.message[msgKey]?.message}</Body1>
        </VStack>
      </Flex>
    </PageRoot>
  );
});
