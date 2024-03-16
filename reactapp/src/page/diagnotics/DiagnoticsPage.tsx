import { observer } from "mobx-react";
import { PageRoot } from "../../design/other/flexs";
import { Body1, H4 } from "../../design/text/textsystem";
import { Diagnotic, diagnoticsStore } from "../../stores/DiagnoticsStore";
import { Grid, VStack } from "@chakra-ui/react";
import { Color } from "../../design/color";
import { useState } from "react";
import { useNavigate } from "react-router-dom";

export const DiagnosticView = observer(
  (prop: { di: Diagnotic; idx: number }) => {
    const { di } = prop;
    const navigate = useNavigate();

    const [hover, setHover] = useState(false);

    const color = Object.values(Color)[prop.idx % Object.values(Color).length];

    return (
      <VStack
        style={{
          background: color + (hover ? "80" : "FF"),
          padding: "1rem",
          margin: "10px",
          borderRadius: "1rem",
          width: "20rem",
          height: "10rem",
          alignItems: "flex-start",
          justifyContent: "space-between",
          transition: "0.3s",
        }}
        onClick={() => {
          diagnoticsStore.putDetailMessage(di.name);
          navigate("/diagnostics/detail");
        }}
        onMouseEnter={() => setHover(true)}
        onMouseLeave={() => setHover(false)}
      >
        <Body1
          style={{
            color: "white",
          }}
        >
          {di.name.split(":")[1]}
        </Body1>
        <H4
          style={{
            color: "white",
          }}
        >
          {di.message}
        </H4>
      </VStack>
    );
  }
);

export const DiagnosticPage = observer(() => {
  return (
    <PageRoot title="Diagnostics">
      <Grid templateColumns="repeat(4, 1fr)" gap={6}>
        {Object.values(diagnoticsStore.message).map((diagnotic, index) => {
          return <DiagnosticView key={index} di={diagnotic} idx={index} />;
        })}
        <img
          src="/ros/video/oakd_preview"
          style={{
            width: "20rem",
            height: "10rem",
            borderRadius: "1rem",
            background: "black",
            margin: "10px",
          }}
        />
      </Grid>
    </PageRoot>
  );
});
