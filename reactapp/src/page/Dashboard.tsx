import { Text, VStack } from "@chakra-ui/react";
import PkgStatusBar from "./PkgStatusBar";
import { useState } from "react";
import { PkgNodeInfo } from "../data/PkgNode";
import NodeStatusBoard from "./NodeStatusBoard";
import { H2 } from "../design/text/textsystem";
import { VDivBlock } from "../design/other/blocks";
import TopicBoard from "./TopicBoard";
import JoysticPage from "./Joystick";

const Dashboard = () => {
  const [nodeChosen, setNodeChosen] = useState<PkgNodeInfo>();

  return (
    <VStack
      style={{
        width: "100%",
        height: "100vh",

        overflowY: "auto",
        justifyContent: "flex-start",
        alignItems: "flex-start",
        paddingLeft: "2rem",
        paddingTop: "2rem",
        background: "#ffffff",
        overflowX: "hidden",
      }}
    >
      <H2>Nodes</H2>
      <VDivBlock color="#A1E3CB" />
      <PkgStatusBar onClickNode={setNodeChosen} />
      {nodeChosen && (
        <NodeStatusBoard
          node={nodeChosen}
          close={() => setNodeChosen(undefined)}
        />
      )}
      <H2>Topics</H2>
      <VDivBlock color="#A1E3CB" />
      <TopicBoard />
    </VStack>
  );
};

export default Dashboard;
