import { useState } from "react";
import { PkgNodeInfo } from "../../data/PkgNode";
import { PageRoot } from "../../design/other/flexs";
import NodeStatusBoard from "../NodeStatusBoard";
import PkgStatusBar from "../PkgStatusBar";

export const LaunchPage = () => {
  const [nodeChosen, setNodeChosen] = useState<PkgNodeInfo>();

  return (
    <PageRoot title="Launch">
      <PkgStatusBar onClickNode={setNodeChosen} />
      {nodeChosen && (
        <NodeStatusBoard
          node={nodeChosen}
          close={() => setNodeChosen(undefined)}
        />
      )}
    </PageRoot>
  );
};
