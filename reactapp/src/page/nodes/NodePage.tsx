import { useEffect, useState } from "react";
import { PageRoot } from "../../design/other/flexs";
import { httpGet } from "../../connect/http/request";
import { Flex, HStack, VStack } from "@chakra-ui/react";
import { Body3, H1, H3 } from "../../design/text/textsystem";
import { Btn } from "../../design/button/button";
import { observer } from "mobx-react";
import { nodeStore } from "../../stores/NodeStore";
import { useNavigate } from "react-router-dom";
import { topicStore } from "../../stores/TopicStore";

export interface Node {
  name: string;
  namespace: string;
}

export interface Act {
  name: string;
  type: string;
}
export interface NodeDetail {
  services: Act[];
  subscriptions: Act[];
  publishers: Act[];
  clients: Act[];
}

const ActBlock = ({ act }: { act: Act }) => {
  return (
    <VStack
      justifyContent="space-between"
      alignItems="flex-start"
      borderRadius="0.5rem"
      _hover={{ bg: "gray.200" }}
      padding="0.5rem" // Add padding of 0.5rem
    >
      <Body3>{act.name}</Body3>
      <Body3>{act.type}</Body3>
    </VStack>
  );
};

const TopicBlock = ({ act }: { act: Act }) => {
  const navigate = useNavigate();
  return (
    <Btn
      size="md"
      varient="gray"
      onClick={() => {
        topicStore.topicDetailView = act.name;
        navigate("/topic/detail");
      }}
    >
      {act.name}
    </Btn>
  );
};

export const NodeDetailView = observer(({ node }: { node?: Node }) => {
  useEffect(() => {
    node && nodeStore.fetchGetNodeDetail(node);
  }, [node]);

  return (
    <VStack width="100%" alignItems="flex-start" flexGrow={1} gap={0}>
      <H1>{node?.name}</H1>
      <H3>Subscriptions</H3>
      <Flex width="100%" flexWrap="wrap" gap={2}>
        {nodeStore.nodeDetail?.subscriptions.map((subscription) => {
          return <TopicBlock act={subscription} />;
        })}
      </Flex>
      <H3>Publishers</H3>
      <Flex width="100%" flexWrap="wrap" gap={2}>
        {nodeStore.nodeDetail?.publishers.map((publisher) => {
          return <TopicBlock act={publisher} />;
        })}
      </Flex>
      <H3>Clients</H3>
      <Flex width="100%" flexWrap="wrap" gap={2}>
        {nodeStore.nodeDetail?.clients.map((client) => {
          return <ActBlock act={client} />;
        })}
      </Flex>

      <H3>Services</H3>
      <Flex width="100%" flexWrap="wrap" gap={2}>
        {nodeStore.nodeDetail?.services.map((service) => {
          return <ActBlock act={service} />;
        })}
      </Flex>
    </VStack>
  );
});

export const NodeDetailPage = observer(() => {
  return (
    <PageRoot
      title="Node Detail"
      backPage={{
        title: "Nodes",
        path: "/nodes",
      }}
    >
      <NodeDetailView node={nodeStore.nodeDetailView} />
    </PageRoot>
  );
});

export const NodePage = observer(() => {
  const navigate = useNavigate();
  useEffect(() => {
    nodeStore.fetchGetNodeList();
  }, []);
  return (
    <PageRoot title="Nodes">
      <Flex width="100%" flexWrap="wrap" gap={2}>
        {nodeStore.nodes.map((node) => {
          return (
            <Btn
              onClick={() => {
                nodeStore.nodeDetailView = node;
                navigate("/nodes/detail");
              }}
              key={node.name}
            >
              {node.name}
            </Btn>
          );
        })}
      </Flex>
    </PageRoot>
  );
});
