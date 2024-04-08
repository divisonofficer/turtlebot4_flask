import { HStack, IconButton, Textarea, VStack } from "@chakra-ui/react";
import { Body2, Body3, H2, H3, H4 } from "../../design/text/textsystem";
import { Btn } from "../../design/button/button";
import { useEffect, useState } from "react";
import { TopicSpec } from "../../data/Topic";
import { httpPost } from "../../connect/http/request";
import TopicBoard, { TopicNodes, TopicNode } from "../TopicBoard";
import { RepeatOnce, TelegramLogo } from "@phosphor-icons/react";
import { stringify } from "json5";
import { observer } from "mobx-react";
import { topicStore } from "../../stores/TopicStore";
import { PageRoot } from "../../design/other/flexs";
import { useNavigate } from "react-router-dom";
import { nodeStore } from "../../stores/NodeStore";
import { Color } from "../../design/color";

export type TopicOutput = {
  interval: number;
  log: string;
  timeStamp: string;
  timeMs: number;
  topic?: string;
};

export const PublishTest = ({ topic }: { topic: TopicSpec }) => {
  const [topicJson, setTopicJson] = useState<string>("");
  const fetchFormat = () => {
    httpPost("/ros/topic/type/format", {
      topic_name: topic.topic,
      topic_type: topic.type,
    })
      .onSuccess((data: any) => {
        const jsonStr = JSON.stringify(data, null, 2);
        setTopicJson(jsonStr);
      })
      .fetch();
  };

  const fetchJsonMsg = () => {
    httpPost("/ros/topic/publish", {
      topic_name: topic.topic,
      topic_type: topic.type,
      message: topicJson,
    })
      .onError((code, message, error) => {
        console.log(code, message, error);
      })
      .fetch();
  };
  useEffect(() => {
    fetchFormat();
  }, [topic]);

  return (
    <VStack
      style={{
        width: "100%",
        height: "20rem",
        alignItems: "flex-start",
      }}
    >
      <HStack
        style={{
          width: "100%",
          justifyContent: "space-between",
        }}
      >
        <H2>Publish Test</H2>
        <HStack>
          <IconButton
            icon={<RepeatOnce />}
            aria-label={""}
            onClick={fetchFormat}
          />
          <IconButton
            icon={<TelegramLogo />}
            aria-label={""}
            onClick={() => {
              fetchJsonMsg();
            }}
          />
        </HStack>
      </HStack>
      <Textarea
        value={topicJson}
        style={{
          height: "100%",
          background: "#F7F9FB",
          borderRadius: "1rem",
          padding: "1rem",
        }}
        onChange={(e) => {
          setTopicJson(e.target.value);
        }}
      />
    </VStack>
  );
};

export const TopicLogs = ({
  topic,
  logs,
}: {
  topic: string;
  logs: TopicOutput[];
}) => {
  return (
    <VStack
      style={{
        width: "100%",
        alignItems: "flex-start",
      }}
    >
      <H3>Logs</H3>
      <VStack
        style={{
          width: "100%",
          borderRadius: "1rem",
          background: "#E5ECF6",
        }}
      >
        <HStack
          style={{
            width: "100%",
            padding: "1rem",
            justifyContent: "space-between",
          }}
        >
          <Body3>Time</Body3>
          <Body3>Log</Body3>
        </HStack>

        {logs.map((log, i) => {
          return (
            log.topic === topic && (
              <HStack
                style={{
                  width: "100%",
                  padding: "0rem 1rem",
                  justifyContent: "space-between",
                }}
                key={topic + stringify(i)}
              >
                <Body3>{log.timeStamp}</Body3>
                <Body3>{log.log}</Body3>
              </HStack>
            )
          );
        })}
      </VStack>{" "}
    </VStack>
  );
};

export const TopicNodeItem = ({
  head,
  nodes,
}: {
  head: string;
  nodes: TopicNode[];
}) => {
  const navigate = useNavigate();
  return (
    <VStack
      style={{
        width: "100%",
        alignItems: "flex-start",
      }}
    >
      <H4>{head}</H4>
      <VStack
        style={{
          width: "100%",
          background: "#F7F9FB",
          borderRadius: "1rem",
          overflow: "hidden",
        }}
      >
        <HStack
          style={{
            width: "100%",
            marginLeft: "2rem",
            marginTop: "1rem",
          }}
        >
          <Body3 style={{ flex: 2 }}>Name</Body3>
          <Body3 style={{ flex: 3 }}>Namespace</Body3>
        </HStack>
        {nodes.map((node, j) => {
          return (
            <HStack
              style={{
                alignItems: "center",
                width: "100%",
                paddingLeft: "1rem",
                paddingTop: "1rem",
                paddingBottom: "1rem",
                background: j % 2 === 1 ? "#F7F9FB" : "#E5ECF6",
                cursor: "pointer",
              }}
              _hover={{
                background: j % 2 === 1 ? "#d7d9dB" : "#C4C4C4",
              }}
              onClick={() => {
                nodeStore.nodeDetailView = node;
                navigate("/nodes/detail");
              }}
            >
              <H3 style={{ flex: 2 }}>{node["name"]}</H3>
              <Body3 style={{ flex: 3 }}>{node["namespace"]}</Body3>
            </HStack>
          );
        })}
      </VStack>
    </VStack>
  );
};

export interface InfoState {
  topicNodes?: TopicNodes;
  logs: TopicOutput[];
  timeInterval: number;
  messageCount: number;
}

const TopicItem = ({
  topic,
  onClick,
}: {
  topic?: TopicSpec;
  onClick?: () => void;
}) => {
  const [hover, setHover] = useState(false);
  const open = topicStore.getTopic(topic?.topic!)?.running || false;
  const publishing =
    (topicStore.getTopic(topic?.topic!)?.nodes?.publishers.length ?? 0) > 0;
  const subscribing =
    (topicStore.getTopic(topic?.topic!)?.nodes?.subscriptions.length ?? 0) > 0;

  const status = open
    ? {
        color: "#4AA785",
        text: "Listening",
      }
    : publishing && subscribing
    ? {
        color: Color.Blue,
        text: "Communicating",
      }
    : publishing
    ? {
        color: Color.Mint,
        text: "Publishing",
      }
    : {
        color: "#F2C94C",
        text: "Not Publishing",
      };

  const topicOpen = (status: Boolean) => {
    if (status) {
      topicStore.fetchTopicOpen(topic?.topic!);
    } else {
      topicStore.fetchTopicClose(topic?.topic!);
    }
  };

  return (
    <VStack
      style={{
        width: "16rem",
        height: "9rem",
        background: "#F7F9FB",
        borderRadius: "1rem",
        alignItems: "flex-start",
        padding: "1rem",
        ...(hover && {
          boxShadow: "0px 4px 4px rgba(0, 0, 0, 0.25)",
          transition: "box-shadow 0.2s ease",
          background: "#d7d9dB",
        }),
      }}
      onMouseEnter={() => setHover(true)}
      onMouseLeave={() => setHover(false)}
      onClick={onClick}
    >
      <H4
        style={{
          fontSize: "1rem",
        }}
      >
        {topic && topic.topic}
      </H4>
      <Body3>{topic && topic.type}</Body3>
      <HStack
        style={{
          width: "100%",
          justifyContent: "flex-end",
        }}
      >
        <Body3>152ms</Body3>
        <Btn varient="borderless" size="sm" onClick={() => topicOpen(!open)}>
          <HStack>
            <div
              style={{
                width: "0.2rem",
                height: "0.2rem",
                borderRadius: "0.1rem",
                backgroundColor: status.color,
              }}
            />
            <Body2
              style={{
                color: status.color,
                fontWeight: "bold",
              }}
            >
              {status.text}
            </Body2>
          </HStack>
        </Btn>
      </HStack>
    </VStack>
  );
};

const TopicPage = observer(() => {
  return (
    <PageRoot
      title="Topics"
      style={{
        height: "100%",
        width: "100%",
        overflowY: "auto",
      }}
    >
      <TopicBoard />
    </PageRoot>
  );
});

export { TopicItem, TopicPage };
