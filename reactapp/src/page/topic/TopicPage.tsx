import { HStack, VStack } from "@chakra-ui/react";
import { Body1, Body2, Body3, H2, H3, H4 } from "../../design/text/textsystem";
import { Btn } from "../../design/button/button";
import { useCallback, useEffect, useState } from "react";
import { TopicSpec } from "../../data/Topic";
import { httpPost } from "../../connect/http/request";
import TopicBoard, { TopicNodes, TopicNode } from "../TopicBoard";
import { rosSocket } from "../../connect/socket/subscribe";
import { InfoCard } from "../../design/other/infocard";
import {
  Clock,
  DownloadSimple,
  TelegramLogo,
  UploadSimple,
} from "@phosphor-icons/react";
import { m } from "framer-motion";

export type TopicOutput = {
  log: string;
  timeStamp: string;
  timeMs: number;
};

const TopicLogs = ({ logs }: { logs: TopicOutput[] }) => {
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
            <HStack
              style={{
                width: "100%",
                padding: "0rem 1rem",
                justifyContent: "space-between",
              }}
              key={log.timeMs}
            >
              <Body3>{log.timeStamp}</Body3>
              <Body3>{log.log}</Body3>
            </HStack>
          );
        })}
      </VStack>{" "}
    </VStack>
  );
};

const TopicNodeItem = ({
  head,
  nodes,
}: {
  head: string;
  nodes: TopicNode[];
}) => {
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
              }}
            >
              <H3 style={{ flex: 2 }}>{node["Node name"]}</H3>
              <Body3 style={{ flex: 3 }}>{node["Node namespace"]}</Body3>
            </HStack>
          );
        })}
      </VStack>
    </VStack>
  );
};

const TopicInfo = ({ topic }: { topic?: TopicSpec }) => {
  const [topicNodes, setTopicNodes] = useState<TopicNodes>();
  const [logs, setLogs] = useState<TopicOutput[]>([]);
  const [timeInterval, setTimeInterval] = useState(0);
  const [messageCount, setMessageCount] = useState(0);

  const getTopicNodes = useCallback(() => {
    httpPost(`/ros/topic/nodes`, {
      topic_name: topic?.topic,
    })
      .onSuccess((data: TopicNodes) => {
        setTopicNodes(data);
      })
      .fetch();
  }, []);

  useEffect(() => {
    if (topic?.running) {
      console.log("subscribing to ", topic.topic);
      rosSocket.subscribe(topic.topic, (data: any) => {
        publishLog(JSON.stringify(data));
      });
    } else {
      rosSocket.unsubscribe(topic?.topic || "");
    }
    getTopicNodes();
    return () => {
      rosSocket.unsubscribe(topic?.topic || "");
    };
  }, [topic]);

  const publishLog = (log: string) => {
    const timeStamp = new Date().toLocaleTimeString();
    const timeMs = new Date().getTime();
    var logList = logs;
    if (logList.length > 10) {
      logList.shift();
    }

    if (logList.length > 0) {
      setTimeInterval(timeMs - logList[logList.length - 1].timeMs);
    }

    setMessageCount(messageCount + 1);

    logList.push({ log, timeStamp, timeMs });
    setLogs(logList);
  };
  return (
    <VStack
      style={{
        width: "100%",
        alignItems: "flex-start",
        padding: "1rem",
      }}
    >
      <H2>{topic?.topic}</H2>
      <HStack>
        <InfoCard
          title="Subscriptions"
          value={topicNodes?.Subscriptions.length}
          icon={DownloadSimple}
        />
        <InfoCard
          title="Publishers"
          value={topicNodes?.Publishers.length}
          icon={UploadSimple}
          color="#E5ECF6"
        />
        <InfoCard title="Messages" value={messageCount} icon={TelegramLogo} />
        <InfoCard
          title="Interval"
          value={timeInterval + "ms"}
          icon={Clock}
          color="#E5ECF6"
        />
      </HStack>

      {logs.length > 0 && <TopicLogs logs={logs} />}
      {topicNodes &&
        [
          {
            key: "Publishers",
            nodes: topicNodes.Publishers,
          },
          {
            key: "Subscriptions",
            nodes: topicNodes.Subscriptions,
          },
        ].map((key, i) => {
          return <TopicNodeItem head={key.key} nodes={key.nodes} />;
        })}
    </VStack>
  );
};

const TopicItem = ({
  topic,
  onClick,
}: {
  topic?: TopicSpec;
  onClick?: () => void;
}) => {
  const [hover, setHover] = useState(false);
  const [open, setOpen] = useState(topic?.running || false);

  const status = open
    ? {
        color: "#4AA785",
        text: "Listening",
      }
    : {
        color: "#F2C94C",
        text: "Not Listening",
      };

  const topicOpen = (status: Boolean) => {
    if (status) {
      httpPost(`/ros/topic`, {
        topic_name: topic?.topic,
        topic_type: topic?.type,
      })
        .onSuccess(() => {
          setOpen(true);
        })
        .fetch();
    } else {
      httpPost(`/ros/topic/delete`, {
        topic_name: topic?.topic,
      })
        .onSuccess(() => {
          setOpen(false);
        })
        .fetch();
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

const TopicPage = () => {
  return (
    <VStack
      style={{
        height: "100%",
        width: "100%",
        overflowY: "auto",
      }}
    >
      <TopicBoard />
    </VStack>
  );
};

export { TopicInfo, TopicItem, TopicPage };
