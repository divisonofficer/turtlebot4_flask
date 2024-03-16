import { Flex, HStack, VStack, useMediaQuery } from "@chakra-ui/react";
import { H2 } from "../../design/text/textsystem";
import { useCallback, useEffect, useRef, useState } from "react";
import { TopicSpec } from "../../data/Topic";
import { httpPost } from "../../connect/http/request";
import { TopicNodes } from "../TopicBoard";
import { rosSocket } from "../../connect/socket/subscribe";
import { InfoCard, InfoCardBtn } from "../../design/other/infocard";
import {
  Article,
  Clock,
  DoorOpen,
  DownloadSimple,
  Paperclip,
  Play,
  Stop,
  TelegramLogo,
  UploadSimple,
} from "@phosphor-icons/react";
import { observer } from "mobx-react";
import { topicStore } from "../../stores/TopicStore";
import { InfoState, PublishTest, TopicLogs, TopicNodeItem } from "./TopicPage";

export const TopicInfo = observer(({ topic }: { topic?: TopicSpec }) => {
  const [blockOpen, setBlockOpen] = useState({
    publish: false,
    logs: true,
    nodes: true,
  });
  const [state, setState] = useState<InfoState>({
    topicNodes: undefined,
    logs: [],
    timeInterval: 0,
    messageCount: 0,
  });

  const getTopicNodes = useCallback(() => {
    httpPost(`/ros/topic/nodes`, {
      topic_name: topic?.topic,
    })
      .onSuccess((data: TopicNodes) => {
        console.log(data);
        setState((prev) => ({ ...prev, topicNodes: data }));
      })
      .fetch();
  }, [topic?.topic]);

  const getTopicLogs = useCallback(() => {
    setState((prev) => ({
      ...prev,
      logs: [],
      timeInterval: 0,
      messageCount: 0,
    }));
    const topicName = topic?.topic;
    httpPost(`/ros/topic/logs`, {
      topic_name: topicName,
    })
      .onSuccess(
        (data: { count: number; interval: number; logs: Object[] }) => {
          setState((prev) => ({
            ...prev,
            messageCount: data.count || 0,
            timeInterval: data.interval || 0,
            logs: data["logs"]
              ? data["logs"].map((log: any) => {
                  return {
                    topic: topic?.topic,
                    log: log["log"],
                    timeStamp: log["timeStamp"],
                    timeMs: log["timeMs"],
                    interval: log["interval"],
                  };
                })
              : [],
          }));
        }
      )
      .onError(() => {})
      .fetch();
  }, [topic]);
  const publishLog = useCallback(
    (log: string, interval: number) => {
      var logList = state.logs;
      if (logList.length > 10) {
        logList.shift();
      }

      const timeStamp = new Date().toLocaleTimeString();
      logList.push({
        log,
        timeStamp,
        timeMs: new Date().getTime(),
        interval,
        topic: topic?.topic,
      });
      if (logList.length > 0) {
        setState((prev) => ({
          ...prev,
          timeInterval: interval * 1000,
          messageCount: prev.messageCount + 1,
          logs: logList,
        }));
      }
    },
    [topic]
  );
  useEffect(() => {
    topic &&
      rosSocket.subscribe(topic.topic, (data: any) => {
        const interval = data["interval"].toFixed(6);
        publishLog(JSON.stringify(data), interval);
      });
    getTopicNodes();
    getTopicLogs();

    return () => {
      rosSocket.unsubscribe(topic?.topic || "");
    };
  }, [topic, getTopicNodes, getTopicLogs, publishLog]);

  const openTopic = () => {
    topicStore.fetchTopicOpen(topic?.topic!);
  };

  const closeTopic = () => {
    topicStore.fetchTopicClose(topic?.topic!);
  };

  const TopicMenus = observer(() => {
    const cards = [
      <InfoCard
        title="Subscriptions"
        value={state.topicNodes?.subscriptions.length}
        icon={DownloadSimple}
      />,
      <InfoCard
        title="Publishers"
        value={state.topicNodes?.publishers.length}
        icon={UploadSimple}
        color="#E5ECF6"
      />,
      <InfoCard
        title="Messages"
        value={state.messageCount}
        icon={TelegramLogo}
      />,
      <InfoCard
        title="Interval"
        value={state.timeInterval.toFixed(2) + "ms"}
        icon={Clock}
        color="#E5ECF6"
      />,
    ];
    const btns = [
      topicStore.getTopic(topic?.topic!)?.running ? (
        <InfoCardBtn
          title="Shutdown"
          Icon={Stop}
          color="#FFCC00"
          onClick={closeTopic}
        />
      ) : (
        <InfoCardBtn
          Icon={Play}
          title="Subscribe"
          color="#34C759"
          onClick={openTopic}
        />
      ),
      <InfoCardBtn
        title="Publish"
        Icon={DoorOpen}
        onClick={() =>
          setBlockOpen((prev) => ({ ...prev, publish: !prev.publish }))
        }
      />,
      <InfoCardBtn
        title="Logs"
        color="#EB5A35"
        Icon={Paperclip}
        onClick={() => setBlockOpen((prev) => ({ ...prev, logs: !prev.logs }))}
      />,
      <InfoCardBtn
        title="Nodes"
        color="#7881FF"
        Icon={Article}
        onClick={() =>
          setBlockOpen((prev) => ({ ...prev, nodes: !prev.nodes }))
        }
      />,
      <div />,
    ];

    const containerRef = useRef<HTMLDivElement>(null);
    const isMobile = useMediaQuery("(max-width: 1068px)")[0];
    return (
      <Flex
        ref={containerRef}
        style={{
          width: "100%",
        }}
      >
        {isMobile ? (
          <VStack>
            {cards.map((card, i) => {
              return (
                <HStack
                  style={{
                    width: "100%",
                    alignItems: "flex-start",
                  }}
                >
                  {card}
                  {btns[i]}
                </HStack>
              );
            })}
          </VStack>
        ) : (
          <HStack
            style={{
              width: "100%",
              justifyContent: "flex-start",
            }}
          >
            {cards}
            {btns}
          </HStack>
        )}
      </Flex>
    );
  });

  return (
    <VStack
      style={{
        width: "100%",
        alignItems: "flex-start",
        padding: "1rem",
      }}
    >
      <H2>{topic?.topic}</H2>
      <TopicMenus />
      {topic && blockOpen.publish && <PublishTest topic={topic} />}
      {state.logs.length > 0 && blockOpen.logs && (
        <TopicLogs topic={topic!.topic} logs={state.logs} />
      )}
      {state.topicNodes &&
        blockOpen.nodes &&
        [
          {
            key: "Publishers",
            nodes: state.topicNodes.publishers,
          },
          {
            key: "Subscriptions",
            nodes: state.topicNodes.subscriptions,
          },
        ].map((key, i) => {
          return <TopicNodeItem head={key.key} nodes={key.nodes} />;
        })}
    </VStack>
  );
});
