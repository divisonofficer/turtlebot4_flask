import {
  Checkbox,
  Flex,
  Grid,
  HStack,
  IconButton,
  Input,
  VStack,
} from "@chakra-ui/react";
import { useEffect, useState } from "react";

import { RepeatIcon } from "@chakra-ui/icons";
import { TopicItem } from "./topic/TopicPage";
import { topicStore } from "../stores/TopicStore";
import { observer } from "mobx-react";
import { reaction } from "mobx";
import { useNavigate } from "react-router-dom";

export type TopicNodes = {
  publishers: TopicNode[];
  subscriptions: TopicNode[];
};
export type TopicNode = {
  name: string;
  namespace: string;
  qosProfile: QoSProfile;
};

type QoSProfile = {
  deadline: string;
  durability: string;
  lifespan: string;
  liveliness: string;
};

export const TopicPreviewBoard = () => {
  const [topicList, setTopicList] = useState<
    { topic: string; type: string; running: boolean }[]
  >([]);
};

const TopicToolbar = observer(
  ({
    topicSearchText,
    setTopicSearchText,
  }: {
    topicSearchText: string;
    setTopicSearchText: (text: string) => void;
  }) => {
    return (
      <VStack>
        <Flex
          style={{
            width: "100%",
          }}
          wrap="wrap"
        >
          <Checkbox
            isChecked={Object.values(topicStore.topicGroupChecked).every(
              (value) => value === true
            )}
            onChange={(e) => {
              topicStore.updateTopicGroupChecked("all", e.target.checked);
            }}
          >
            All
          </Checkbox>
          {Object.keys(topicStore.topicGroup).map((key, i) => (
            <Checkbox
              key={key}
              isChecked={topicStore.topicGroupChecked[key]}
              onChange={(e) => {
                topicStore.updateTopicGroupChecked(key, e.target.checked);
              }}
            >
              {key}
            </Checkbox>
          ))}
          <IconButton
            aria-label="Search database"
            icon={<RepeatIcon />}
            onClick={() => topicStore.fetchGetTopicList()}
          />
        </Flex>
        <Input
          value={topicSearchText}
          onChange={(e) => setTopicSearchText(e.target.value)}
        />
      </VStack>
    );
  }
);

const TopicBoard = observer(() => {
  useEffect(() => {
    topicStore.fetchGetTopicList();
  }, []);

  const [topicVisibleList, setTopicVisibleList] = useState<string[]>([]);

  const [topicSearchText, setTopicSearchText] = useState<string>("");

  const [onFetchingAlert, setOnFetchingAlert] = useState(false);

  useEffect(() => {
    const dispose = reaction(
      () => [topicStore.topicGroup, topicStore.topicGroupChecked] as const,
      ([groups, groupChecked]) => {
        const topicList = [];
        for (const key in groupChecked) {
          if (groupChecked[key] && groups[key]) {
            topicList.push(...groups[key]);
          }
        }

        setTopicVisibleList(topicList);
      }
    );
    return () => dispose();
  }, []);

  const navigate = useNavigate();

  return (
    <VStack
      style={{
        width: "100%",
        padding: "1rem",
      }}
    >
      <TopicToolbar
        topicSearchText={topicSearchText}
        setTopicSearchText={setTopicSearchText}
      />
      <Flex wrap={"wrap"} gap={6}>
        {topicStore.topics.map(
          (topic, i) =>
            ((topicVisibleList.includes(topic.topic) &&
              topic.topic.includes(topicSearchText)) ||
              topic.running) && (
              <TopicItem
                key={topic.topic}
                topic={topic}
                onClick={() => {
                  topicStore.topicDetailView = topic.topic;
                  navigate("/topic/detail");
                }}
              />
            )
        )}
      </Flex>
      {
        /* 
            A loading bar floating on top of the topic list
            */
        onFetchingAlert && (
          <div
            style={{
              position: "absolute",
              top: 0,
              left: 0,
              width: "100%",
              height: "100%",
              background: "rgba(255, 255, 255, 0.5)",
              display: "flex",
              justifyContent: "center",
              alignItems: "center",
            }}
          >
            <RepeatIcon w={8} h={8} />
          </div>
        )
      }
    </VStack>
  );
});

export default TopicBoard;
