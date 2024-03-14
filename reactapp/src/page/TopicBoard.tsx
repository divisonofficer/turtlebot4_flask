import {
  Checkbox,
  Grid,
  HStack,
  IconButton,
  Switch,
  VStack,
} from "@chakra-ui/react";
import { Body1, Body2, Body3, H3, H4 } from "../design/text/textsystem";
import { useCallback, useEffect, useState } from "react";
import { rosSocket } from "../connect/socket/subscribe";
import { httpDel, httpGet, httpPost } from "../connect/http/request";
import { InfoIcon, RepeatIcon } from "@chakra-ui/icons";
import { TopicInfo, TopicItem } from "./topic/TopicPage";
import { TopicSpec } from "../data/Topic";

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

const TopicButton = ({
  name,
  type,
  running,
}: {
  name: string;
  type?: string;
  running?: boolean;
}) => {
  const [hover, setHover] = useState(false);
  const [open, setOpen] = useState(running || false);

  const [topicNodes, setTopicNodes] = useState<TopicNodes>();
  const [log, setLog] = useState<string>("");

  const topicOpen = (status: Boolean) => {
    if (status) {
      httpPost(`/ros/topic`, {
        topic_name: name,
        topic_type: type,
      })
        .onSuccess(() => {
          setOpen(true);
        })
        .fetch();
    } else {
      httpPost(`/ros/topic/delete`, {
        topic_name: name,
      })
        .onSuccess(() => {
          setOpen(false);
        })
        .fetch();
    }
  };

  const getTopicNodes = useCallback(() => {
    httpPost(`/ros/topic/nodes`, {
      topic_name: name,
    })
      .onSuccess((data: TopicNodes) => {
        setTopicNodes(data);
      })
      .fetch();
  }, []);

  useEffect(() => {
    if (open) {
      console.log("subscribing to ", name);
      rosSocket.subscribe(name, (data: any) => {
        setLog(JSON.stringify(data));
      });
    } else {
      rosSocket.unsubscribe(name);
    }
  }, [open]);

  return (
    <VStack
      style={{
        width: "100%",
        alignItems: "flex-start",
      }}
    >
      <HStack
        style={{
          width: "100%",
          background: hover ? "#f0f0f0" : "white",
          padding: "1rem",
        }}
        onMouseEnter={() => setHover(true)}
        onMouseLeave={() => setHover(false)}
      >
        <H4>{name}</H4>
        <H4>{type}</H4>
        <div style={{ flexGrow: 1 }}></div>
        <IconButton
          aria-label="Search Nodes"
          icon={<InfoIcon />}
          onClick={() => getTopicNodes()}
        />
        <Switch
          isChecked={open}
          onChange={(e) => topicOpen(e.target.checked)}
        />
      </HStack>
      {open && (
        <VStack
          style={{
            width: "100%",
            padding: "1rem",
            alignItems: "flex-start",
          }}
        >
          <HStack>
            <Body3>{log}</Body3>
            {log.split("frame_id").length > 1 && (
              <img
                src={`
                            /ros/topic/preview/${JSON.parse(log)["frame_id"]}
                            `}
                style={{ width: "100px", height: "100px" }}
                alt=""
              />
            )}
          </HStack>
        </VStack>
      )}
      {topicNodes && (
        <VStack>
          {[
            {
              key: "Publishers",
              nodes: topicNodes.publishers,
            },
            {
              key: "Subscriptions",
              nodes: topicNodes.subscriptions,
            },
          ].map((key, i) => {
            return (
              <VStack key={key.key}>
                <H4>{key.key}</H4>
                <VStack>
                  {key.nodes.map((node, j) => {
                    return (
                      <VStack
                        style={{
                          alignItems: "flex-start",
                          width: "100%",
                        }}
                      >
                        <H3>{node["name"]}</H3>
                        <Body3>{node["namespace"]}</Body3>
                        <Body3>{node["qosProfile"].deadline}</Body3>
                        <Body3>{node["qosProfile"].durability}</Body3>
                        <Body3>{node["qosProfile"].lifespan}</Body3>
                        <Body3>{node["qosProfile"].liveliness}</Body3>
                      </VStack>
                    );
                  })}
                </VStack>
              </VStack>
            );
          })}
        </VStack>
      )}
      <div
        style={{
          width: "100%",
          height: "1px",
          background: "#1C1C1C0D",
        }}
      ></div>
    </VStack>
  );
};

const TopicBoard = () => {
  const [topicList, setTopicList] = useState<
    { topic: string; type: string; running: boolean }[]
  >([]);

  useEffect(() => {
    fetchTopicList();
  }, []);

  const [topicGroup, setTopicGroup] = useState<{ [key: string]: string[] }>({});
  const [topicGroupChecked, setTopicGroupChecked] = useState<{
    [key: string]: boolean;
  }>({});
  const [topicVisibleList, setTopicVisibleList] = useState<string[]>([]);

  const [onFetchingAlert, setOnFetchingAlert] = useState(false);

  const fetchTopicList = useCallback(() => {
    setOnFetchingAlert(true);
    httpGet("/ros/topic/list")
      .onSuccess(
        (data: { topic: string; type: string[]; running: boolean }[]) => {
          setTopicList(
            data
              .map((topic) => {
                return {
                  topic: topic.topic,
                  type: topic.type[0],
                  running: topic.running,
                };
              })
              .sort((a, b) => (a.running ? -1 : 1))
          );
          sliceTopicGroup(data);
          setOnFetchingAlert(false);
        }
      )
      .onError((e) => {
        console.log(e);
        setOnFetchingAlert(false);
      })
      .fetch();
  }, []);

  const sliceTopicGroup = (
    topicList: { topic: string; type: string[]; running: boolean }[]
  ) => {
    // Create Tree structure by slicing topic name with "/"
    const topicGroup: { [key: string]: string[] } = {};
    topicList.forEach((topic) => {
      const topicName = topic.topic.split("/");
      if (topicName.length > 2) {
        if (!topicGroup[topicName[1]]) {
          topicGroup[topicName[1]] = [];
          topicGroupChecked[topicName[1]] = true;
        }
        topicGroup[topicName[1]].push(topic.topic);
      } else {
        if (!topicGroup["/"]) {
          topicGroup["/"] = [];
          topicGroupChecked["/"] = true;
        }
        topicGroup["/"].push(topic.topic);
      }
    });
    setTopicGroup(topicGroup);
  };

  const topicGroupCollect = useCallback(() => {
    const topicList = [];
    console.log(topicGroupChecked);
    for (const key in topicGroupChecked) {
      if (topicGroupChecked[key]) {
        topicList.push(...topicGroup[key]);
      }
    }
    setTopicVisibleList(topicList);
  }, [topicGroupChecked, topicGroup]);
  useEffect(() => {
    topicGroupCollect();
  }, [topicGroupChecked, topicGroup, topicGroupCollect]);

  const TopicToolbar = () => {
    return (
      <HStack
        style={{
          width: "100%",
        }}
      >
        <Checkbox
          isChecked={Object.values(topicGroupChecked).every(
            (value) => value === true
          )}
          onChange={(e) => {
            if (e.target.checked) {
              let newTopicGroupChecked = { ...topicGroupChecked };
              for (const key in newTopicGroupChecked) {
                newTopicGroupChecked[key] = true;
              }
              setTopicGroupChecked(newTopicGroupChecked);
            } else {
              let newTopicGroupChecked = { ...topicGroupChecked };
              for (const key in newTopicGroupChecked) {
                newTopicGroupChecked[key] = false;
              }
              setTopicGroupChecked(newTopicGroupChecked);
            }
          }}
        >
          All
        </Checkbox>
        {Object.keys(topicGroup).map((key, i) => (
          <Checkbox
            key={key}
            isChecked={topicGroupChecked[key]}
            onChange={(e) => {
              setTopicGroupChecked({
                ...topicGroupChecked,
                [key]: e.target.checked,
              });
            }}
          >
            {key}
          </Checkbox>
        ))}
        <IconButton
          aria-label="Search database"
          icon={<RepeatIcon />}
          onClick={() => fetchTopicList()}
        />
      </HStack>
    );
  };

  const [topicDisplay, setTopicDisplay] = useState<TopicSpec>();
  return (
    <VStack
      style={{
        width: "100%",
        padding: "1rem",
      }}
    >
      {topicDisplay && <TopicInfo topic={topicDisplay} />}
      <TopicToolbar />
      <Grid templateColumns="repeat(4, 1fr)" gap={6}>
        {topicList.map(
          (topic, i) =>
            (topicVisibleList.includes(topic.topic) || topic.running) && (
              <TopicItem
                key={topic.topic}
                topic={topic}
                onClick={() => {
                  setTopicDisplay(topic);
                }}
              />
            )
        )}
      </Grid>
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
};

export default TopicBoard;
