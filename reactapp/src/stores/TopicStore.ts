import { makeAutoObservable, runInAction } from "mobx";
import { httpGet, httpPost } from "../connect/http/request";
import { TopicSpec } from "../data/Topic";

class TopicStore {
  topics: TopicSpec[] = [];
  topicGroup: { [key: string]: string[] } = {};
  topicGroupChecked: { [key: string]: boolean } = {};

  constructor() {
    makeAutoObservable(this);
  }

  private updateTopicGroup(topicGroup: { [key: string]: string[] }) {
    runInAction(() => {
      this.topicGroup = topicGroup;
    });
  }

  private updateTopics(topics: TopicSpec[]) {
    runInAction(() => {
      this.topics = topics;
    });
  }

  private updateTopicRunning(topic: string, running: boolean) {
    const _topic = this.topics.find((t) => t.topic === topic);
    if (_topic) {
      runInAction(() => {
        _topic.running = running;
      });
    }
  }

  getTopic(topic: string): TopicSpec | undefined {
    return this.topics.find((t) => t.topic === topic);
  }

  updateTopicGroupChecked(topicGroup: string, checked: boolean) {
    if (topicGroup === "all") {
      runInAction(() => {
        // set every keys to checked
        const newObject: { [key: string]: boolean } = {};
        Object.keys(this.topicGroupChecked).forEach((key) => {
          newObject[key] = checked;
        });
        this.topicGroupChecked = newObject;
      });
      return;
    }

    runInAction(() => {
      this.topicGroupChecked = {
        ...this.topicGroupChecked,
        [topicGroup]: checked,
      };
    });
  }

  fetchGetTopicList() {
    httpGet("/ros/topic/list")
      .onSuccess(
        (data: { topic: string; type: string[]; running: boolean }[]) => {
          this.updateTopics(
            data.map((topic) => {
              return {
                topic: topic.topic,
                type: topic.type[0],
                running: topic.running,
              };
            })
          );
          this.sliceTopicGroup(this.topics);
        }
      )
      .fetch();
  }

  fetchTopicOpen(topic: string) {
    const topic_type = this.topics.find((t) => t.topic === topic)?.type;
    if (topic_type) {
      httpPost(`/ros/topic`, {
        topic_name: topic,
        topic_type: topic_type,
      })
        .onSuccess(() => {
          this.updateTopicRunning(topic, true);
        })
        .fetch();
    }
  }

  fetchTopicClose(topic: string) {
    httpPost(`/ros/topic/delete`, {
      topic_name: topic,
    })
      .onSuccess(() => {
        this.updateTopicRunning(topic, false);
      })
      .fetch();
  }

  private sliceTopicGroup(topicList: TopicSpec[]) {
    // Create Tree structure by slicing topic name with "/"
    const topicGroup: { [key: string]: string[] } = {};
    topicList.forEach((topic) => {
      const topicName = topic.topic.split("/");
      if (topicName.length > 2) {
        if (!topicGroup[topicName[1]]) {
          topicGroup[topicName[1]] = [];
          this.updateTopicGroupChecked(topicName[1], true);
        }
        topicGroup[topicName[1]].push(topic.topic);
      } else {
        if (!topicGroup["/"]) {
          topicGroup["/"] = [];
          this.updateTopicGroupChecked("/", true);
        }
        topicGroup["/"].push(topic.topic);
      }
    });
    this.updateTopicGroup(topicGroup);
  }
}

export const topicStore = new TopicStore();
