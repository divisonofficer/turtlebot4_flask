import { TopicNode, TopicNodes } from "../page/TopicBoard";

export interface TopicSpec {
  topic: string;
  type: string;
  nodes?: TopicNodes;
  running: boolean;
}
