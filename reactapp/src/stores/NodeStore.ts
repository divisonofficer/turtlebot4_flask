import { makeAutoObservable, runInAction } from "mobx";
import { Node, NodeDetail } from "../page/nodes/NodePage";
import { httpGet } from "../connect/http/request";

class NodeStore {
  constructor() {
    makeAutoObservable(this);
  }

  nodes: Node[] = [];

  nodeDetailView: Node | undefined = undefined;
  nodeDetail: NodeDetail | undefined = undefined;

  fetchGetNodeList = () => {
    httpGet("/ros/nodes/list")
      .onSuccess((data) => {
        runInAction(() => {
          this.nodes = data;
        });
      })
      .fetch();
  };

  fetchGetNodeDetail = (node: Node) => {
    httpGet(`/ros/nodes/${node.name}`)
      .onSuccess((data) => {
        runInAction(() => {
          this.nodeDetail = data;
        });
      })
      .fetch();
  };
}

export const nodeStore = new NodeStore();
