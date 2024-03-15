import { action, makeAutoObservable, runInAction } from "mobx";
import { rosSocket } from "../connect/socket/subscribe";

export interface Diagnotic {
  name: string;
  message: string;
  hardware_id: string;
  values: { [key: string]: string };
}

class DiagnoticsStore {
  message: { [key: string]: Diagnotic } = {};

  detailMessageKey: string = "";

  constructor() {
    makeAutoObservable(this);
    this.subscribeToDiagnostics();
  }

  putDetailMessage(name: string) {
    this.detailMessageKey = name;
  }

  subscribeToDiagnostics() {
    rosSocket.subscribe("/turtlebot/diagnostic", (message: any) => {
      runInAction(() => {
        this.message = JSON.parse(message);
      });
    });
  }
}

export const diagnoticsStore = new DiagnoticsStore();
