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
  message_timestamp: number = 0;
  message_session_active: boolean = false;
  message_timestamp_passed = 0;
  detailMessageKey: string = "";

  sessionCheckDaemon = setInterval(() => {
    this.message_session_active = Date.now() - this.message_timestamp < 5000;
    this.message_timestamp_passed =
      (Date.now() - this.message_timestamp) / 1000;
  }, 627);

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
        this.message_timestamp = Date.now();
      });
    });
  }
}

export const diagnoticsStore = new DiagnoticsStore();
