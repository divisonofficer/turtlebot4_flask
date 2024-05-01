import { action, makeAutoObservable } from "mobx";

class AlertStore {
  alertList: AlertData[] = [];

  constructor() {
    makeAutoObservable(this);
  }

  @action
  addAlert(
    type: "error" | "warning" | "info" | "success",
    message: string,
    title?: string
  ) {
    this.alertList.push({ type, message, title });
  }

  @action
  consumeFirstAlert() {
    if (this.alertList.length > 0) {
      this.alertList.shift();
    }
  }
}

export interface AlertData {
  title?: string;
  message?: string;
  type?: "error" | "warning" | "info" | "success";
}

export const alertStore = new AlertStore();
