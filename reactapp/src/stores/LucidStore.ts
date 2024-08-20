import { makeAutoObservable } from "mobx";
import { lucidSocket } from "../connect/socket/subscribe";
import { httpPost } from "../connect/http/request";

class LucidStore {
  lucidStatusJson: string = "{}";

  storageEnabled = false;

  constructor() {
    makeAutoObservable(this);

    lucidSocket.subscribe("status", (data) => {
      this.lucidStatusJson = JSON.stringify(data);
      this.storageEnabled = data.storage_enabled;
    });
  }

  fetchEnableStorage() {
    httpPost("/lucid/storage/enable").fetch();
  }

  fetchDisableStorage() {
    httpPost("/lucid/storage/disable").fetch();
  }
}

export const lucidStore = new LucidStore();
