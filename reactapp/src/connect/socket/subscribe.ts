import io, { Socket } from "socket.io-client";

class SocketWrapper {
  socket: Socket;
  subscriptions: { [key: string]: (data: any) => void } = {};
  constructor(
    _namespace: string,
    path: string = "/socket.io",
    transports: string[] = ["polling"]
  ) {
    this.socket = io(_namespace, {
      path: path,
      transports: transports, // Add this line
    }).connect();
    this.socket.on("connect", () => {
      console.log("Connected to ", _namespace);
    });
  }

  subscribe(event: string, callback: (data: any) => void) {
    if (this.subscriptions[event]) {
      this.socket.off(event, this.subscriptions[event]);
    }
    this.socket.on(event, callback);
  }

  subscribeBuffer(
    event: string,
    protoBufDefinition: { decode: any },
    callback: (data: any) => void
  ) {
    if (this.subscriptions[event]) {
      this.socket.off(event, this.subscriptions[event]);
    }
    this.socket.on(event, (data: ArrayBuffer) => {
      const decoded = protoBufDefinition.decode(new Uint8Array(data));
      callback(decoded);
    });
  }

  subscribeDebug(event: string) {
    this.socket.on(event, (data: any) => {
      console.log(event, data);
    });
  }

  unsubscribe(event: string) {
    this.socket.off(event, this.subscriptions[event]);
  }

  publish(event: string, data: any) {
    this.socket.emit(event, data);
  }
}

const rosSocket = new SocketWrapper("/socket/ros");
const slamSocket = new SocketWrapper("/slam", "/slam/socket.io");
const laptopSocket = new SocketWrapper("/socket/battery", "/laptop/socket.io");
export const captureSocket = new SocketWrapper("/socket", "/capture/socket.io");
export { rosSocket, slamSocket, laptopSocket };
export const jaiSocket = new SocketWrapper("", "/jai/socket.io");
export const lucidSocket = new SocketWrapper("", "/lucid/socket.io");
