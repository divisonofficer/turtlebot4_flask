import io from 'socket.io-client';

class SocketWrapper{
    socket:any;
    subscriptions: {[key: string]: (data:any)=>void} = {};
    constructor(_namespace: string){
        this.socket = io(_namespace);
        this.socket.on('connect', () => {
            console.log('Connected to ', _namespace);
        }
        );
    }

    subscribe(event: string, callback: (data:any)=>void){
        if (this.subscriptions[event]){
            this.socket.off(event, this.subscriptions[event]);
        }
        this.socket.on(event, callback);
    }

    subscribeDebug(event: string){
        this.socket.on(event, (data:any) => {
            console.log(event, data);
        });
    }


}



const rosSocket = new SocketWrapper('/socket/ros');

export { rosSocket }