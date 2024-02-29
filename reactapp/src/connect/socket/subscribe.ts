import io from 'socket.io-client';


const subscribe = (topic: string, events : [{eventName: string, callback: (data:any)=>void}]) => {
    
    const socket = io(topic);
    
    socket.on('connect', () => {
        console.log('Connected to ', topic);
    });

    // Listen for 'camera_info' events to receive data
    
    events.forEach(event => {
        socket.on(event.eventName, (data: any) => {
            event.callback(data);
        });
    })

    // Cleanup on component unmount
    return () => {
        socket.disconnect();
    };
}



export { subscribe }