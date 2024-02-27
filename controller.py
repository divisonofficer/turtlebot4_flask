from controller_ros import *
import sys
import rclpy


class Controller:
    def __init__(self):
        rclpy.init(args=None)
    
    def con_stop_motor(self, args = None):
        return ros_lidar_stop_motor()
        
        
    def con_subscribe_camera_preview(self, args = None, callback = None):
        return ros_camera_preview_raw(callback) 



    def command_exists(self, command):
        return command in self.command_map
    
    def run_command(self, command, args = None):
        if not self.command_exists(command):
            print(f"Unknown command: {command}")
            print_help(self)
            return None
        

            
        
        command_function = self.command_map[command]["function"]
        if "callback" in self.command_map[command]:
            return command_function(self, args, lambda x: print(x))
        return command_function(self, args)
        
    command_map = {
        "stop_motor":{
            "function": con_stop_motor,
            "description": "Stop the motor of the LIDAR",
        },
        "subscribe_camera_preview":{
            "function": con_subscribe_camera_preview,
            "description": "Subscribe to the camera preview",
            "callback": True,
        },
    }



def print_help(controller: Controller):
    print("Usage: python3 controller.py <command>")
    print("Available commands:")
    for command, data in controller.command_map.items():
        print(f"  {command}: {data['description']}")

if __name__ == '__main__':
    args = sys.argv
    if len(args) < 2:
        print_help()
        sys.exit(1)

    
    controller = Controller()
    command = args[1]
    if not controller.command_exists(command):
        print(f"Unknown command: {command}")
        print_help()
        sys.exit(1)




    '''
    if -l in args: run into infinite loop
    '''
    if '-l' in args:
            # Use try-except to catch exceptions and cleanly shutdown
        subscriber = controller.run_command(command, args[2:])
        try:
            rclpy.spin(subscriber)
        except KeyboardInterrupt:
            pass  # Handle Ctrl+C here if needed
        finally:
            # Cleanly shutdown the node
            subscriber.destroy_node()
            rclpy.shutdown()
    else:
        controller.run_command(command, args[2:])

    
    
    