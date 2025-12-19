#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys


class DCS103eClient(Node):
    def __init__(self):
        super().__init__("dcs103e_client")

        # Create service clients
        self.connect_client = self.create_client(Trigger, "/dcs103e_controller/connect")
        self.disconnect_client = self.create_client(
            Trigger, "/dcs103e_controller/disconnect"
        )

        # Channel control clients
        self.channel_enable_clients = []
        self.channel_disable_clients = []

        for i in range(3):
            enable_client = self.create_client(
                Trigger, f"/dcs103e_controller/channel_{i}/enable"
            )
            disable_client = self.create_client(
                Trigger, f"/dcs103e_controller/channel_{i}/disable"
            )

            self.channel_enable_clients.append(enable_client)
            self.channel_disable_clients.append(disable_client)

    def wait_for_services(self):
        """Wait for all services to be available"""
        self.get_logger().info("Waiting for DCS103e services...")

        services = [self.connect_client, self.disconnect_client]
        services.extend(self.channel_enable_clients)
        services.extend(self.channel_disable_clients)

        for service in services:
            if not service.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Service {service.srv_name} not available")
                return False

        self.get_logger().info("All services are available!")
        return True

    def connect(self):
        """Connect to DCS103e device"""
        request = Trigger.Request()

        future = self.connect_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f"Connect result: {response.success}, {response.message}"
            )
            return response.success
        else:
            self.get_logger().error("Failed to call connect service")
            return False

    def disconnect(self):
        """Disconnect from DCS103e device"""
        request = Trigger.Request()

        future = self.disconnect_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f"Disconnect result: {response.success}, {response.message}"
            )
            return response.success
        else:
            self.get_logger().error("Failed to call disconnect service")
            return False

    def enable_channel(self, channel):
        """Enable a specific channel"""
        if channel < 0 or channel >= 3:
            self.get_logger().error(f"Invalid channel number: {channel}")
            return False

        request = Trigger.Request()

        future = self.channel_enable_clients[channel].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f"Enable channel {channel} result: {response.success}, {response.message}"
            )
            return response.success
        else:
            self.get_logger().error(
                f"Failed to call enable service for channel {channel}"
            )
            return False

    def disable_channel(self, channel):
        """Disable a specific channel"""
        if channel < 0 or channel >= 3:
            self.get_logger().error(f"Invalid channel number: {channel}")
            return False

        request = Trigger.Request()

        future = self.channel_disable_clients[channel].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f"Disable channel {channel} result: {response.success}, {response.message}"
            )
            return response.success
        else:
            self.get_logger().error(
                f"Failed to call disable service for channel {channel}"
            )
            return False


def main():
    rclpy.init()

    client = DCS103eClient()

    if not client.wait_for_services():
        client.get_logger().error("Failed to connect to services")
        rclpy.shutdown()
        return

    # Parse command line arguments
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 dcs103e_client.py connect")
        print("  python3 dcs103e_client.py disconnect")
        print("  python3 dcs103e_client.py enable <channel>")
        print("  python3 dcs103e_client.py disable <channel>")
        rclpy.shutdown()
        return

    command = sys.argv[1]

    if command == "connect":
        client.connect()
    elif command == "disconnect":
        client.disconnect()
    elif command == "enable":
        if len(sys.argv) != 3:
            print("Usage: python3 dcs103e_client.py enable <channel>")
            rclpy.shutdown()
            return
        try:
            channel = int(sys.argv[2])
            client.enable_channel(channel)
        except ValueError:
            print("Channel must be a number (0, 1, or 2)")
    elif command == "disable":
        if len(sys.argv) != 3:
            print("Usage: python3 dcs103e_client.py disable <channel>")
            rclpy.shutdown()
            return
        try:
            channel = int(sys.argv[2])
            client.disable_channel(channel)
        except ValueError:
            print("Channel must be a number (0, 1, or 2)")
    else:
        print(f"Unknown command: {command}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
