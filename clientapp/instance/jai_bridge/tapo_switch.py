from tapo import ApiClient
import asyncio
import sys
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import asyncio
import threading

TAPO_USERNAME = os.getenv("TAPO_USERNAME")
TAPO_PASSWORD = os.getenv("TAPO_PASSWORD")

TAPO_IP = "169.254.0.3"


class RosTapoNode(Node):

    def __init__(self):
        super().__init__("tapo_node")
        try:
            self.start_event_loop()
            asyncio.run_coroutine_threadsafe(
                self.__tapo_device_init(), self.loop
            ).result()

            self.service_server_turn_on = self.create_service(
                Trigger, "tapo/on", self.turn_on
            )
            self.service_server_turn_off = self.create_service(
                Trigger, "tapo/off", self.turn_off
            )
        except Exception as e:
            print(e)

    def start_event_loop(self):
        self.loop = asyncio.new_event_loop()

        def loop_thread(loop):
            asyncio.set_event_loop(loop)
            loop.run_forever()

        t = threading.Thread(target=loop_thread, args=(self.loop,), daemon=True)
        t.start()

    async def __turn_on(self, request, response):
        try:
            await self.tapo_device.on()
            response.success = True
        except Exception as e:
            response.success = False
            response.message = str(e)
            print(e)
        return response

    async def __turn_off(self, request, response):
        try:
            await self.tapo_device.off()
            response.success = True
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def turn_on(self, request, response):
        future = asyncio.run_coroutine_threadsafe(
            self.__turn_on(request, response), self.loop
        )
        return future.result()

    def turn_off(self, request, response):
        future = asyncio.run_coroutine_threadsafe(
            self.__turn_off(request, response), self.loop
        )
        return future.result()

    async def __tapo_device_init(self):
        self.tapo_client = ApiClient(TAPO_USERNAME, TAPO_PASSWORD)
        self.tapo_device = await self.tapo_client.p110(TAPO_IP)

    def __del__(self):
        asyncio.run(self.tapo_device.off())


async def main():
    client = ApiClient(TAPO_USERNAME, TAPO_PASSWORD)
    device = await client.p110(TAPO_IP)
    if action == "on":
        await device.on()
    else:
        await device.off()


if __name__ == "__main__":

    if len(sys.argv) != 2 or sys.argv[1] not in ["on", "off", "server"]:
        print("Usage: python3 app.py [on|off]")
        sys.exit(1)

    action = sys.argv[1]
    if action == "server":
        rclpy.init()
        node = RosTapoNode()
        rclpy.spin(node)
        rclpy.shutdown()

    asyncio.run(main())
