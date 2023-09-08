# Copyright 2023 Luk Tze Ching.
# All Rights Reserved.

import rclpy
from rclpy.node import Node
import websockets
import asyncio
import json

from std_msgs.msg import String, Bool

class WebsocketReceiver(Node):
    def __init__(self):
        super().__init__('websocket_receiver_node')
        self.websocket_to_ros_pub = self.create_publisher(String, '/from_websocket', 10)
        self.connection_state_pub = self.create_publisher(Bool, '/charging_robot/connection_state', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.uri = 'ws://localhost:8000/ws/001'
        self.connection_state = False

    def timer_callback(self):
        msg = Bool()
        msg.data = self.connection_state
        self.connection_state_pub.publish(msg)

    async def recv_from_websocket(self, websocket):
        try:
            async for message in websocket:
                await self.websocket_to_ros_pub.publish(message)
                self.get_logger().info('Successfully receive message from server')
        except Exception as e:
            self.get_logger().error(str(e))
    
    async def connect_to_server(self):
        async for websocket in websockets.connect(self.uri):
            try:
                self.connection_state = True
                self.get_logger().info(f'Successful to connect the server {self.uri}')
                await self.recv_from_websocket(websocket)
            except websockets.ConnectionClosed:
                self.connection_state = False
                self.get_logger().error(f'Fail to connect the server {self.uri}')
                continue


async def spinning(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def run(args, loop: asyncio):
    rclpy.init(args=args)
    websocket_receiver = WebsocketReceiver()

    spin_task = loop.create_task(spinning(websocket_receiver))

    try:
        await spin_task
    except asyncio.exceptions.CancelledError:
        pass

    websocket_receiver.destroy_node()
    rclpy.shutdown()


def main(args=None):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args, loop=loop))


if __name__ == '__main__':
    main()
