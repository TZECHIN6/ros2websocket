# Copyright 2023 Luk Tze Ching.
# All Rights Reserved.

import rclpy
from rclpy.node import Node
import websockets
import asyncio
import json

from std_msgs.msg import String

class WebsocketSender(Node):
    def __init__(self):
        super().__init__('websocket_sender_node')
        self.subscription = self.create_subscription(
            String, '/to_websocket', self.sub_callback, 10)
        
        self.declare_parameter('server_ip', 'localhost')
        self.declare_parameter('server_port', 8000)
        self.declare_parameter('robot_id', rclpy.Parameter.Type.STRING)

        server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.get_logger().info(f'websocket ip: {server_ip}')
        self.get_logger().info(f'websocket port: {server_port}')
        self.get_logger().info(f'websocket client id: {robot_id}')
        self.uri = f'ws://{server_ip}:{server_port}/ws/{robot_id}'
        self.get_logger().info(f'websocket uri: {self.uri}')

    """
    This method might implement into another node,
    thus the message send to /to_websocket already
    serialized.
    """
    # async def serialize_data(self, data):
    #     try:
    #         return json.dumps(data)
    #     except Exception as e:
    #         self.get_logger().error(e)
    #         return None

    async def sub_callback(self, msg: String):
        payload = msg.data
        if payload is not None:
            asyncio.create_task(self.send_to_websocket(payload))

    async def send_to_websocket(self, payload):
        try:
            async with websockets.connect(self.uri) as websocket:
                await websocket.send(payload)
                self.get_logger().info('Successfully sent message to server')
                await asyncio.sleep(0)
        except Exception as e:
            self.get_logger().error(str(e))


async def spinning(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def run(args, loop: asyncio):
    rclpy.init(args=args)
    websocket_sender = WebsocketSender()

    spin_task = loop.create_task(spinning(websocket_sender))

    try:
        await spin_task
    except asyncio.exceptions.CancelledError:
        pass

    websocket_sender.destroy_node()
    rclpy.shutdown()


def main(args=None):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args, loop=loop))


if __name__ == '__main__':
    main()
