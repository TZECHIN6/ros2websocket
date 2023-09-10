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

        self.connection_state = False

    def timer_callback(self):
        msg = Bool()
        msg.data = self.connection_state
        self.connection_state_pub.publish(msg)

    async def recv_from_websocket(self, websocket):
        async for message in websocket:
            if message is None:
                self.get_logger().warning(f'Received message is NoneType')
                continue
            if not message:
                self.get_logger().warning(f'Received empty message')
                continue
            string_msg = String()
            string_msg.data = message
            print(string_msg)
            self.websocket_to_ros_pub.publish(string_msg)
            self.get_logger().info('Successfully receive message from server')


async def connect_to_server(node: WebsocketReceiver):
    async for websocket in websockets.connect(node.uri):
        try:
            node.connection_state = True
            node.get_logger().info(f'Successful to connect the server {node.uri}')
            await node.recv_from_websocket(websocket)
        except websockets.ConnectionClosed:
            node.connection_state = False
            node.get_logger().error(f'Fail to connect the server {node.uri}')
            continue


async def spinning(node: WebsocketReceiver):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def run(args, loop: asyncio):
    rclpy.init(args=args)
    websocket_receiver = WebsocketReceiver()

    spin_task = loop.create_task(spinning(websocket_receiver))
    connect_task = loop.create_task(connect_to_server(websocket_receiver))
    try:
        await asyncio.gather(spin_task, connect_task)
    except asyncio.exceptions.CancelledError:
        pass

    websocket_receiver.destroy_node()
    rclpy.shutdown()


def main(args=None):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args, loop=loop))


if __name__ == '__main__':
    main()
