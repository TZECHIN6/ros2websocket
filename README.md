# ros2websocket
This repo is to implement the WebSocket client as a ROS2 node.

## Installation
1. Git clone the project under the `src` directory of your ros2 workspace.
2. Use `rosdep` to install necessary dependencies.
3. Build the workspace with `colcon`.

## Usage
This package includes both websocket sender and receiver nodes. A launch file is prepared for launching both nodes. 
You could change the websocket server uri by changing the launch arguments. In this example, the websocket server uri is in this format: _ws://{server_ip}:{server_port}/ws/{robot_id}_.
Run below commands to launch the ros2 websocket bridge:

```
cd <ros2_ws>
source install/setup.bash
ros2 launch ros2websocket ros2websocket_bridge.launch.xml
```

To use this package to communicate using WebSocket, there are two topics created for in-bound and out-bound messages, see the table below:

| Node                    | Topic           | Interface |
|-------------------------|-----------------|-----------|
| websocket_receiver_node | /from_websocket | String    |
| websocket_sender_node   | /to_websocket   | String    |

To keep the ros2 websocket interface node simple and easy to manage, it is designed to separate with message logic handling. You could create your node to complete all the logic and data processes, 
and publish and subscribe to these two topics.

## Contributions
Any suggestions are welcomed! Feel free to start an issue. :relaxed:
