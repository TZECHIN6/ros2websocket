from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node


def generate_launch_description():
    websocket_sender_node = Node(
        package='ros2websocket',
        executable='ros2websocket_sender_node',
        name='ros2websocket_sender_node',
        namespace=TextSubstitution(text=''),
        parameters=[{
          'server_ip': LaunchConfiguration('server_ip'),
          'server_port': LaunchConfiguration('server_port'),
          'robot_id': LaunchConfiguration('robot_id'),
        }],
        output='screen',
      )
   
    return LaunchDescription([
        DeclareLaunchArgument('server_ip', default_value='localhost'),
        DeclareLaunchArgument('server_port', default_value='8000'),
        DeclareLaunchArgument('robot_id', default_value='CB-00'),
        websocket_sender_node,
   ])