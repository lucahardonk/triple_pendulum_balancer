from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="odrive_can",
            executable="odrive_can_node",
            name="can_node",
            namespace="odrive_axis0",
            output="screen",
            parameters=[{
                "node_id": 0,
                "interface": "can0",
                "legacy_fw05": True,
                "poll_rate_hz": 200.0,
            }],
        ),
        Node(package="joy", executable="joy_node", output="screen"),
    ])
