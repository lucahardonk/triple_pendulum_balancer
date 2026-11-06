import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mux_config = os.path.join(
        get_package_share_directory("pendulum_safety_package"), "config", "control_mux.yaml"
    )

    return LaunchDescription([
        # Driver ODrive su CAN
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
        # Unico nodo che pubblica su /odrive_axis0/control_message.
        # Priorita': safety > joystick > controller (config/control_mux.yaml)
        Node(
            package="pendulum_safety_package",
            executable="control_mux",
            output="screen",
            parameters=[mux_config],
        ),
        # Calibrazione corsa + barriere virtuali, comanda il motore su /control/safety
        Node(
            package="pendulum_safety_package",
            executable="overrun_node",
            output="screen",
        ),
        Node(package="joy", executable="joy_node", output="screen"),
    ])
