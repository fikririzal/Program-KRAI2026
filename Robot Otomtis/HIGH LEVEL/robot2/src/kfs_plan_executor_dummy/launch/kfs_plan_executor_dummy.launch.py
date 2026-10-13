from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kfs_plan_executor_dummy",
            executable="kfs_plan_executor_dummy_node",
            name="kfs_plan_executor_dummy",
            output="screen",
            emulate_tty=True,
        )
    ])
