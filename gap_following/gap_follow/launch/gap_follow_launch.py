from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('gap_follow'),
        'config',
        'default.yaml'   # ← 여기만 바꾸면 됨
    )

    return LaunchDescription([
        Node(
            package='gap_follow',
            executable='reactive_node',
            name='reactive_node',
            output='screen',
            parameters=[param_file]
        )
    ])
