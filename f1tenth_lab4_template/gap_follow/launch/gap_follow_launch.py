from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gap_follow',
            executable='reactive_node',
            name='reactive_node',
            output='screen'
           )
        # ),

        # # 2. 오차 실시간 그래프 (rqt_plot)
        # Node(
        #     package='rqt_plot',
        #     executable='rqt_plot',
        #     name='rqt_plot',
        #     arguments=['/wall_error/data'],
        # )
    ])