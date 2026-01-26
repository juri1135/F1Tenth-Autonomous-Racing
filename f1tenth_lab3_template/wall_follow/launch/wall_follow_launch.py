from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 월 팔로잉 메인 노드 (C++ 실행파일)
        Node(
            package='wall_follow',
            executable='wall_follow_node',
            name='wall_follow_node',
            output='screen',
            parameters=[{
                'kp': 0.8,
                'ki': 0.0,
                'kd': 0.1,
            }])
        # ),

        # # 2. 오차 실시간 그래프 (rqt_plot)
        # Node(
        #     package='rqt_plot',
        #     executable='rqt_plot',
        #     name='rqt_plot',
        #     arguments=['/wall_error/data'],
        # )
    ])