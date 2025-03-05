import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']

    if compiled == 'True':
        example_package_path = get_package_share_directory('example')
        xf_mic_asr_offline_package_path = get_package_share_directory('xf_mic_asr_offline')
    else:
        example_package_path = '/home/ubuntu/ros2_ws/src/example'
        xf_mic_asr_offline_package_path = '/home/ubuntu/ros2_ws/src/xf_mic_asr_offline'

    map_name = LaunchConfiguration('map_name', default='map_01')
    map_name_arg = DeclareLaunchArgument('map_name', default_value=map_name)

    navigation_transport_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(example_package_path, 'example/navigation_transport/navigation_transport.launch.py')),
        launch_arguments={
            'map': map_name,
            'broadcast': 'true',
            'place_position': "[0.0, 0.5, 0.0, 0.0, 0.0]",
        }.items(),
    )

    mic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xf_mic_asr_offline_package_path, 'launch/mic_init.launch.py')),
    )

    voice_control_navigation_transport_node = Node(
        package='xf_mic_asr_offline',
        executable='voice_control_navigation_transport.py',
        output='screen',
        parameters=[{
            'pick_position': [1.5, 0, 0.0, 0.0, 0.0],
        }]
    )

    return [map_name_arg,
            navigation_transport_launch,
            mic_launch,
            voice_control_navigation_transport_node,
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
