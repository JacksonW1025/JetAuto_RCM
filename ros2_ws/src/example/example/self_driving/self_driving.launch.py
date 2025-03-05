import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']
    start = LaunchConfiguration('start', default='true')
    start_arg = DeclareLaunchArgument('start', default_value=start)
    only_line_follow = LaunchConfiguration('only_line_follow', default='false')
    only_line_follow_arg = DeclareLaunchArgument('only_line_follow', default_value=only_line_follow)
    if compiled == 'True':
        controller_package_path = get_package_share_directory('controller')
    else:
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )

    self_driving_node = Node(
        package='example',
        executable='self_driving',
        output='screen',
        parameters=[{'start': start}, {'only_line_follow': only_line_follow}],
    )

    return [start_arg,
            only_line_follow_arg,
            controller_launch,
            self_driving_node,
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

