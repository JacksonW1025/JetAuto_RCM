import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']
    enable_display = LaunchConfiguration('enable_display', default='false')
    enable_display_arg = DeclareLaunchArgument('enable_display', default_value=enable_display)
    start = LaunchConfiguration('start', default='true')
    start_arg = DeclareLaunchArgument('start', default_value=start)
    use_depth_cam = LaunchConfiguration('use_depth_cam', default='true')
    use_depth_cam_arg = DeclareLaunchArgument('use_depth_cam', default_value=use_depth_cam)

    if compiled == 'True':
        controller_package_path = get_package_share_directory('controller')
        kinematics_package_path = get_package_share_directory('kinematics')
        example_package_path = get_package_share_directory('example')
    else:
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        kinematics_package_path = '/home/ubuntu/ros2_ws/src/driver/kinematics'
        example_package_path = '/home/ubuntu/ros2_ws/src/example'

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )

    kinematics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kinematics_package_path, 'launch/kinematics_node.launch.py')),
    )

    color_detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(example_package_path, 'example/color_detect/color_detect_node.launch.py')),
        launch_arguments={
            'enable_display': enable_display,
            'use_depth_cam': use_depth_cam,
        }.items()
    )

    color_track_node = Node(
        package='example',
        executable='color_track',
        output='screen',
        parameters=[{'start': start}]
    )

    return [start_arg,
            enable_display_arg,
            use_depth_cam_arg,
            controller_launch,
            kinematics_launch,
            color_detect_launch,
            color_track_node,
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
