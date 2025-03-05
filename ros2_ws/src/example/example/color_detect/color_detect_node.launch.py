import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, GroupAction

def launch_setup(context):
    compiled = os.environ['need_compile']
    enable_display = LaunchConfiguration('enable_display', default='false')
    enable_display_arg = DeclareLaunchArgument('enable_display', default_value=enable_display)
    use_depth_cam = LaunchConfiguration('use_depth_cam', default='true')
    use_depth_cam_arg = DeclareLaunchArgument('use_depth_cam', default_value=use_depth_cam)

    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
        example_package_path = get_package_share_directory('example')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        example_package_path = '/home/ubuntu/ros2_ws/src/example'

    color_detect_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
            condition=IfCondition(use_depth_cam)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(peripherals_package_path, 'launch/usb_cam.launch.py')),
            condition=UnlessCondition(use_depth_cam)),

        Node(
            package='example',
            executable='color_detect',
            output='screen',
            parameters=[os.path.join(example_package_path, 'config/roi.yaml'), {'enable_display': enable_display}]),
    ])

    return [enable_display_arg,
            use_depth_cam_arg,
            color_detect_group,
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
