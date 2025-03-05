import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction, DeclareLaunchArgument

def launch_setup(context):
    compiled = os.environ['need_compile']
    use_depth_cam = LaunchConfiguration('use_depth_cam', default='true')
    use_depth_cam_arg = DeclareLaunchArgument('use_depth_cam', default_value=use_depth_cam)
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
        controller_package_path = get_package_share_directory('controller')
        kinematics_package_path = get_package_share_directory('kinematics')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        kinematics_package_path = '/home/ubuntu/ros2_ws/src/driver/kinematics'

    hand_track_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
            condition=IfCondition(use_depth_cam)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(peripherals_package_path, 'launch/usb_cam.launch.py')),
            condition=UnlessCondition(use_depth_cam)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(controller_package_path, 'launch/controller.launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(kinematics_package_path, 'launch/kinematics_node.launch.py'))),

        Node(
            package='example',
            executable='hand_detect',
            output='screen'),

        Node(
            package='example',
            executable='hand_track',
            output='screen')
    ])

    return [use_depth_cam_arg,
            hand_track_group
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
