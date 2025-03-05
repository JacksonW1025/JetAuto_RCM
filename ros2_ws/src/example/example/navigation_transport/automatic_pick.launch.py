import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')
        example_package_path = get_package_share_directory('example')
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'
        example_package_path = '/home/ubuntu/ros2_ws/src/example'

    debug = LaunchConfiguration('debug', default='false')
    broadcast = LaunchConfiguration('broadcast', default='false')
    place_without_color = LaunchConfiguration('place_without_color', default='true')
    place_position = LaunchConfiguration('place_position', default="[1.0, 1.0, 0.0, 0.0, 0.0]")
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST'])
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER'])

    debug_arg = DeclareLaunchArgument('debug', default_value=debug)
    broadcast_arg = DeclareLaunchArgument('broadcast', default_value=broadcast)
    place_without_color_arg = DeclareLaunchArgument('place_without_color', default_value=place_without_color)
    place_position_arg = DeclareLaunchArgument('place_position', default_value=place_position)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'master_name': master_name,
            'robot_name': robot_name
        }.items(),
    )

    automatic_pick_node = Node(
        package='example',
        executable='automatic_pick',
        output='screen',
        parameters=[os.path.join(example_package_path, 'config/automatic_pick_roi.yaml'), {'debug': debug, 'broadcast': broadcast, 'place_without_color': place_without_color, 'place_position': place_position}]
    )

    return [debug_arg, 
            broadcast_arg, 
            place_without_color_arg, 
            place_position_arg,
            master_name_arg, 
            robot_name_arg, 
            base_launch, 
            automatic_pick_node
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
