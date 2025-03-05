import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction

def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        description_package_path = get_package_share_directory('jetauto_description')
        slam_package_path = get_package_share_directory('slam')
        navigation_package_path = get_package_share_directory('navigation')
        example_package_path = get_package_share_directory('example')
    else:
        description_package_path = '/home/ubuntu/ros2_ws/src/simulations/jetauto_description'
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'
        navigation_package_path = '/home/ubuntu/ros2_ws/src/navigation'
        example_package_path = '/home/ubuntu/ros2_ws/src/example'

    map_name = LaunchConfiguration('map', default='map_01').perform(context)
    debug = LaunchConfiguration('debug', default='false')
    broadcast = LaunchConfiguration('broadcast', default='false')
    place_without_color = LaunchConfiguration('place_without_color', default='true')
    place_position = LaunchConfiguration('place_position', default="[1.0, 1.0, 0.0, 0.0, 0.0]")
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST'])
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER'])

    map_name_arg = DeclareLaunchArgument('map', default_value=map_name)
    debug_arg = DeclareLaunchArgument('debug', default_value=debug)
    broadcast_arg = DeclareLaunchArgument('broadcast', default_value=broadcast)
    place_without_color_arg = DeclareLaunchArgument('place_without_color', default_value=place_without_color)
    place_position_arg = DeclareLaunchArgument('place_position', default_value=place_position)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    automatic_pick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(example_package_path, 'example/navigation_transport/automatic_pick.launch.py')),
        launch_arguments={
            'broadcast': broadcast,
            'debug': debug,
            'place_without_color': place_without_color,
            'place_position': place_position,
            'master_name': master_name,
            'robot_name': robot_name
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_package_path, 'launch/include/bringup.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'map': os.path.join(slam_package_path, 'maps', map_name + '.yaml'),
            'params_file': os.path.join(navigation_package_path, 'config', 'nav2_params.yaml'),
            'namespace': robot_name,
            'use_namespace': 'false',
            'autostart': 'true',
        }.items(),
    )

    navigation_transport_node = Node(
        package='example',
        executable='navigation_transport',
        output='screen',
        parameters=[{'map_frame': 'map', 'nav_goal': '/nav_goal'}]
    )

    rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description_package_path, 'launch/rviz.launch.py')),
            launch_arguments={
                              'namespace': '',
                              'use_namespace': 'false',
                              'rviz_config': LaunchConfiguration('rviz', default=os.path.join(navigation_package_path, 'rviz/navigation_transport.rviz')).perform(context)}.items())

    bringup_launch = GroupAction(
     actions=[
         PushRosNamespace(robot_name),
         automatic_pick_launch,
         TimerAction(
             period=10.0,  # 延时等待其它节点启动好
             actions=[navigation_launch],
         ),
      ]
    )

    return [map_name_arg, 
            debug_arg, 
            broadcast_arg, 
            place_without_color_arg, 
            place_position_arg,
            master_name_arg,
            robot_name_arg,
            bringup_launch, 
            navigation_transport_node,
            # rviz_launch
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
