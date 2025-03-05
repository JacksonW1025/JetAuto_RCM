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
        navigation_package_path = get_package_share_directory('navigation')
        xf_mic_asr_offline_package_path = get_package_share_directory('xf_mic_asr_offline')
    else:
        navigation_package_path = '/home/ubuntu/ros2_ws/src/navigation'
        xf_mic_asr_offline_package_path = '/home/ubuntu/ros2_ws/src/xf_mic_asr_offline'

    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST']).perform(context)
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER']).perform(context)
    map_name = LaunchConfiguration('map_name', default='map_01')
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)
    map_name_arg = DeclareLaunchArgument('map_name', default_value=map_name)

    topic_prefix = '' if robot_name == '/' else '/%s'%robot_name
    frame_prefix = '' if robot_name == '/' else '%s/'%robot_name
    map_frame = '{}map'.format(frame_prefix) if robot_name == master_name else '{}/map'.format(master_name)
    cmd_vel = '{}/controller/cmd_vel'.format(topic_prefix)
    goal = '{}/goal'.format(topic_prefix)
    cosmap = '{}/local_costmap/costmap'.format(topic_prefix)

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/navigation.launch.py')),
        launch_arguments={
            'map': map_name,
            'master_name': master_name,
            'robot_name': robot_name
        }.items(),
    )

    mic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xf_mic_asr_offline_package_path, 'launch/mic_init.launch.py')),
    )

    voice_control_navigation_node = Node(
        package='xf_mic_asr_offline',
        executable='voice_control_navigation.py',
        output='screen',
        parameters=[{
            'map_frame': map_frame,
            'costmap': cosmap,
            'cmd_vel': cmd_vel,
            'goal': goal,
        }]
    )

    return [master_name_arg,
            robot_name_arg,
            map_name_arg,
            navigation_launch,
            mic_launch,
            voice_control_navigation_node,
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
