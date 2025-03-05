import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    compiled = os.environ['need_compile']
    lidar_type = os.environ['LIDAR_TYPE']
    lidar_frame = LaunchConfiguration('lidar_frame', default='lidar_frame')
    scan_raw = LaunchConfiguration('scan_raw', default='scan_raw')
    scan_topic = LaunchConfiguration('scan_topic', default='scan')

    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value=lidar_frame)
    scan_raw_arg = DeclareLaunchArgument('scan_raw', default_value=scan_raw)
    scan_topic_arg = DeclareLaunchArgument('scan_topic', default_value=scan_topic)

    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    if lidar_type == 'LD19':
        lidar_launch_path = os.path.join(peripherals_package_path, 'launch/include/ldlidar_LD19.launch.py')
    elif lidar_type == 'A1':
        lidar_launch_path = os.path.join(peripherals_package_path, 'launch/include/sllidar_a1.launch.py')
    elif lidar_type == 'G4':
        lidar_launch_path = os.path.join(peripherals_package_path, 'launch/include/ydlidar_g4.launch.py')

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={'lidar_frame': lidar_frame,
                          'scan_raw': scan_raw}.items())

    laser_filters_config = ''
    if lidar_type == 'A1':
        laser_filters_config = os.path.join(peripherals_package_path, 'config/lidar_filters_config_a1.yaml')
    elif lidar_type == 'G4':
        laser_filters_config = os.path.join(peripherals_package_path, 'config/lidar_filters_config_g4.yaml')
    elif lidar_type == 'LD14P':
        laser_filters_config = os.path.join(peripherals_package_path, 'config/lidar_filters_config_ld14p.yaml')
    elif lidar_type == 'LD19':
        laser_filters_config = os.path.join(peripherals_package_path, 'config/lidar_filters_config_ld19.yaml')
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        output='screen',
        parameters=[laser_filters_config],
        remappings=[('scan', scan_raw),
                    ('scan_filtered', scan_topic)]
    )

    return LaunchDescription([
        lidar_frame_arg,
        scan_raw_arg,
        scan_topic_arg,
        lidar_launch,
        laser_filter_node,
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
