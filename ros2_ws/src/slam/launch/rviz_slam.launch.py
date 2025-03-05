import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription

def launch_setup(context):
    compiled = os.environ['need_compile']
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST']).perform(context)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    if compiled == 'True':
        description_package_path = get_package_share_directory('jetauto_description')
        slam_package_path = get_package_share_directory('slam')
    else:
        description_package_path = '/home/ubuntu/ros2_ws/src/simulations/jetauto_description'
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'

    if robot_name == '/':
        namespace = ''
        use_namespace = 'false'
    else:
        namespace = robot_name
        use_namespace = 'true'
    
    rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description_package_path, 'launch/rviz.launch.py')),
            launch_arguments={
                              'namespace': namespace,
                              'use_namespace': use_namespace,
                              'rviz_config': os.path.join(slam_package_path, 'rviz/slam.rviz')}.items())


    bringup_launch = GroupAction(
     actions=[
         PushRosNamespace(robot_name),
         rviz_launch,
      ]
    )

    return [robot_name_arg, bringup_launch]

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
