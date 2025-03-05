import os
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    compiled = os.environ['need_compile']
    base_frame = LaunchConfiguration('base_frame', default='')
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value=base_frame)

    if compiled == 'True':
        servo_controller_package_path = get_package_share_directory('servo_controller')
    else:
        servo_controller_package_path = '/home/ubuntu/ros2_ws/src/driver/servo_controller'

    servo_controller_node = Node(
        package='servo_controller',
        executable='servo_controller',
        output='screen',
        parameters=[os.path.join(servo_controller_package_path, 'config/servo_controller.yaml'), {'base_frame': base_frame}]
    )

    return LaunchDescription([
        base_frame_arg,
        servo_controller_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
