from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    max_linear = LaunchConfiguration('max_linear', default='0.5')
    max_angular = LaunchConfiguration('max_angular', default='2.0')
    remap_cmd_vel = LaunchConfiguration('remap_cmd_vel', default='controller/cmd_vel')

    max_linear_arg = DeclareLaunchArgument('max_linear', default_value=max_linear)
    max_angular_arg = DeclareLaunchArgument('max_angular', default_value=max_angular)
    remap_cmd_vel_arg = DeclareLaunchArgument('remap_cmd_vel', default_value=remap_cmd_vel)

    joystick_control_node = Node(
        package='peripherals',
        executable='joystick_control',
        name='joystick_control',
        output='screen',
        parameters=[
            {'max_linear': max_linear,
            'max_angular': max_angular,
            'disable_servo_control': True}
        ],
        remappings=[('controller/cmd_vel', remap_cmd_vel)]
    )

    return LaunchDescription([
        max_linear_arg,
        max_angular_arg,
        remap_cmd_vel_arg,
        joystick_control_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
