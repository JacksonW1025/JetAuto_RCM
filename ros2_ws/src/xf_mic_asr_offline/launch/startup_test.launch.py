import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    compiled = os.environ['need_compile']

    if compiled == 'True':
        xf_mic_asr_offline_package_path = get_package_share_directory('xf_mic_asr_offline')
    else:
        xf_mic_asr_offline_package_path = '/home/ubuntu/ros2_ws/src/xf_mic_asr_offline'

    mic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xf_mic_asr_offline_package_path, 'launch/mic_init.launch.py')),
    )

    voice_control_move_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xf_mic_asr_offline_package_path, 'launch/voice_control_move.launch.py')),
    )

    return [mic_launch, voice_control_move_launch]

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
