import os
from glob import glob
from setuptools import setup

package_name = 'example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('example', '**/*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*.dae'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detect = example.color_detect.color_detect_node:main',
            'color_track = example.color_track.color_track_node:main',
            'color_sorting = example.color_sorting.color_sorting_node:main',
            'hand_trajectory = example.hand_trajectory.hand_trajectory_node:main',
            'hand_gesture_control = example.hand_gesture_control.hand_gesture_control_node:main',
            'hand_track = example.hand_track.hand_track_node:main',
            'hand_detect = example.hand_track.hand_detect_node:main',
            'finger_control = example.hand_trajectory.finger_control:main',
            'body_and_rgb_control = example.body_control.include.body_and_rgb_control:main',
            'body_control = example.body_control.include.body_control:main',
            'body_track = example.body_control.include.body_track:main',
            'fall_down_detect = example.body_control.include.fall_down_detect:main',
            'line_follow_clean = example.line_follow_clean.line_follow_clean_node:main',
            'garbage_classification = example.garbage_classification.garbage_classification:main',
            'self_driving = example.self_driving.self_driving:main',

            'cross_bridge = example.rgbd_function.cross_bridge_node:main',
            'prevent_falling = example.rgbd_function.prevent_falling_node:main',
            'track_object = example.rgbd_function.track_object_node:main',
            'object_classification = example.rgbd_function.object_classification:main',
            'track_and_grab = example.rgbd_function.track_and_grab:main',
            'vc_object_classification = example.rgbd_function.vc_object_classification:main',
            'vc_track_and_grab = example.rgbd_function.vc_track_and_grab:main',
            'automatic_pick = example.navigation_transport.automatic_pick:main',
            'navigation_transport = example.navigation_transport.navigation_transport:main',
        ],
    },
)
