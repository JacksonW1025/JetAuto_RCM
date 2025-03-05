from launch import LaunchDescription, LaunchService
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch.actions import GroupAction, OpaqueFunction, DeclareLaunchArgument

def launch_setup(context):
    # Declare arguments
    tf_prefix = LaunchConfiguration('tf_prefix', default='').perform(context)
    camera_name = LaunchConfiguration('camera_name', default='depth_cam').perform(context)
    args = [
            DeclareLaunchArgument('enable_depth', default_value='true'),
            DeclareLaunchArgument('camera_name', default_value=camera_name),
            DeclareLaunchArgument('tf_prefix', default_value=tf_prefix),
            DeclareLaunchArgument('depth_registration', default_value='false'),
            DeclareLaunchArgument('serial_number', default_value=''),
            DeclareLaunchArgument('usb_port', default_value=''),
            DeclareLaunchArgument('device_num', default_value='1'),
            DeclareLaunchArgument('vendor_id', default_value='0x2bc5'),
            DeclareLaunchArgument('product_id', default_value=''),
            DeclareLaunchArgument('enable_point_cloud', default_value='true'), 
            DeclareLaunchArgument('enable_colored_point_cloud', default_value='false'),
            DeclareLaunchArgument('point_cloud_qos', default_value='default'),
            DeclareLaunchArgument('connection_delay', default_value='100'),
            DeclareLaunchArgument('color_width', default_value='640'),
            DeclareLaunchArgument('color_height', default_value='360'),
            DeclareLaunchArgument('color_fps', default_value='30'),
            DeclareLaunchArgument('color_format', default_value='MJPG'),
            DeclareLaunchArgument('enable_color', default_value='true'),
            DeclareLaunchArgument('flip_color', default_value='false'),
            DeclareLaunchArgument('color_qos', default_value='default'),
            DeclareLaunchArgument('color_camera_info_qos', default_value='default'),
            DeclareLaunchArgument('enable_color_auto_exposure', default_value='true'),
            DeclareLaunchArgument('depth_width', default_value='640'),
            DeclareLaunchArgument('depth_height', default_value='360'),
            DeclareLaunchArgument('depth_fps', default_value='30'),
            DeclareLaunchArgument('depth_format', default_value='Y11'),
            DeclareLaunchArgument('enable_depth', default_value='true'),
            DeclareLaunchArgument('flip_depth', default_value='false'),
            DeclareLaunchArgument('depth_qos', default_value='default'),
            DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),
            DeclareLaunchArgument('ir_width', default_value='640'),
            DeclareLaunchArgument('ir_height', default_value='480'),
            DeclareLaunchArgument('ir_fps', default_value='30'),
            DeclareLaunchArgument('ir_format', default_value='Y10'),
            DeclareLaunchArgument('enable_ir', default_value='true'), 
            DeclareLaunchArgument('flip_ir', default_value='false'),
            DeclareLaunchArgument('ir_qos', default_value='default'),
            DeclareLaunchArgument('ir_camera_info_qos', default_value='default'),
            DeclareLaunchArgument('enable_ir_auto_exposure', default_value='true'),
            DeclareLaunchArgument('publish_tf', default_value='true'),
            DeclareLaunchArgument('tf_publish_rate', default_value='10.0'),
            DeclareLaunchArgument('ir_info_url', default_value=''),
            DeclareLaunchArgument('color_info_url', default_value=''),
            DeclareLaunchArgument('log_level', default_value='none'),
            DeclareLaunchArgument('enable_publish_extrinsic', default_value='false'),
            DeclareLaunchArgument('enable_d2c_viewer', default_value='false'),
            DeclareLaunchArgument('enable_ldp', default_value='true'),
            DeclareLaunchArgument('enable_soft_filter', default_value='true'),
            DeclareLaunchArgument('soft_filter_max_diff', default_value='8'),
            DeclareLaunchArgument('soft_filter_speckle_size', default_value='100'),
            DeclareLaunchArgument('ordered_pc', default_value='false'),
            DeclareLaunchArgument('enable_frame_sync', default_value='false'),
            DeclareLaunchArgument('use_hardware_time', default_value='false'),
            DeclareLaunchArgument('enable_depth_scale', default_value='true'),
            DeclareLaunchArgument('align_mode', default_value='HW'),
    ]

    # Node configuration
    parameters = [{arg.name: LaunchConfiguration(arg.name)} for arg in args]
    # Define the ComposableNode
    compose_node = ComposableNode(
        package='orbbec_camera',
        plugin='orbbec_camera::OBCameraNodeDriver',
        name=camera_name,
        namespace='',
        parameters=parameters,
        remappings=[('/tf', '/' + tf_prefix + 'tf'), 
                    ('/tf_static', '/' + tf_prefix + 'tf_static'),
                    ('/' + camera_name + '/color/camera_info', '/' + camera_name + '/rgb/camera_info'),
                    ('/' + camera_name + '/color/image_raw', '/' + camera_name + '/rgb/image_raw'),
                    ('/' + camera_name + '/color/image_raw/compressed', '/' + camera_name + '/rgb/image_raw/compressed'),
                    ('/' + camera_name + '/color/image_raw/compressedDepth', '/' + camera_name + '/rgb/image_raw/compressedDepth'),
                    ('/' + camera_name + '/depth/color/points', '/' + camera_name + '/depth_registered/points'),
                    ]
    )
    # Define the ComposableNodeContainer
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            compose_node,
        ],
        output='screen',
    )

    action_node = GroupAction(
            actions=[
                PushRosNamespace(camera_name),
                container
            ])
    return args + [action_node]

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
