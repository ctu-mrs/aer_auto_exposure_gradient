from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def get_processed_launch_objects(context):
    uav_name = LaunchConfiguration('uav_name').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    custom_config = LaunchConfiguration('custom_config').perform(context)

    package_params = PathJoinSubstitution([
        FindPackageShare('aer_auto_exposure_gradient'),
        'config', 'exp_node_params.yaml'
    ]).perform(context)

    parameters = [package_params]
    if custom_config != '':
        parameters.append(custom_config)

    aer_node = ComposableNode(
        package='aer_auto_exposure_gradient',
        plugin='exp_node::ExpNode',
        name='aer_node',
        namespace=uav_name,
        parameters=parameters,
        remappings=[
            ('image/in',           PathJoinSubstitution(['/', uav_name, camera_name, 'image_raw'])),
            ('expose_us/out',      PathJoinSubstitution(['/', uav_name, camera_name, 'expose_us'])),
            ('gain_db/out',        PathJoinSubstitution(['/', uav_name, camera_name, 'gain_db'])),
            ('shutter_limit/in',   PathJoinSubstitution(['/', uav_name, camera_name, 'shutter_limit'])),
            ('gradient/D',         PathJoinSubstitution(['/', uav_name, camera_name, 'gradient/D'])),
            ('gradient/D_clipped', PathJoinSubstitution(['/', uav_name, camera_name, 'gradient/D_clipped'])),
            ('gradient/gamma_est', PathJoinSubstitution(['/', uav_name, camera_name, 'gradient/gamma_est'])),
            ('plot_example',       PathJoinSubstitution(['/', uav_name, camera_name, 'plot_example'])),
            ('plot_sweep',         PathJoinSubstitution(['/', uav_name, camera_name, 'plot_sweep']))
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='autoexposure',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[aer_node],
    )

    return [container]


def generate_launch_description():
    declare_uav_name = DeclareLaunchArgument(
        'uav_name',
        default_value=EnvironmentVariable('UAV_NAME'),
        description='UAV namespace'
    )

    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='',
        description='Camera name used in topic remapping'
    )

    declare_custom_config = DeclareLaunchArgument(
        'custom_config',
        default_value='',
        description='Path to a custom parameter yaml file (overrides package defaults)'
    )

    return LaunchDescription([
        declare_uav_name,
        declare_camera_name,
        declare_custom_config,
        OpaqueFunction(function=get_processed_launch_objects),
    ])
