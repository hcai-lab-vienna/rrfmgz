import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

def generate_launch_description():
    pkg_share = get_package_share_directory('scout_description')
    ros_gz_scout_share = get_package_share_directory('ros_gz_scout_lehrforst_sim')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch_path = os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
    default_model_path = os.path.join(pkg_share, 'urdf', 'scout_v2', 'scout_v2.urdf')
    world_path = os.path.join(ros_gz_scout_share, 'worlds', 'scout_in_lehrforst.sdf')
    bridge_config_path = os.path.join(ros_gz_scout_share, 'config', 'bridges.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['cat ', LaunchConfiguration('model')])},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        robot_state_publisher_node,
        RosGzBridge(
            bridge_name='ros_gz_bridge',
            config_file=bridge_config_path,
            container_name='ros_gz_container',
            create_own_container='False',
            use_composition='True',
        ),
        gz_server
    ])
