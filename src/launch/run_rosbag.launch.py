import os
import sys
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('rosbag_vision'), 'launch'))

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    def get_rosbag_player_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='rosbag_player_node',
            parameters=[ {'rosbag_file':
                '/home/xu/视频/record/dart_auto_record/record_2025_4_10_10_52_29/meimiaozhunjidi2.db3'
                }],
            extra_arguments=[{'use_intra_process_comms': True}]
        )   

    # 创建节点描述
    ros_bag_player_node = get_rosbag_player_node('rosbag_player', 'RosbagPlayer')

    # 创建 LaunchDescription
    ld = LaunchDescription()
    ld.add_action(ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ros_bag_player_node
        ],
        output='screen'
    ))

    return ld