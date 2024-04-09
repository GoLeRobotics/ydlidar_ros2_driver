#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch.actions import LogInfo
from nav2_common.launch import RewrittenYaml

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    rviz_config_file = os.path.join(share_dir, 'config','ydlidar.rviz')

    bringup_dir = get_package_share_directory('nav2_bringup')
    node_name = 'ydlidar_ros2_driver_node'
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']
    # parameter_file = LaunchConfiguration('params_file')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    FL_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='gole_lidar_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[
                                            {"port": f"/dev/ttyLIDARFL", "frame_id": f"lidar_FL", "ignore_array": ""},
                                            {"baudrate": 512000, "lidar_type": 1, "device_type": 0, "sample_rate": 6, "abnormal_check_count": 4,
                                            "intensity_bit": 8},
                                            {"resolution_fixed": False, "auto_reconnect": True, "reversion": True, "inverted": True,
                                            "isSingleChannel": False, "intensity": False, "support_motor_dtr": False,
                                            "invalid_range_is_inf": False, "point_cloud_preservative": False},
                                            {"angle_min": -180.0, "angle_max": 180.0, "range_min": 0.1, "range_max": 15.0, "frequency": 6.0},
                                        ],
                                namespace='/FL',
                                )
    
    tf2_lidar_FL_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_lidar_FL',
                    arguments=['0.442240850568205', '0.352279461646644', '0.134869277594228','0', '0', '0.3826833568853094', '0.9238795637760319','base_link','lidar_FL'],
                    )

    rviz2_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    )

    return LaunchDescription([
        FL_node,
        tf2_lidar_FL_node,
        rviz2_node,
    ])
