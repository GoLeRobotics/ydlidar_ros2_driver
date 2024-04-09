# https://github.com/jdgalviss/realsense_ros2/blob/979350f8b5c1c70bea1d54182f893e8be6bc5e17/realsense_ros2/launch/cartographer_launch.py

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

import os
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    rviz_config_file = os.path.join(share_dir, 'config','ydlidar_cartographer_custom.rviz')
    # rviz_config_file = os.path.join(share_dir, 'config','ydlidar.rviz')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                    default=os.path.join(share_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='ydlidar_cartographer.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    return LaunchDescription([

        Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
            ),
        
        LifecycleNode(package='ydlidar_ros2_driver',
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
                                            {"range_min": 0.1, "range_max": 15.0, "frequency": 10.0}, #"angle_min": -180.0, "angle_max": 180.0, 
                                        ],
                                namespace='/FL',
                                ),
        Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_lidar_FL',
                    arguments=['0.442240850568205', '0.352279461646644', '0.134869277594228','0', '0', '0.3826833568853094', '0.9238795637760319','base_link','lidar_FL'],
                    ),
        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'lidar_FL']
            ),

        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            # map TF to odom TF
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
        ),

        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            # odom TF to base_footprint
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_footprint']
        ),
            
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen', #log
            # parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename],
            remappings=[
                        ("/points2", "/fusion"), # Remapping pointcloud2 topic to /fusion
                        ("/odom", "/gole/odom_sim"), # Remapping odom topic to /gole/odom_sim
                        ("/imu_link", "/imu_plugin_g"), # Remapping imu topic to gazebo_imu_pluginazebo/out
                        ("/scan","/FL/scan"),]
            ),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
    ])