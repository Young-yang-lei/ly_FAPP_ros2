from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag = LaunchConfiguration('bag')
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='ROS 2 bag directory to play'),
        ExecuteProcess(cmd=['ros2', 'bag', 'play', bag, '--clock'], output='screen'),
        Node(
            package='mot_mapping', executable='mapping_node', name='mapping_node',
            output='screen', parameters=[{
                'use_sim_time': True,
                'dbscan.core_pts': 5,
                'dbscan.tolerance': 0.02,
                'dbscan.min_cluster': 10,
                'dbscan.max_cluster': 800,
                'detection.thresh_dist': 0.06,
                'detection.thresh_var': 0.28,
                'lidar.range_x': 15.0,
                'lidar.range_y': 15.0,
                'lidar.range_z': 5.0,
                'ros.lidar_topic': '/map_generator/obj_cloud',
                'ros.odom_topic': '/drone_0_visual_slam/odom',
            }],
        ),
    ])
