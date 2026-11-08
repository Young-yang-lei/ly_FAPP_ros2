from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mot_mapping', executable='mapping_node', name='mapping_node',
            output='screen',
            parameters=[{
                'dbscan.core_pts': 3,
                'dbscan.tolerance': 0.4,
                'dbscan.min_cluster': 15,
                'dbscan.max_cluster': 1000,
                'detection.thresh_dist': 0.02,
                'detection.thresh_var': 0.8,
                'lidar.range_x': 15.0,
                'lidar.range_y': 15.0,
                'lidar.range_z': 5.0,
                'ros.lidar_topic': '/map_generator/obj_cloud',
                'ros.odom_topic': '/drone_0_visual_slam/odom',
            }],
        ),
    ])
