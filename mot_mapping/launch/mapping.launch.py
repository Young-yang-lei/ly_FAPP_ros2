from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('mot_mapping'), 'rviz', 'display_point_cloud.rviz'
            ]),
        ),
        Node(
            package='mot_mapping', executable='mapping_node', name='mapping_node',
            output='screen',
            parameters=[{
                'dbscan.core_pts': 3,
                'dbscan.tolerance': 0.02,
                'dbscan.min_cluster': 20,
                'dbscan.max_cluster': 800,
                'detection.thresh_dist': 0.04,
                'detection.thresh_var': 0.28,
                'lidar.range_x': 8.0,
                'lidar.range_y': 8.0,
                'lidar.range_z': 2.2,
                'ros.lidar_topic': '/map_generator/obj_cloud',
                'ros.odom_topic': '/drone_0_visual_slam/odom',
            }],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='lio_to_world',
            arguments=['1', '0', '0', '0', '0', '0', 'world', 'camera_init'],
        ),
        Node(
            package='rviz2', executable='rviz2', name='mapping_rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
