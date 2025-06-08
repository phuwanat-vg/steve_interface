from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 0.0,
                'zero_altitude': True,
                'publish_filtered_gps': True,
                'use_odometry_yaw': False,
                'use_manual_datum': False,
                'broadcast_utm_transform': True,
                'broadcast_utm_transform_as_parent_frame': False,
                'world_frame': 'map',
                'base_link_frame': 'base_link',
                'odom_frame': 'odom',
                'use_sim_time': False
            }],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('gps/filtered', '/gps/filtered'),
                ('odometry/gps', '/odometry/gps')
            ]
        )
    ])
