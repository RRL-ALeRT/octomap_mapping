from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        output='screen',
        parameters=[{
            "resolution": 0.1,
            "frame_id": "map",
            "base_frame_id": "body",
            "sensor_model.max_range": 3.0,
            "latch": True,
        }],
        remappings=[
            ("cloud_in", "/velodyne_points"),
            # ("cloud_in", "/filtered_velodyne_points"),
            # ("cloud_in", "/spot_pcl"),
        ],
    )

    map_1m = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "1", "0", "0", "0", "map", "map_1m"],
    )

    map_2m = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "2", "0", "0", "0", "map", "map_2m"],
    )

    return LaunchDescription([octomap_server, map_1m, map_2m])
