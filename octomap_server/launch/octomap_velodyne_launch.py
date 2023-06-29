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

    return LaunchDescription([octomap_server])
