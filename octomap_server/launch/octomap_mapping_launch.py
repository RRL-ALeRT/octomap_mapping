from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    audio_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        output='screen',
        parameters=[{
            "resolution": 0.05,
            "frame_id": "vision",
            "base_frame_id": "body",
            "sensor_model.max_range": 10.0,
            "latch": False,
        }],
        remappings=[
            # ("cloud_in", "/filtered_point_cloud"),
            ("cloud_in", "/velodyne_points"),
        ],
    )

    return LaunchDescription([audio_server])
