from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    node_list = []
    map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "vision", "map"],
    )
    node_list.append(map_odom)

    map_1m = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "1", "0", "0", "0", "map", "map_1m"],
    )
    node_list.append(map_1m)

    map_2m = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "2", "0", "0", "0", "map", "map_2m"],
    )
    node_list.append(map_2m)

    octomap_nav_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        # namespace='navigation',
        # namespace='navigation',
        output='screen',
        parameters=[{
            "resolution": 0.15,
            "frame_id": "map",
            "base_frame_id": "body",
            "sensor_model.max_range": 3.0,
            "sensor_model.min_range": 0.5,
            "latch": False,
            "exploration": True,
            "multiple_pointclouds": False,
            "point_cloud_max_z": 1.0,
            "occupancy_max_z": 1.0,
        }],
        remappings=[
            ("/cloud_in", "/livox/lidar"),
            # ("octomap_full", "/navigation/octomap_full"),
            # ("octomap_point_cloud_centers", "/navigation/octomap_point_cloud_centers")
        ],
    )
    node_list.append(octomap_nav_server)

    # octomap_filtering_container = ComposableNodeContainer(
    #     name='octomap_pointcloud_filter',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     namespace='',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='rrl_launchers',
    #             plugin='rrl_launchers::FilteredPointCloud',
    #             name='filter_pcl',
    #             # remappings=[
    #             #     ("/points", "/navigation/octomap_point_cloud_centers"),
    #             #     ("/filtered_points", "/navigation/octomap_point_cloud_centers_filtered"),
    #             # ],
    #         ),
    #     ],
    #     output='both',
    # )
    # node_list.append(octomap_filtering_container)

    return LaunchDescription(node_list)

