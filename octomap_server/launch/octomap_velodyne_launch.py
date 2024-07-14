from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    node_list = []

    realsenses_filtering_container = ComposableNodeContainer(
        name='realsense_pointcloud_filter',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='rrl_launchers',
                plugin='rrl_launchers::FilteredPointCloud',
                name='filter_pcl',
                remappings=[
                    ("/velodyne_points", "/rs_front/depth/color/points"),
                    ("/filtered_velodyne_points", "/rs_front/depth/color/points_filtered"),
                ],
            ),
            ComposableNode(
                package='rrl_launchers',
                plugin='rrl_launchers::FilteredPointCloud',
                name='filter_pcl',
                remappings=[
                    ("/velodyne_points", "/rs_left/depth/color/points"),
                    ("/filtered_velodyne_points", "/rs_left/depth/color/points_filtered"),
                ],
            ),
            ComposableNode(
                package='rrl_launchers',
                plugin='rrl_launchers::FilteredPointCloud',
                name='filter_pcl',
                remappings=[
                    ("/velodyne_points", "/rs_right/depth/color/points"),
                    ("/filtered_velodyne_points", "/rs_right/depth/color/points_filtered"),
                ],
            )
        ],
        output='both',
    )
    node_list.append(realsenses_filtering_container)

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        output='screen',
        parameters=[{
            "resolution": 0.1,
            "frame_id": "map",
            "base_frame_id": "body",
            "sensor_model.max_range": 1.8,
            "latch": False,
            "exploration": True,
            "multiple_pointclouds": True,
        }],
        remappings=[
            ("cloud_in_1", "/rs_front/depth/color/points_filtered"),
            ("cloud_in_2", "/rs_left/depth/color/points_filtered"),
            ("cloud_in_3", "/rs_right/depth/color/points_filtered"),
        ],
    )
    node_list.append(octomap_server)

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
        namespace='navigation',
        output='screen',
        parameters=[{
            "resolution": 0.05,
            "frame_id": "map",
            "base_frame_id": "body",
            "sensor_model.max_range": 1.2,
            "latch": False,
            "exploration": False,
            "multiple_pointclouds": True,
        }],
        remappings=[
            ("/navigation/cloud_in_1", "/depth/frontleft/points"),
            ("/navigation/cloud_in_2", "/depth/frontright/points"),
            ("/navigation/cloud_in_3", "/depth/back/points"),
        ],
    )
    node_list.append(octomap_nav_server)

    spotfront_pointcloud = ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[('image_rect', '/depth/frontleft/image'),
                                ('camera_info', '/depth/frontleft/camera_info'),
                                ('points', '/depth/frontleft/points')]
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[('image_rect', '/depth/frontright/image'),
                                ('camera_info', '/depth/frontright/camera_info'),
                                ('points', '/depth/frontright/points')]
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[('image_rect', '/depth/back/image'),
                                ('camera_info', '/depth/back/camera_info'),
                                ('points', '/depth/back/points')]
                ),
            ],
            output='screen',
        )
    node_list.append(spotfront_pointcloud)

    return LaunchDescription(node_list)

