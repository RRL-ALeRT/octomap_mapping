from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
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
                    ("/velodyne_points", "/realsenses_combined"),
                    ("/filtered_velodyne_points", "/realsenses_combined_filtered"),
                ],
            )
        ],
        output='both',
    )

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
        }],
        remappings=[
            ("cloud_in", "/realsenses_combined_filtered"),
            # ("cloud_in", "/velodyne_points"),
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

    relay_rs_front = ExecuteProcess(
                        cmd=['ros2', 'run', 'topic_tools', 'relay', '/rs_front/depth/color/points', '/realsenses_combined'],
                        output='screen'
                    )

    relay_rs_left = ExecuteProcess(
                        cmd=['ros2', 'run', 'topic_tools', 'relay', '/rs_left/depth/color/points', '/realsenses_combined'],
                        output='screen'
                    )

    relay_rs_right = ExecuteProcess(
                        cmd=['ros2', 'run', 'topic_tools', 'relay', '/rs_right/depth/color/points', '/realsenses_combined'],
                        output='screen'
                    )

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
        }],
        remappings=[
            ("/navigation/cloud_in", "/spotfront_pointcloud"),
        ],
    )

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
                                ('points', '/spotfront_pointcloud')]
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[('image_rect', '/depth/frontright/image'),
                                ('camera_info', '/depth/frontright/camera_info'),
                                ('points', '/spotfront_pointcloud')]
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[('image_rect', '/depth/back/image'),
                                ('camera_info', '/depth/back/camera_info'),
                                ('points', '/spotfront_pointcloud')]
                ),
            ],
            output='screen',
        )

    # spotfront_filtering_container = ComposableNodeContainer(
    #     name='realsense_pointcloud_filter',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     namespace='',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='rrl_launchers',
    #             plugin='rrl_launchers::FilteredPointCloud',
    #             name='filter_pcl',
    #             remappings=[
    #                 ("/velodyne_points", "/navigation/octomap_point_cloud_centers"),
    #                 ("/filtered_velodyne_points", "/navigation/octomap_point_cloud_centers_filtered"),
    #             ],
    #         )
    #     ],
    #     output='both',
    # )

    return LaunchDescription(
        [   realsenses_filtering_container,
            octomap_server,
            map_1m,
            map_2m,
            relay_rs_front,
            relay_rs_left,
            relay_rs_right,
            octomap_nav_server,
            spotfront_pointcloud,
            # spotfront_filtering_container,
        ]
    )

