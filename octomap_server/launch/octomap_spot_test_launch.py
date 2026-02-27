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
        namespace='navigation',
        output='screen',
        parameters=[{
            "resolution": 0.05,
            "frame_id": "map",
            "base_frame_id": "body",
            "sensor_model.max_range": 2.5,
            "latch": False,
            "exploration": False,
            "multiple_pointclouds": False,
        }],
        remappings=[
            ("/navigation/cloud_in", "/spot_depth_points"),
        ],
    )
    node_list.append(octomap_nav_server)

    spot_pointcloud = ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[('image_rect', '/depth_registered/frontleft/image'),
                                ('camera_info', '/depth_registered/frontleft/camera_info'),
                                ('points', '/spot_depth_points')]
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[('image_rect', '/depth_registered/frontright/image'),
                                ('camera_info', '/depth_registered/frontright/camera_info'),
                                ('points', '/spot_depth_points')]
                ),
                # ComposableNode(
                #     package='depth_image_proc',
                #     plugin='depth_image_proc::PointCloudXyzNode',
                #     name='point_cloud_xyz_node',
                #     remappings=[('image_rect', '/depth_registered/left/image'),
                #                 ('camera_info', '/depth_registered/left/camera_info'),
                #                 ('points', '/spot_depth_points')]
                # ),
                # ComposableNode(
                #     package='depth_image_proc',
                #     plugin='depth_image_proc::PointCloudXyzNode',
                #     name='point_cloud_xyz_node',
                #     remappings=[('image_rect', '/depth_registered/right/image'),
                #                 ('camera_info', '/depth_registered/right/camera_info'),
                #                 ('points', '/spot_depth_points')]
                # ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[('image_rect', '/depth_registered/back/image'),
                                ('camera_info', '/depth_registered/back/camera_info'),
                                ('points', '/spot_depth_points')]
                ),
                
            ],
            output='screen',
        )
    node_list.append(spot_pointcloud)

    octomap_filtering_container = ComposableNodeContainer(
        name='octomap_pointcloud_filter',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='rrl_launchers',
                plugin='rrl_launchers::FilteredPointCloud',
                name='filter_pcl',
                remappings=[
                    ("/points", "/navigation/octomap_point_cloud_centers"),
                    ("/filtered_points", "/navigation/octomap_point_cloud_centers_filtered"),
                ],
            ),
        ],
        output='both',
    )
    node_list.append(octomap_filtering_container)

    return LaunchDescription(node_list)

