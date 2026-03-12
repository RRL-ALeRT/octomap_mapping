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

    # rs_pointcloud = ComposableNodeContainer(
    #         name='container',
    #         namespace='',
    #         package='rclcpp_components',
    #         executable='component_container',
    #         composable_node_descriptions=[
    #             ComposableNode(
    #                 package='depth_image_proc',
    #                 plugin='depth_image_proc::PointCloudXyzrgbNode',
    #                 name='point_cloud_xyz_rgb_node_rs_front',
    #                 remappings=[('rgb/image_rect_color', '/rs_front/camera/color/image_raw'),
    #                             ('rgb/camera_info', '/rs_front/camera/color/camera_info'),
    #                             ('depth/image_rect', '/rs_front/camera/aligned_depth_to_color/image_raw'),
    #                             ('depth/camera_info', '/rs_front/camera/aligned_depth_to_color/camera_info'),
    #                             ('points', '/rs_color_points')]
    #             ),
    #             ComposableNode(
    #                 package='depth_image_proc',
    #                 plugin='depth_image_proc::PointCloudXyzrgbNode',
    #                 name='point_cloud_xyz_rgb_node_rs_left',
    #                 remappings=[('rgb/image_rect_color', '/rs_left/camera/color/image_raw'),
    #                             ('rgb/camera_info', '/rs_left/camera/color/camera_info'),
    #                             ('depth/image_rect', '/rs_left/camera/aligned_depth_to_color/image_raw'),
    #                             ('depth/camera_info', '/rs_left/camera/aligned_depth_to_color/camera_info'),
    #                             ('points', '/rs_color_points')]
    #             ),
    #             ComposableNode(
    #                 package='depth_image_proc',
    #                 plugin='depth_image_proc::PointCloudXyzrgbNode',
    #                 name='point_cloud_xyz_rgb_node_rs_right',
    #                 remappings=[('rgb/image_rect_color', '/rs_right/camera/color/image_raw'),
    #                             ('rgb/camera_info', '/rs_right/camera/color/camera_info'),
    #                             ('depth/image_rect', '/rs_right/camera/aligned_depth_to_color/image_raw'),
    #                             ('depth/camera_info', '/rs_right/camera/aligned_depth_to_color/camera_info'),
    #                             ('points', '/rs_color_points')]
    #             ),
    #         ],
    #         output='screen',
    #     )
    # node_list.append(rs_pointcloud)

    # realsenses_filtering_container = ComposableNodeContainer(
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
    #                 ("/points", "/rs_front/depth/color/points"),
    #                 ("/filtered_points", "/rs_front/depth/color/points_filtered"),
    #             ],
    #         ),
    #         ComposableNode(
    #             package='rrl_launchers',
    #             plugin='rrl_launchers::FilteredPointCloud',
    #             name='filter_pcl',
    #             remappings=[
    #                 ("/points", "/rs_left/depth/color/points"),
    #                 ("/filtered_points", "/rs_left/depth/color/points_filtered"),
    #             ],
    #         ),
    #         ComposableNode(
    #             package='rrl_launchers',
    #             plugin='rrl_launchers::FilteredPointCloud',
    #             name='filter_pcl',
    #             remappings=[
    #                 ("/points", "/rs_right/depth/color/points"),
    #                 ("/filtered_points", "/rs_right/depth/color/points_filtered"),
    #             ],
    #         )
    #     ],
    #     output='both',
    # )
    # node_list.append(realsenses_filtering_container)

    # rs_front_relay = ExecuteProcess(
    #     cmd=['ros2', 'run', 'topic_tools', 'relay', '/rs_front/depth/color/points_filtered', '/rs_combined_filtered'],
    #     output='screen'
    # )
    # node_list.append(rs_front_relay)

    # rs_left_relay = ExecuteProcess(
    #     cmd=['ros2', 'run', 'topic_tools', 'relay', '/rs_right/camera/depth/color/points', '/rs_combined_filtered'],
    #     output='screen'
    # )
    # node_list.append(rs_left_relay)

    # rs_right_relay = ExecuteProcess(
    #     cmd=['ros2', 'run', 'topic_tools', 'relay', '/rs_left/camera/depth/color/points', '/rs_combined_filtered'],
    #     output='screen'
    # )
    # node_list.append(rs_right_relay)

    octomap_server = Node(
        package='octomap_server',
        executable='color_octomap_server_node',
        output='screen',
        # namespace= 'color',
        parameters=[{
            "resolution": 0.05,
            "frame_id": "map",
            "base_frame_id": "body",
            "sensor_model.max_range": 2.0,
            "latch": False,
            "exploration": True,
            "multiple_pointclouds": False,
        }],
        remappings=[
            ("cloud_in", "/rs_color_points"),
        ],
    )
    node_list.append(octomap_server)

    return LaunchDescription(node_list)

