from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    node_list = []

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        output='screen',
        parameters=[{
            "resolution": 0.1,
            "frame_id": "map",
            "base_frame_id": "base_link",
            "sensor_model.max_range": 3.0,
            "filter_ground" : True,
            "latch": False,
            "exploration": True,
            "multiple_pointclouds": True,
            "use_sim_time": True,
        }],
        remappings=[
            ("cloud_in_1", "/Spot/left_head_depth/point_cloud"),
            ("cloud_in_2", "/Spot/right_head_depth/point_cloud"),
            ("cloud_in_3", "/Spot/rear_depth/point_cloud"),
            ("cloud_in_4", "/Spot/left_flank_depth/point_cloud"),
            ("cloud_in_5", "/Spot/right_flank_depth/point_cloud"),
            ("cloud_in_6", "/Spot/Velodyne_Puck/point_cloud")
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

    map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )
    node_list.append(map_odom)

    lhd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "left head depth", "left_head_depth"],
    )
    node_list.append(lhd)

    rhd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "right head depth", "right_head_depth"],
    )
    node_list.append(rhd)

    rd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "rear depth", "rear_depth"],
    )
    node_list.append(rd)

    rfd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "right flank depth", "right_flank_depth"],
    )
    node_list.append(rfd)

    lfd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "left flank depth", "left_flank_depth"],
    )
    node_list.append(lfd)

    body_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "body"],
    )
    node_list.append(body_base_link)

    return LaunchDescription(node_list)
