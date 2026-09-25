# Copyright 2026
# SPDX-License-Identifier: GPL-3.0-or-later
"""
lidar_localization_bringup.launch.py

Starts lidar_localization_ros2 (NDT/GICP against a pre-built, ENU-aligned
GLIM map) as a second absolute-pose source for fusion_graph, alongside
fusion_graph's own navigation.launch.py — NOT replacing it, and NOT using
lidar_localization_ros2's own nav2_navigation.launch.py (that launch brings
up a competing Nav2 stack + expects to own TF; you already have Nav2 wired
to fusion_graph and want to keep it that way).

IMPORTANT — TF ownership. fusion_graph_node publishes BOTH map->odom and
odom->base_footprint itself (see navigation.launch.py's header comment).
lidar_localization_ros2 must therefore run WITHOUT publishing any TF: only
/pcl_pose (geometry_msgs/PoseWithCovarianceStamped) is consumed, by
fusion_graph's new OnLidarPose callback (fusion_graph_node_lidar_primary.cpp).
Check lidar_localization_ros2's current parameter name for this (it has moved
around release to release — grep their param file for "broadcast_tf" /
"publish_tf" / "publish_map_odom") and set it false. If you can't find such a
flag, the safe alternative is to let it publish and then simply not include a
static/dynamic transform consumer for it (a competing broadcaster on the same
map->odom edge is still a bug waiting to happen — worth fixing upstream-side
before relying on this in the field).

Usage: include this from your own bringup (or ros2 launch it standalone next
to navigation.launch.py) with the launch arguments below.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    map_path = LaunchConfiguration("map_path")
    cloud_topic = LaunchConfiguration("cloud_topic")
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                description=(
                    "Path to the ENU-aligned map (see tools/align_glim_map.py). "
                    "lidar_localization_ros2 accepts .pcd or .ply directly."
                ),
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/velodyne_points",
                description="3D pointcloud topic from your VLP16 driver.",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description=(
                    "MUST match fusion_graph's map_frame parameter (default 'map') — "
                    "OnLidarPose drops any /pcl_pose whose frame_id differs."
                ),
            ),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # lidar_localization_ros2's own node. Package/executable/parameter
            # names below follow the project's nav2_lidar_localization.launch.py
            # at the time this was written (2026-09) — diff against your checked-
            # out version before relying on this, their launch API has been
            # actively evolving (quickstart.py, profiles, deskew, etc.).
            Node(
                package="lidar_localization_ros2",
                executable="lidar_localization_node",
                name="lidar_localization_node",
                output="screen",
                parameters=[
                    {
                        "map_path": map_path,
                        "use_pcd_map": True,
                        "map_frame": map_frame,  # note: NOT necessarily the same
                        "base_frame": base_frame,  # param name in every release —
                        "cloud_topic": cloud_topic,  # verify against your checkout
                        "use_sim_time": use_sim_time,
                        "registration_method": "NDT_OMP",
                        # See the module docstring: this must be false so
                        # fusion_graph remains the sole TF publisher.
                        "publish_tf": False,
                    }
                ],
                remappings=[
                    ("cloud", cloud_topic),
                ],
            ),
        ]
    )
