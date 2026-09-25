# Copyright 2026
# SPDX-License-Identifier: GPL-3.0-or-later
"""
pointcloud_to_laserscan.launch.py

Slices the VLP16's 3D pointcloud into a 2D LaserScan so the EXISTING, already
field-tuned Nav2 costmap/collision pipeline in
mowgli_bringup/config/nav2_params_lidar.yaml keeps working unchanged.

That pipeline (obstacle_layer via costmap_scan_filter_node's /scan_costmap,
collision_monitor via /scan_collision) was built around the original 2D
LD19, which publishes sensor_msgs/LaserScan natively. A 3D VLP16 does not —
this bridge is the missing piece, nothing else in that pipeline needs to
change. It is independent of localization: this feeds obstacle
avoidance, not fusion_graph's pose estimate. See
fusion_graph_node_lidar_primary.cpp / lidar_localization_bringup.launch.py
for the (separate) localization path.

Uses the standard ros-perception/pointcloud_to_laserscan package
(pointcloud_to_laserscan_node), taking a HEIGHT BAND across all 16 rings
rather than a single ring (contrast: velodyne_laserscan, which the VLP16
driver stack may already provide — it picks one ring, defaulting to the one
"closest to horizontal", and is documented to misbehave on other rings —
ros-drivers/velodyne#192). On a mower whose VLP16 sits ~0.40 m up, a single
tilted ring can miss short obstacles entirely on level ground; a height band
across multiple rings catches them at more ranges. Deliberate trade-off, not
a strict upgrade — see the two caveats below before trusting this in the
field.

── CAVEAT 1 (fixed by this version) — filter height must be GROUND-relative ──
target_frame is set to base_frame (see the base_frame argument), not left
empty. Filtering happens in the cloud's OWN frame when target_frame is empty
— at a 0.40 m sensor height that would filter "min/max metres above the
LIDAR", not above the ground, which is not what you want. With
target_frame=base_frame, pointcloud_to_laserscan transforms every point
through the (already-calibrated, rigid) lidar->base_frame TF first, so
min_height/max_height are metres above the ground regardless of mount
height or angle. This requires that static TF to be correct — you need it
anyway for lidar_localization_ros2's pose output to be correct, so it
should already exist.

── CAVEAT 2 (NOT fixed by any conversion method — physical limit) ──────────
VLP-16 vertical FOV is only ±15° (Velodyne VLP-16 User Manual, table 9-1:
https://data.ouster.io/downloads/velodyne/user-manual/vlp-16-user-manual-revf.pdf).
Mounted at height h, the lowest beam only reaches the ground at horizontal
distance h / tan(15 deg). At h=0.40 m that is ~1.5 m: anything on the ground
closer than ~1.5 m to the robot is OUTSIDE the VLP16's coverage entirely —
true for a single ring AND for this height-band approach alike, since both
derive from the same physical returns. This bridge cannot fix that; if nothing
else on the robot covers close-range contact (bump switch, the STM32
firmware's own safety envelope, etc.), that gap needs a different sensor, not
a different scan-conversion parameter.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    base_frame = LaunchConfiguration("base_frame")
    min_height = LaunchConfiguration("min_height")
    max_height = LaunchConfiguration("max_height")
    range_min = LaunchConfiguration("range_min")

    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/velodyne_points"),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description=(
                    "fusion_graph's scan_topic parameter defaults to /scan_deskewed "
                    "(passthrough of raw /scan when no deskew IMU is available) and "
                    "nav2_params_lidar.yaml's costmap_scan_filter_node reads /scan — "
                    "verify both against this before assuming it 'just works'."
                ),
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description=(
                    "Ground-relative filtering: every point is transformed into this "
                    "frame BEFORE the height-band filter, via the static lidar->base "
                    "TF, so min/max_height mean metres above the ground, not above the "
                    "sensor. Must match a frame in your URDF/TF tree (check whether "
                    "your costmap/collision_monitor use base_link or base_footprint "
                    "and mirror that, though for a rigid mount either works — they "
                    "differ only in z, and z isn't what this filters ambiguously here)."
                ),
            ),
            # Ground-relative now (see base_frame above). 0.03 m: above typical
            # grass-blade height so mowed/unmowed lawn texture doesn't turn into
            # phantom obstacles. 0.35 m: just below the VLP16's own mount height,
            # so it still catches hoses, toys, garden edging, sprinkler heads —
            # raise it if you also want taller obstacles (fence posts, low
            # branches) in this same /scan rather than relying on the 3D cloud
            # elsewhere. Re-tune both for YOUR actual mount height/angle.
            DeclareLaunchArgument("min_height", default_value="0.03"),
            DeclareLaunchArgument("max_height", default_value="0.35"),
            # See CAVEAT 2 above: at a 0.40 m mount, nothing on the ground closer
            # than ~1.5 m is physically visible to the VLP16 regardless of this
            # value. range_min here just avoids the robot's own chassis/deck
            # showing up as a return, it does NOT create a close-range blind
            # zone that wasn't already there.
            DeclareLaunchArgument("range_min", default_value="0.30"),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="costmap_scan_slice",
                remappings=[("cloud_in", cloud_topic), ("scan", scan_topic)],
                parameters=[
                    {
                        "target_frame": base_frame,
                        "min_height": min_height,
                        "max_height": max_height,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.0087,  # ~0.5 deg
                        "range_min": range_min,
                        "range_max": 12.0,
                        "use_inf": True,
                        "transform_tolerance": 0.05,
                    }
                ],
            ),
        ]
    )
