#include "node_lidar_ros.h"

#include "node_lidar.h"

#include "rclcpp/rclcpp.hpp"

// =============================================================================
// Stripped-down ROS2 wrapper for the cspc_lidar SDK (Coin-D4A / Coin-D6 family).
//
// Removed vs. the upstream cspc_lidar package, to match the footprint of the
// other LiDAR drivers in MowgliNext (lidar-ldlidar, lidar-rplidar, lidar-stl27l),
// none of which carry any of this extra surface area:
//   - PCL-based /point_cloud (PointCloud2) publisher: pulled in libpcl-dev +
//     pcl_conversions (large, multi-arch-build-risk deps) for a topic nothing
//     downstream consumes. MowgliNext only ever needs /scan (LaserScan).
//   - The "lidar_status" UInt16 control subscriber (start/stop/exposure/speed
//     commands) and the "lsd_error" String error topic: unused by any
//     MowgliNext consumer (collision_monitor, Nav2 costmap, obstacle_tracker
//     all just read /scan). The lidar auto-starts on its own regardless of
//     whether anything ever publishes to lidar_status.
//
// Defaults below were changed from the upstream package to match MowgliNext's
// conventions for other LiDAR sensor containers:
//   - port:      /dev/lidar  (the generic udev symlink MowgliNext's installer
//                creates for whichever LiDAR is configured, instead of the
//                upstream's hardware-specific /dev/sc_mini)
//   - frame_id:  lidar_link  (matches mowgli_bringup's URDF, same frame name
//                used by the LD19/RPLiDAR/STL27L drivers)
//   - version:   4           (M1CT_TOF mode -- confirmed identical wire
//                protocol and numeric value to the Coin-D6 vendor driver's
//                M1CT_Coin_D2 setting)
//   - baudrate:  230400      (Coin-D6 default, same as LD19/STL27L)
// =============================================================================

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	auto node = rclcpp::Node::make_shared("cspc_lidar");

	node->declare_parameter<std::string>("port", "/dev/lidar");
	node->get_parameter("port", node_lidar.lidar_general_info.port);

	node->declare_parameter<int>("baudrate", 230400);
	node->get_parameter("baudrate", node_lidar.lidar_general_info.m_SerialBaudrate);

	node->declare_parameter<std::string>("frame_id", "lidar_link");
	node->get_parameter("frame_id", node_lidar.lidar_general_info.frame_id);

	node->declare_parameter<int>("version", 4);
	node->get_parameter("version", node_lidar.lidar_general_info.version);

	rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(100));
	auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos_profile);

	node_start();

	while (rclcpp::ok())
	{
		if (node_lidar.lidar_status.lidar_abnormal_state != 0)
		{
			if (node_lidar.lidar_status.lidar_abnormal_state & 0x01)
			{
				RCLCPP_ERROR(node->get_logger(), "node_lidar is trapped");
			}
			if (node_lidar.lidar_status.lidar_abnormal_state & 0x02)
			{
				RCLCPP_ERROR(node->get_logger(), "node_lidar frequency abnormal");
			}
			if (node_lidar.lidar_status.lidar_abnormal_state & 0x04)
			{
				RCLCPP_ERROR(node->get_logger(), "node_lidar is blocked");
			}
			node_lidar.serial_port->write_data(end_lidar, 4);
			node_lidar.lidar_status.lidar_ready = false;

			sleep(1);
		}

		LaserScan scan;

		if (data_handling(scan))
		{
			auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

			scan_msg->ranges.resize(scan.points.size());
			scan_msg->intensities.resize(scan.points.size());
			scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
			scan_msg->header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
			scan_msg->header.frame_id = node_lidar.lidar_general_info.frame_id;
			scan_msg->angle_min = scan.config.min_angle;
			scan_msg->angle_max = scan.config.max_angle;
			scan_msg->angle_increment = scan.config.angle_increment;
			scan_msg->scan_time = scan.config.scan_time;
			scan_msg->time_increment = scan.config.time_increment;
			scan_msg->range_min = scan.config.min_range;
			scan_msg->range_max = scan.config.max_range;

			for (size_t i = 0; i < scan.points.size(); i++)
			{
				scan_msg->ranges[i] = scan.points[i].range;
				scan_msg->intensities[i] = scan.points[i].intensity;
			}

			laser_pub->publish(*scan_msg);
		}
	}

	node_lidar.serial_port->write_data(end_lidar, 4);

	return 0;
}
