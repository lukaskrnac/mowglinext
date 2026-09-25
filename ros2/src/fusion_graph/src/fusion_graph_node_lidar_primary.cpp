// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — external absolute-pose source from lidar_localization_ros2.
//
// This is deliberately symmetric to OnGnss in fusion_graph_node_callbacks_a.cpp: same
// idea (an absolute XY observation, with its own covariance, queued onto the graph
// through GraphManager), same shape of safety gates (freshness, covariance sanity,
// graph-node association), different source. It does NOT touch graph_manager.cpp:
// GraphManager::QueueLidarMapXy() is a plain API call and is NOT gated by
// use_lidar_map_anchor_ (that bool only controls this node's own built-in Beluga
// 2D-grid fallback, see fusion_graph_node_lidar_anchor.cpp) — so this external NDT
// pose and the built-in Beluga anchor can coexist without any GTSAM/iSAM2 change.
//
// Manual switch: `primary_localization_source` ("gps" | "lidar") decides which of
// OnGnss / OnLidarPose actually calls its Queue*() method. The other source keeps
// running and updating its own freshness/diagnostics bookkeeping, so its quality is
// visible on /fusion_graph/diagnostics before you ever flip the switch. See
// OnSetParameters() below for the runtime (`ros2 param set`) hook.
//
// What this deliberately does NOT do (yet), and why:
//   - No wheel/gyro motion-consistency "jump gate" the way OnGnss has for RTK
//     wrong-fixes (rtk_wrongfix_gate.hpp). /alignment_status already carries the
//     localizer's own accept/reject signal (failure_category, consecutive_rejected)
//     which plays a similar role; bolting a second, independent jump gate onto a
//     shared accumulator with the GPS path risks corrupting GPS's own wrong-fix
//     state across a source switch. Add a LiDAR-specific jump gate later if field
//     data shows /alignment_status isn't catching bad jumps on its own.
//   - Heading is fed via a SEPARATE QueueYaw() call, gated by lidar_pose_feed_yaw_
//     and only while LiDAR is the active primary source — never silently mixed into
//     COG/mag yaw fusion while GPS is primary. The project's own lidar_anchor code
//     is XY-only on purpose (2026-07-22 yaw-flip incident); NDT heading is generally
//     far more reliable than that 2D-grid AMCL heading estimate would have been, but
//     keep this gated and watch /fusion_graph/diagnostics cov_yawyaw after enabling.

#include "fusion_graph/fusion_graph_node.hpp"

#include <algorithm>
#include <cmath>

#include "fusion_graph/lidar_covariance.hpp"
#include <Eigen/Eigenvalues>

namespace fusion_graph
{

namespace
{
double LargestSigma(const Eigen::Matrix2d& cov)
{
  Eigen::Matrix2d sym = 0.5 * (cov + cov.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(sym);
  if (es.info() != Eigen::Success)
    return std::numeric_limits<double>::quiet_NaN();
  return std::sqrt(std::max(0.0, es.eigenvalues().maxCoeff()));
}
}  // namespace

void FusionGraphNode::OnLidarAlignmentStatus(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
{
  // lidar_localization_ros2 publishes one DiagnosticStatus per update, named
  // "lidar_localization_ros2/alignment" (docs/troubleshooting.md). Defensive:
  // don't assume it's status[0], and don't crash if the name ever changes —
  // just keep the last-known-good snapshot (safe/closed default from the
  // header's field initializers covers "never seen a message yet").
  for (const auto& status : msg->status)
  {
    if (status.name.find("lidar_localization_ros2/alignment") == std::string::npos)
      continue;
    for (const auto& kv : status.values)
    {
      if (kv.key == "failure_category")
        lidar_failure_category_ = kv.value;
      else if (kv.key == "consecutive_rejected_updates")
      {
        try
        {
          lidar_consecutive_rejected_updates_ = std::stoll(kv.value);
        }
        catch (const std::exception&)
        {
          // Malformed diagnostic value: treat as "not healthy" rather than crash.
          lidar_consecutive_rejected_updates_ = std::numeric_limits<int64_t>::max();
        }
      }
      else if (kv.key == "reinitialization_requested")
        lidar_reinitialization_requested_ = (kv.value == "true" || kv.value == "1");
    }
    lidar_alignment_stamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    return;
  }
}

void FusionGraphNode::OnLidarPose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  // frame_id must already be the SAME map frame fusion_graph uses (ENU metres
  // around datum_lat_/datum_lon_, dock at the origin) — see
  // tools/align_glim_map.py. This node does not re-project lat/lon here; the
  // alignment is a one-time offline step on the map file, not a runtime TF.
  if (msg->header.frame_id != map_frame_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "fusion_graph: /pcl_pose frame_id '%s' != map_frame '%s' — sample "
                         "dropped (is the GLIM map aligned to the fusion_graph datum?)",
                         msg->header.frame_id.c_str(),
                         map_frame_.c_str());
    return;
  }

  // Freshness. lidar_localization_ros2 stamps /pcl_pose at scan time, same
  // convention as the GNSS receipt-stamp handling in OnGnss.
  const rclcpp::Time now_stamp = this->now();
  const rclcpp::Time measurement_stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double age_s = (now_stamp - measurement_stamp).seconds();
  if (!std::isfinite(age_s) || age_s < 0.0 || age_s > lidar_pose_max_age_s_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "fusion_graph: /pcl_pose age %.3f s (> %.3f s or negative/non-finite) "
                         "— sample dropped",
                         age_s,
                         lidar_pose_max_age_s_);
    return;
  }

  // Health gate from /alignment_status. A stale status (localizer not
  // publishing, or crashed) is treated as unhealthy — fail closed, same
  // philosophy as the GPS unknown-covariance reject in OnGnss.
  constexpr double kAlignmentStatusMaxAgeS = 2.0;
  const bool alignment_fresh =
      lidar_alignment_stamp_.has_value() &&
      (now_stamp - *lidar_alignment_stamp_).seconds() <= kAlignmentStatusMaxAgeS;
  const bool healthy = alignment_fresh && lidar_failure_category_ == "healthy" &&
                       lidar_consecutive_rejected_updates_ <=
                           static_cast<int64_t>(lidar_pose_max_consecutive_rejected_) &&
                       !lidar_reinitialization_requested_;
  if (!healthy)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "fusion_graph: LiDAR localizer unhealthy (fresh=%d, category=%s, "
                         "consecutive_rejected=%ld, reinit_requested=%d) — sample dropped",
                         alignment_fresh,
                         lidar_failure_category_.c_str(),
                         static_cast<long>(lidar_consecutive_rejected_updates_),
                         lidar_reinitialization_requested_);
    return;
  }

  // Covariance: /pcl_pose is geometry_msgs/PoseWithCovarianceStamped, 6x6
  // row-major (x,y,z,roll,pitch,yaw), diagonal indices 0/7/14/21/28/35
  // (docs/pose_covariance.md). Read the xy 2x2 block including any
  // off-diagonal term the localizer's covariance mode produces.
  Eigen::Matrix2d cov;
  cov(0, 0) = msg->pose.covariance[0];
  cov(0, 1) = msg->pose.covariance[1];
  cov(1, 0) = msg->pose.covariance[6];
  cov(1, 1) = msg->pose.covariance[7];
  const double sigma = LargestSigma(cov);
  if (!std::isfinite(sigma) || sigma <= 0.0)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "fusion_graph: /pcl_pose has non-finite/zero covariance — sample "
                         "dropped");
    return;
  }
  if (sigma > lidar_pose_max_sigma_reject_m_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "fusion_graph: /pcl_pose sigma=%.3f m > reject threshold %.3f m — "
                         "sample dropped",
                         sigma,
                         lidar_pose_max_sigma_reject_m_);
    return;
  }
  const auto cov_applied = FloorLidarCovariance(cov, lidar_pose_sigma_floor_m_);
  if (!cov_applied)
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "fusion_graph: /pcl_pose covariance not PSD — dropped");
    return;
  }

  // Associate with the graph node live at this measurement's timestamp —
  // identical pattern to OnGnss's measurement_node / FindNodeAtOrBefore.
  std::optional<uint64_t> measurement_node;
  if (graph_->IsInitialized())
  {
    measurement_node = graph_->FindNodeAtOrBefore(measurement_stamp.seconds());
    if (!measurement_node)
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           5000,
                           "fusion_graph: no live graph node at/before /pcl_pose epoch — "
                           "sample dropped");
      return;
    }
  }

  const double mx = msg->pose.pose.position.x;
  const double my = msg->pose.pose.position.y;

  // The manual switch: only actually fuse into the graph while LiDAR is the
  // selected primary source. Health/covariance bookkeeping above still ran
  // unconditionally, so /fusion_graph/diagnostics shows LiDAR quality live
  // even while GPS is primary — that's what you watch before flipping.
  if (!primary_is_lidar_.load(std::memory_order_relaxed))
    return;

  graph_->QueueLidarMapXy(
      gtsam::Vector2(mx, my), *cov_applied, lidar_pose_robust_, measurement_node);

  if (lidar_pose_feed_yaw_)
  {
    const auto& q = msg->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    // cov[35] is yaw-yaw variance. Floor it the same way GPS/mag yaw sigma is
    // floored elsewhere — never trust a covariance report tighter than the
    // configured floor.
    const double yaw_var = msg->pose.covariance[35];
    const double sigma_yaw =
        std::isfinite(yaw_var) && yaw_var > 0.0
            ? std::max(std::sqrt(yaw_var), lidar_pose_yaw_sigma_floor_rad_)
            : lidar_pose_yaw_sigma_floor_rad_;
    graph_->QueueYaw(yaw, sigma_yaw, /*robust=*/true);
  }
}

rcl_interfaces::msg::SetParametersResult FusionGraphNode::OnSetParameters(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto& p : params)
  {
    if (p.get_name() != "primary_localization_source")
      continue;  // not ours: accept, some other mechanism may own it
    const std::string value = p.as_string();
    if (value == "gps")
    {
      primary_is_lidar_.store(false, std::memory_order_relaxed);
      RCLCPP_INFO(get_logger(), "fusion_graph: primary_localization_source -> gps");
    }
    else if (value == "lidar")
    {
      primary_is_lidar_.store(true, std::memory_order_relaxed);
      RCLCPP_INFO(get_logger(), "fusion_graph: primary_localization_source -> lidar");
    }
    else
    {
      result.successful = false;
      result.reason = "primary_localization_source must be 'gps' or 'lidar'";
    }
  }
  return result;
}

}  // namespace fusion_graph
