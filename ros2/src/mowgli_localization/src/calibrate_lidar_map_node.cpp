// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// calibrate_lidar_map_node — on-demand calibration of the external LiDAR
// localizer's map frame (lidar_localization_ros2 against a GLIM map, frame
// "lidar_map") into fusion_graph's `map` frame (ENU around the datum,
// Invariant 4).
//
// Why: a GLIM map lives in an arbitrary local frame (origin and heading of
// wherever mapping started). fusion_graph, the GUI areas and the dock pose
// all live in the datum ENU frame. A rigid 2D transform
//     p_map = R(theta) * p_lidar_map + t
// connects them. This node estimates it from a short drive under RTK-Fixed.
//
// How:
//   * idle until ~/start (std_srvs/Trigger) is called;
//   * for every RTK-Fixed /gps/fix it pairs the antenna ENU position with the
//     antenna position predicted from /pcl_pose (interpolated to the fix
//     time, lever arm rotated by the LiDAR's own yaw). Neither side uses a
//     fusion_graph yaw, so calibration is valid whatever
//     primary_localization_source is set to;
//   * pairs are gated on RTK-Fixed + receiver accuracy, a healthy
//     /alignment_status, low speed / yaw rate, and a minimum spacing;
//   * every fit_period_s it runs a robust rigid fit (lidar_map_alignment.hpp)
//     and finishes once there are enough pairs, enough spread in both
//     directions, a low RMS and a fit that has stopped moving;
//   * on success it writes calibration_path (YAML, like mag_calibration.yaml)
//     and pushes lidar_pose_map_{x,y,yaw,calibrated} to fusion_graph_node as
//     live parameters, so no restart is needed. fusion_graph.launch.py reads
//     the same file at the next start.
//
// Progress: ~/status (std_msgs/String, JSON, transient_local, 1 Hz).
// Cancel:   ~/cancel (std_srvs/Trigger).
//
// Operator procedure: RTK-Fixed, lidar_localization tracking, call ~/start,
// drive slowly (< 0.4 m/s) over as much of the garden as practical — a big
// "L" or a figure of eight, not just a straight line — until state is
// "succeeded".

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/string.hpp>

#include "mowgli_interfaces/gnss_status_utils.hpp"
#include "mowgli_interfaces/msg/gnss_status.hpp"
#include "mowgli_interfaces/wgs84_projection.hpp"
#include "mowgli_localization/lidar_map_alignment.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

namespace mowgli_localization
{

namespace lma = lidar_map_alignment;

namespace
{
std::string UtcNow()
{
  const std::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm tm_utc{};
  gmtime_r(&tt, &tm_utc);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
  return buf;
}

double YawFromQuat(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::string Fmt(double v, int prec = 4)
{
  if (!std::isfinite(v))
    return "null";
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
  return buf;
}
}  // namespace

class CalibrateLidarMapNode : public rclcpp::Node
{
public:
  explicit CalibrateLidarMapNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("calibrate_lidar_map_node", options)
  {
    datum_lat_ = declare_parameter<double>("datum_lat", 0.0);
    datum_lon_ = declare_parameter<double>("datum_lon", 0.0);
    lever_x_ = declare_parameter<double>("lever_arm_x", 0.0);
    lever_y_ = declare_parameter<double>("lever_arm_y", 0.0);

    const auto pose_topic = declare_parameter<std::string>("lidar_pose_topic", "/pcl_pose");
    const auto status_topic =
        declare_parameter<std::string>("alignment_status_topic", "/alignment_status");
    const auto fix_topic = declare_parameter<std::string>("gps_fix_topic", "/gps/fix");
    const auto gnss_status_topic =
        declare_parameter<std::string>("gps_status_topic", "/gps/status");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar_map");

    max_gps_accuracy_m_ = declare_parameter<double>("max_gps_accuracy_m", 0.03);
    gps_time_offset_s_ = declare_parameter<double>("gps_time_offset_s", 0.0);
    max_pair_gap_s_ = declare_parameter<double>("max_pair_gap_s", 0.5);
    max_speed_mps_ = declare_parameter<double>("max_speed_mps", 0.5);
    max_yaw_rate_rps_ = declare_parameter<double>("max_yaw_rate_rps", 0.6);
    min_pair_spacing_m_ = declare_parameter<double>("min_pair_spacing_m", 0.05);
    min_pairs_ = declare_parameter<int>("min_pairs", 300);
    min_major_std_m_ = declare_parameter<double>("min_major_std_m", 2.0);
    min_minor_std_m_ = declare_parameter<double>("min_minor_std_m", 1.0);
    max_rms_m_ = declare_parameter<double>("max_rms_m", 0.05);
    stable_window_s_ = declare_parameter<double>("stable_window_s", 20.0);
    stable_theta_rad_ = declare_parameter<double>("stable_theta_rad", 0.0035);  // 0.2 deg
    stable_t_m_ = declare_parameter<double>("stable_t_m", 0.02);
    fit_period_s_ = declare_parameter<double>("fit_period_s", 2.0);
    timeout_s_ = declare_parameter<double>("timeout_s", 900.0);
    calibration_path_ = declare_parameter<std::string>("calibration_path",
                                                       "/ros2_ws/maps/lidar_map_calibration.yaml");
    fusion_graph_node_ = declare_parameter<std::string>("fusion_graph_node", "/fusion_graph_node");

    auto sensor_qos = rclcpp::SensorDataQoS();
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic,
        sensor_qos,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m)
        {
          OnPose(*m);
        });
    sub_align_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        status_topic,
        rclcpp::QoS(10),
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m)
        {
          OnAlignment(*m);
        });
    sub_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        fix_topic,
        sensor_qos,
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m)
        {
          OnFix(*m);
        });
    sub_gnss_status_ = create_subscription<mowgli_interfaces::msg::GnssStatus>(
        gnss_status_topic,
        sensor_qos,
        [this](mowgli_interfaces::msg::GnssStatus::ConstSharedPtr m)
        {
          gnss_status_ = *m;
          gnss_status_rx_ = now();
        });

    status_pub_ =
        create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(1).transient_local());
    srv_start_ = create_service<std_srvs::srv::Trigger>(
        "~/start",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
          Start(*res);
        });
    srv_cancel_ = create_service<std_srvs::srv::Trigger>(
        "~/cancel",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
          if (state_ != State::kCollecting)
          {
            res->success = false;
            res->message = "not running";
            return;
          }
          Finish(State::kCanceled, "canceled by operator");
          res->success = true;
          res->message = "canceled";
        });

    fit_timer_ = create_wall_timer(std::chrono::duration<double>(std::max(0.2, fit_period_s_)),
                                   [this]()
                                   {
                                     OnFitTimer();
                                   });
    status_timer_ = create_wall_timer(std::chrono::seconds(1),
                                      [this]()
                                      {
                                        PublishStatus();
                                      });

    fg_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, fusion_graph_node_);

    RCLCPP_INFO(get_logger(),
                "LiDAR map calibration ready (idle). Start with ~/start, watch ~/status. "
                "datum=(%.9f, %.9f) lever_arm=(%.3f, %.3f) frame=%s output=%s",
                datum_lat_,
                datum_lon_,
                lever_x_,
                lever_y_,
                lidar_frame_.c_str(),
                calibration_path_.c_str());
    PublishStatus();
  }

private:
  enum class State
  {
    kIdle,
    kCollecting,
    kSucceeded,
    kFailed,
    kCanceled
  };

  static const char* StateName(State s)
  {
    switch (s)
    {
      case State::kIdle:
        return "idle";
      case State::kCollecting:
        return "collecting";
      case State::kSucceeded:
        return "succeeded";
      case State::kFailed:
        return "failed";
      case State::kCanceled:
        return "canceled";
    }
    return "unknown";
  }

  struct PendingFix
  {
    double t = 0.0;  // fix time (+ gps_time_offset_s), ROS clock
    double received = 0.0;  // arrival time, for the wait timeout
    double east = 0.0;
    double north = 0.0;
  };
  static constexpr double kMaxPairWaitS = 3.0;

  struct FitSample
  {
    double t;
    lma::Rigid2D tf;
  };

  // ── callbacks ───────────────────────────────────────────────────
  void OnPose(const geometry_msgs::msg::PoseWithCovarianceStamped& m)
  {
    last_pose_frame_ = m.header.frame_id;
    if (m.header.frame_id != lidar_frame_)
      return;
    lma::TimedPose p;
    p.t = rclcpp::Time(m.header.stamp).seconds();
    p.x = m.pose.pose.position.x;
    p.y = m.pose.pose.position.y;
    p.yaw = YawFromQuat(m.pose.pose.orientation);
    if (!poses_.empty() && p.t <= poses_.back().t)
      return;  // out-of-order / duplicate
    poses_.push_back(p);
    while (!poses_.empty() && poses_.front().t < p.t - 10.0)
      poses_.pop_front();
    if (state_ == State::kCollecting)
      ProcessPending();
  }

  void OnAlignment(const diagnostic_msgs::msg::DiagnosticArray& m)
  {
    for (const auto& st : m.status)
    {
      if (st.name.find("lidar_localization_ros2/alignment") == std::string::npos)
        continue;
      bool healthy = false;
      bool reinit = false;
      for (const auto& kv : st.values)
      {
        if (kv.key == "failure_category")
          healthy = kv.value == "healthy";
        else if (kv.key == "reinitialization_requested")
          reinit = kv.value == "true" || kv.value == "1";
      }
      lidar_healthy_ = healthy && !reinit;
      lidar_health_rx_ = now();
      return;
    }
  }

  bool LidarHealthy() const
  {
    return lidar_health_rx_.has_value() && (now() - *lidar_health_rx_).seconds() < 2.0 &&
           lidar_healthy_;
  }

  // RTK-Fixed + receiver accuracy gate. Prefers the typed /gps/status (the
  // same source the rest of the stack trusts), falls back to NavSatFix.
  bool GpsGood(const sensor_msgs::msg::NavSatFix& fix, std::string& why) const
  {
    namespace gsu = mowgli_interfaces::gnss_status_utils;
    if (gnss_status_.has_value() && gnss_status_rx_.has_value() &&
        (now() - *gnss_status_rx_).seconds() < 2.0)
    {
      if (!gsu::IsRtkFixed(*gnss_status_))
      {
        why = "gps_not_rtk_fixed";
        return false;
      }
      const auto acc = gsu::HorizontalAccuracyMeters(*gnss_status_);
      if (acc.has_value() && *acc > max_gps_accuracy_m_)
      {
        why = "gps_accuracy";
        return false;
      }
      return true;
    }
    if (fix.status.status != sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
    {
      why = "gps_not_rtk_fixed";
      return false;
    }
    if (fix.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
    {
      const double sigma =
          std::sqrt(std::max(fix.position_covariance[0], fix.position_covariance[4]));
      if (sigma > max_gps_accuracy_m_)
      {
        why = "gps_accuracy";
        return false;
      }
    }
    return true;
  }

  void Reject(const std::string& why)
  {
    ++rejects_[why];
  }

  // GNSS side of a pair. Gated here (RTK/accuracy and LiDAR health are judged
  // at fix time), then queued: /pcl_pose is stamped at scan time but only
  // published after NDT has run, so when a fix arrives the LiDAR pose for its
  // timestamp usually does not exist yet. ProcessPending() pairs it once the
  // pose buffer has caught up.
  void OnFix(const sensor_msgs::msg::NavSatFix& fix)
  {
    if (state_ != State::kCollecting)
      return;
    ++gps_fixes_;
    if (!std::isfinite(fix.latitude) || !std::isfinite(fix.longitude))
      return Reject("gps_invalid");

    std::string why;
    if (!GpsGood(fix, why))
      return Reject(why);
    if (!LidarHealthy())
      return Reject("lidar_unhealthy");
    if (poses_.empty())
      return Reject(last_pose_frame_.empty() || last_pose_frame_ == lidar_frame_ ? "no_lidar_pose"
                                                                                 : "lidar_frame");

    PendingFix pf;
    pf.t = rclcpp::Time(fix.header.stamp).seconds() + gps_time_offset_s_;
    pf.received = now().seconds();
    mowgli_interfaces::wgs84::ToEnu(
        fix.latitude, fix.longitude, datum_lat_, datum_lon_, pf.east, pf.north);
    pending_.push_back(pf);
    ProcessPending();
  }

  void ProcessPending()
  {
    const double tnow = now().seconds();
    while (!pending_.empty())
    {
      const PendingFix pf = pending_.front();
      if (poses_.empty() || pf.t > poses_.back().t)
      {
        // LiDAR has not produced a pose for this instant yet — wait, but not forever.
        if (tnow - pf.received > kMaxPairWaitS)
        {
          pending_.pop_front();
          Reject("lidar_pose_timeout");
          continue;
        }
        return;
      }
      pending_.pop_front();
      if (pf.t < poses_.front().t)
      {
        Reject("lidar_pose_too_old");
        continue;
      }
      const auto pose = lma::Interpolate(poses_, pf.t, max_pair_gap_s_);
      if (!pose)
      {
        Reject("lidar_pose_gap");
        continue;
      }
      const auto motion = lma::SpeedAndYawRate(poses_, pf.t, 0.3);
      if (motion && (motion->first > max_speed_mps_ || motion->second > max_yaw_rate_rps_))
      {
        Reject("too_fast");
        continue;
      }

      lma::PointPair pr;
      lma::AntennaFromBase(pose->x, pose->y, pose->yaw, lever_x_, lever_y_, pr.src_x, pr.src_y);
      pr.dst_x = pf.east;
      pr.dst_y = pf.north;
      if (!pairs_.empty() && std::hypot(pr.dst_x - pairs_.back().dst_x,
                                        pr.dst_y - pairs_.back().dst_y) < min_pair_spacing_m_)
      {
        Reject("not_moving");
        continue;
      }
      pairs_.push_back(pr);
    }
  }

  // ── lifecycle ───────────────────────────────────────────────────
  void Start(std_srvs::srv::Trigger::Response& res)
  {
    if (state_ == State::kCollecting)
    {
      res.success = false;
      res.message = "already collecting";
      return;
    }
    if (datum_lat_ == 0.0 && datum_lon_ == 0.0)
    {
      res.success = false;
      res.message = "datum_lat/datum_lon not set";
      return;
    }
    pairs_.clear();
    pending_.clear();
    gps_fixes_ = 0;
    rejects_.clear();
    history_.clear();
    last_fit_.reset();
    last_spread_ = {};
    message_ = "collecting — drive slowly over the garden (L-shape / figure of eight)";
    started_ = now();
    state_ = State::kCollecting;
    RCLCPP_INFO(get_logger(), "LiDAR map calibration started.");
    res.success = true;
    res.message = "started";
    PublishStatus();
  }

  void Finish(State s, const std::string& msg)
  {
    state_ = s;
    message_ = msg;
    if (s == State::kSucceeded)
      RCLCPP_INFO(get_logger(), "LiDAR map calibration: %s", msg.c_str());
    else
      RCLCPP_WARN(get_logger(), "LiDAR map calibration %s: %s", StateName(s), msg.c_str());
    PublishStatus();
  }

  void OnFitTimer()
  {
    if (state_ != State::kCollecting)
      return;
    const double elapsed = (now() - started_).seconds();
    ProcessPending();

    if (pairs_.size() >= 20)
    {
      last_spread_ = lma::SourceSpread(pairs_);
      last_fit_ = lma::RobustFitRigid2D(pairs_);
      if (last_fit_)
      {
        const double tnow = now().seconds();
        history_.push_back({tnow, last_fit_->transform});
        while (!history_.empty() && history_.front().t < tnow - 2.0 * stable_window_s_)
          history_.pop_front();
      }
    }

    std::string blocker;
    if (static_cast<int>(pairs_.size()) < min_pairs_)
      blocker = "need more pairs";
    else if (last_spread_.major_std_m < min_major_std_m_ ||
             last_spread_.minor_std_m < min_minor_std_m_)
      blocker = "drive a wider area (spread too small in one direction)";
    else if (!last_fit_)
      blocker = "fit failed";
    else if (last_fit_->rms_m > max_rms_m_)
      blocker = "residual RMS too high";
    else if (!Stable())
      blocker = "waiting for the fit to settle";

    if (blocker.empty())
    {
      Persist();
      return;
    }
    message_ = "collecting — " + blocker;
    if (elapsed > timeout_s_)
      Finish(State::kFailed, "timeout: " + blocker);
  }

  bool Stable() const
  {
    if (history_.empty())
      return false;
    const double tnow = history_.back().t;
    if (tnow - history_.front().t < stable_window_s_)
      return false;
    const auto& ref = history_.back().tf;
    for (const auto& h : history_)
    {
      if (h.t < tnow - stable_window_s_)
        continue;
      if (std::abs(lma::WrapAngle(h.tf.theta - ref.theta)) > stable_theta_rad_ ||
          std::hypot(h.tf.tx - ref.tx, h.tf.ty - ref.ty) > stable_t_m_)
        return false;
    }
    return true;
  }

  void Persist()
  {
    const auto& fit = *last_fit_;
    const auto& tf = fit.transform;

    YAML::Emitter out;
    out.SetDoublePrecision(9);
    out << YAML::BeginMap;
    out << YAML::Key << "lidar_map_calibration" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << tf.tx;
    out << YAML::Key << "y" << YAML::Value << tf.ty;
    out << YAML::Key << "yaw" << YAML::Value << tf.theta;
    out << YAML::Key << "lidar_frame" << YAML::Value << lidar_frame_;
    out << YAML::Key << "datum_lat" << YAML::Value << datum_lat_;
    out << YAML::Key << "datum_lon" << YAML::Value << datum_lon_;
    out << YAML::Key << "pairs" << YAML::Value << pairs_.size();
    out << YAML::Key << "inliers" << YAML::Value << fit.inliers;
    out << YAML::Key << "rms_m" << YAML::Value << fit.rms_m;
    out << YAML::Key << "max_residual_m" << YAML::Value << fit.max_m;
    out << YAML::Key << "spread_major_std_m" << YAML::Value << last_spread_.major_std_m;
    out << YAML::Key << "spread_minor_std_m" << YAML::Value << last_spread_.minor_std_m;
    out << YAML::Key << "calibrated_at" << YAML::Value << UtcNow();
    out << YAML::EndMap;
    out << YAML::EndMap;

    std::error_code ec;
    const auto dir = std::filesystem::path(calibration_path_).parent_path();
    if (!dir.empty())
      std::filesystem::create_directories(dir, ec);
    const std::string tmp = calibration_path_ + ".tmp";
    {
      std::ofstream f(tmp, std::ios::trunc);
      f << "# Written by calibrate_lidar_map_node. p_map = R(yaw) * p_lidar_map + (x, y).\n"
        << "# Delete this file to drop the calibration (fusion_graph then ignores /pcl_pose).\n"
        << out.c_str() << "\n";
      if (!f.good())
      {
        Finish(State::kFailed, "could not write " + tmp);
        return;
      }
    }
    std::filesystem::rename(tmp, calibration_path_, ec);
    if (ec)
    {
      Finish(State::kFailed, "could not write " + calibration_path_ + ": " + ec.message());
      return;
    }

    PushToFusionGraph(tf);

    Finish(State::kSucceeded,
           "saved " + calibration_path_ + ": yaw=" + Fmt(tf.theta * 180.0 / M_PI, 3) + " deg, t=(" +
               Fmt(tf.tx, 3) + ", " + Fmt(tf.ty, 3) + ") m, rms=" + Fmt(fit.rms_m * 100.0, 1) +
               " cm over " + std::to_string(fit.inliers) + " pairs");
  }

  void PushToFusionGraph(const lma::Rigid2D& tf)
  {
    if (!fg_params_->service_is_ready())
    {
      RCLCPP_WARN(get_logger(),
                  "%s parameter service not available — calibration saved, it will apply at "
                  "the next fusion_graph start.",
                  fusion_graph_node_.c_str());
      return;
    }
    fg_params_->set_parameters(
        {rclcpp::Parameter("lidar_pose_map_x", tf.tx),
         rclcpp::Parameter("lidar_pose_map_y", tf.ty),
         rclcpp::Parameter("lidar_pose_map_yaw", tf.theta),
         rclcpp::Parameter("lidar_pose_map_calibrated", true)},
        [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> fut)
        {
          bool ok = true;
          for (const auto& r : fut.get())
          {
            if (!r.successful)
            {
              ok = false;
              RCLCPP_WARN(get_logger(), "fusion_graph rejected parameter: %s", r.reason.c_str());
            }
          }
          if (ok)
            RCLCPP_INFO(get_logger(),
                        "Calibration applied live to %s.",
                        fusion_graph_node_.c_str());
        });
  }

  void PublishStatus()
  {
    std::ostringstream j;
    j << "{\"state\":\"" << StateName(state_) << "\"";
    j << ",\"message\":\"" << message_ << "\"";
    j << ",\"pairs\":" << pairs_.size() << ",\"min_pairs\":" << min_pairs_;
    j << ",\"spread_major_std_m\":" << Fmt(last_spread_.major_std_m, 2)
      << ",\"spread_minor_std_m\":" << Fmt(last_spread_.minor_std_m, 2);
    if (last_fit_)
    {
      j << ",\"yaw_deg\":" << Fmt(last_fit_->transform.theta * 180.0 / M_PI, 3)
        << ",\"x_m\":" << Fmt(last_fit_->transform.tx, 3)
        << ",\"y_m\":" << Fmt(last_fit_->transform.ty, 3)
        << ",\"rms_cm\":" << Fmt(last_fit_->rms_m * 100.0, 1)
        << ",\"inliers\":" << last_fit_->inliers;
    }
    j << ",\"lidar_healthy\":" << (LidarHealthy() ? "true" : "false");
    j << ",\"gps_fixes\":" << gps_fixes_;
    if (gnss_status_.has_value())
    {
      namespace gsu = mowgli_interfaces::gnss_status_utils;
      const auto acc = gsu::HorizontalAccuracyMeters(*gnss_status_);
      j << ",\"gps_rtk_fixed\":" << (gsu::IsRtkFixed(*gnss_status_) ? "true" : "false")
        << ",\"gps_fix_type\":" << static_cast<int>(gnss_status_->fix_type)
        << ",\"gps_rtk_mode\":" << static_cast<int>(gnss_status_->rtk_mode)
        << ",\"gps_accuracy_m\":" << (acc ? Fmt(*acc, 3) : std::string("null"));
    }
    else
    {
      j << ",\"gps_status\":\"no /gps/status received\"";
    }
    if (!last_pose_frame_.empty() && last_pose_frame_ != lidar_frame_)
      j << ",\"warning\":\"/pcl_pose frame is '" << last_pose_frame_ << "', expected '"
        << lidar_frame_ << "' (set lidar_localization global_frame_id)\"";
    j << ",\"rejected\":{";
    bool first = true;
    for (const auto& [k, v] : rejects_)
    {
      j << (first ? "" : ",") << "\"" << k << "\":" << v;
      first = false;
    }
    j << "}}";
    std_msgs::msg::String m;
    m.data = j.str();
    status_pub_->publish(m);
  }

  // ── parameters ──────────────────────────────────────────────────
  double datum_lat_, datum_lon_, lever_x_, lever_y_;
  std::string lidar_frame_;
  double max_gps_accuracy_m_, gps_time_offset_s_, max_pair_gap_s_, max_speed_mps_,
      max_yaw_rate_rps_, min_pair_spacing_m_;
  int min_pairs_;
  double min_major_std_m_, min_minor_std_m_, max_rms_m_, stable_window_s_, stable_theta_rad_,
      stable_t_m_, fit_period_s_, timeout_s_;
  std::string calibration_path_, fusion_graph_node_;

  // ── state ───────────────────────────────────────────────────────
  State state_ = State::kIdle;
  std::string message_ = "idle — call ~/start";
  rclcpp::Time started_{0, 0, RCL_ROS_TIME};
  std::deque<lma::TimedPose> poses_;
  std::string last_pose_frame_;
  std::vector<lma::PointPair> pairs_;
  std::deque<PendingFix> pending_;
  std::size_t gps_fixes_ = 0;
  std::map<std::string, std::size_t> rejects_;
  std::deque<FitSample> history_;
  std::optional<lma::RobustFitResult> last_fit_;
  lma::Spread last_spread_;
  bool lidar_healthy_ = false;
  std::optional<rclcpp::Time> lidar_health_rx_;
  std::optional<mowgli_interfaces::msg::GnssStatus> gnss_status_;
  std::optional<rclcpp::Time> gnss_status_rx_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_align_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Subscription<mowgli_interfaces::msg::GnssStatus>::SharedPtr sub_gnss_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_, srv_cancel_;
  rclcpp::TimerBase::SharedPtr fit_timer_, status_timer_;
  std::shared_ptr<rclcpp::AsyncParametersClient> fg_params_;
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::CalibrateLidarMapNode>());
  rclcpp::shutdown();
  return 0;
}
