// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure math for calibrate_lidar_map_node: estimating the rigid 2D transform
// that takes the external LiDAR localizer's map frame ("lidar_map", i.e. the
// arbitrary local frame of a GLIM map) into fusion_graph's `map` frame (ENU
// around datum_lat/datum_lon, Invariant 4).
//
//   p_map = R(theta) * p_lidar_map + t
//
// No ROS types in here, so everything is unit-testable (test_lidar_map_alignment).
//
// Correspondences are (lidar_map point, map point) pairs of the SAME physical
// point — in practice the GNSS antenna: the map side is the raw RTK fix
// projected to ENU, the lidar side is /pcl_pose with the antenna lever arm
// applied using the LiDAR's own yaw (AntennaFromBase). Neither side uses a
// yaw from fusion_graph, so the fit stays valid even while fusion_graph is
// itself running on LiDAR.
//
// The fit is rigid (no scale): GLIM maps and ENU are both metric, and a free
// scale would only absorb reference errors instead of exposing them.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

namespace mowgli_localization::lidar_map_alignment
{

inline double WrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

/// One correspondence: src = lidar_map frame, dst = fusion_graph map frame.
struct PointPair
{
  double src_x = 0.0;
  double src_y = 0.0;
  double dst_x = 0.0;
  double dst_y = 0.0;
  // Base pose the antenna point was derived from (lidar_map frame). Only
  // used by the lever-arm diagnostic (FitWithLeverArm).
  double base_x = 0.0;
  double base_y = 0.0;
  double base_yaw = 0.0;
};

/// p_dst = R(theta) * p_src + (tx, ty)
struct Rigid2D
{
  double theta = 0.0;
  double tx = 0.0;
  double ty = 0.0;

  void Apply(double x, double y, double& out_x, double& out_y) const
  {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    out_x = c * x - s * y + tx;
    out_y = s * x + c * y + ty;
  }

  /// Inverse transform (map -> lidar_map).
  Rigid2D Inverse() const
  {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    Rigid2D inv;
    inv.theta = -theta;
    inv.tx = -(c * tx + s * ty);
    inv.ty = -(-s * tx + c * ty);
    return inv;
  }
};

/// Closed-form least-squares rigid 2D fit (Umeyama / Kabsch without scale;
/// in 2D the rotation reduces to one atan2). Uses only pairs where
/// `use[i]` is true (all when `use` is empty). Needs >= 2 usable pairs with
/// non-zero source spread.
inline std::optional<Rigid2D> FitRigid2D(const std::vector<PointPair>& pairs,
                                         const std::vector<bool>& use = {})
{
  double n = 0.0;
  double msx = 0.0, msy = 0.0, mdx = 0.0, mdy = 0.0;
  for (std::size_t i = 0; i < pairs.size(); ++i)
  {
    if (!use.empty() && !use[i])
      continue;
    msx += pairs[i].src_x;
    msy += pairs[i].src_y;
    mdx += pairs[i].dst_x;
    mdy += pairs[i].dst_y;
    n += 1.0;
  }
  if (n < 2.0)
    return std::nullopt;
  msx /= n;
  msy /= n;
  mdx /= n;
  mdy /= n;

  double s_cos = 0.0;  // Σ (src·dst)
  double s_sin = 0.0;  // Σ (src × dst)
  double spread = 0.0;
  for (std::size_t i = 0; i < pairs.size(); ++i)
  {
    if (!use.empty() && !use[i])
      continue;
    const double ax = pairs[i].src_x - msx;
    const double ay = pairs[i].src_y - msy;
    const double bx = pairs[i].dst_x - mdx;
    const double by = pairs[i].dst_y - mdy;
    s_cos += ax * bx + ay * by;
    s_sin += ax * by - ay * bx;
    spread += ax * ax + ay * ay;
  }
  if (spread < 1e-9 || (std::abs(s_cos) < 1e-12 && std::abs(s_sin) < 1e-12))
    return std::nullopt;

  Rigid2D t;
  t.theta = std::atan2(s_sin, s_cos);
  const double c = std::cos(t.theta);
  const double s = std::sin(t.theta);
  t.tx = mdx - (c * msx - s * msy);
  t.ty = mdy - (s * msx + c * msy);
  return t;
}

inline double Residual(const Rigid2D& t, const PointPair& p)
{
  double x = 0.0, y = 0.0;
  t.Apply(p.src_x, p.src_y, x, y);
  return std::hypot(x - p.dst_x, y - p.dst_y);
}

struct RobustFitResult
{
  Rigid2D transform;
  std::size_t inliers = 0;
  double rms_m = 0.0;  // over inliers
  double max_m = 0.0;  // over inliers
  double median_m = 0.0;  // over all pairs, final transform
};

inline double Median(std::vector<double> v)
{
  if (v.empty())
    return 0.0;
  const std::size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
  double m = v[mid];
  if (v.size() % 2 == 0)
  {
    const double lower = *std::max_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid));
    m = 0.5 * (m + lower);
  }
  return m;
}

/// Fit, then iteratively drop pairs whose residual exceeds
/// max(min_threshold_m, k_mad * 1.4826 * MAD) and refit.
inline std::optional<RobustFitResult> RobustFitRigid2D(const std::vector<PointPair>& pairs,
                                                       int iterations = 3,
                                                       double k_mad = 3.0,
                                                       double min_threshold_m = 0.05)
{
  std::vector<bool> use(pairs.size(), true);
  auto fit = FitRigid2D(pairs, use);
  if (!fit)
    return std::nullopt;

  for (int it = 0; it < iterations; ++it)
  {
    std::vector<double> res(pairs.size());
    for (std::size_t i = 0; i < pairs.size(); ++i)
      res[i] = Residual(*fit, pairs[i]);
    const double med = Median(res);
    std::vector<double> dev(res.size());
    for (std::size_t i = 0; i < res.size(); ++i)
      dev[i] = std::abs(res[i] - med);
    const double sigma = 1.4826 * Median(dev);
    const double thr = std::max(min_threshold_m, med + k_mad * sigma);
    bool changed = false;
    for (std::size_t i = 0; i < pairs.size(); ++i)
    {
      const bool keep = res[i] <= thr;
      changed = changed || (keep != use[i]);
      use[i] = keep;
    }
    auto refit = FitRigid2D(pairs, use);
    if (!refit)
      return std::nullopt;
    fit = refit;
    if (!changed)
      break;
  }

  RobustFitResult r;
  r.transform = *fit;
  double sse = 0.0;
  std::vector<double> all;
  all.reserve(pairs.size());
  for (std::size_t i = 0; i < pairs.size(); ++i)
  {
    const double e = Residual(*fit, pairs[i]);
    all.push_back(e);
    if (!use[i])
      continue;
    ++r.inliers;
    sse += e * e;
    r.max_m = std::max(r.max_m, e);
  }
  if (r.inliers == 0)
    return std::nullopt;
  r.rms_m = std::sqrt(sse / static_cast<double>(r.inliers));
  r.median_m = Median(all);
  return r;
}

/// Standard deviations of the source points along their principal axes
/// (major >= minor). The rotation is only observable if the trajectory has
/// spread; with RTK noise sigma and spread L, the yaw error is ~sigma/L.
struct Spread
{
  double major_std_m = 0.0;
  double minor_std_m = 0.0;
};

inline Spread SourceSpread(const std::vector<PointPair>& pairs)
{
  Spread s;
  if (pairs.size() < 2)
    return s;
  double mx = 0.0, my = 0.0;
  for (const auto& p : pairs)
  {
    mx += p.src_x;
    my += p.src_y;
  }
  const double n = static_cast<double>(pairs.size());
  mx /= n;
  my /= n;
  double cxx = 0.0, cyy = 0.0, cxy = 0.0;
  for (const auto& p : pairs)
  {
    const double dx = p.src_x - mx;
    const double dy = p.src_y - my;
    cxx += dx * dx;
    cyy += dy * dy;
    cxy += dx * dy;
  }
  cxx /= n;
  cyy /= n;
  cxy /= n;
  const double tr = cxx + cyy;
  const double det = cxx * cyy - cxy * cxy;
  const double disc = std::sqrt(std::max(0.0, tr * tr / 4.0 - det));
  const double l1 = tr / 2.0 + disc;
  const double l2 = std::max(0.0, tr / 2.0 - disc);
  s.major_std_m = std::sqrt(std::max(0.0, l1));
  s.minor_std_m = std::sqrt(l2);
  return s;
}

/// Diagnostic fit that also ESTIMATES the antenna lever arm instead of
/// trusting the configured one. In complex notation (2D rotations commute):
///   dst_i = a * base_i + b * e^{i*yaw_i} + t
/// with a = s*e^{i*theta}, b = a*lever, which is linear in (a, b, t).
/// A lever arm far from the configured gps_x/gps_y — typically its mirror
/// image — means the LiDAR yaw is off (e.g. lidar_yaw by pi) or the lever
/// arm is misconfigured; a scale far from 1 means the map or GNSS is off.
/// Needs heading variation (turns) to separate the lever arm from t.
struct LeverFit
{
  double theta = 0.0;
  double scale = 1.0;
  double tx = 0.0;
  double ty = 0.0;
  double lever_x = 0.0;  // base frame
  double lever_y = 0.0;
  double rms_m = 0.0;
};

inline std::optional<LeverFit> FitWithLeverArm(const std::vector<PointPair>& pairs)
{
  if (pairs.size() < 6)
    return std::nullopt;
  // Unknowns: ar, ai, br, bi, tx, ty. Normal equations N x = r.
  double n_mat[6][7] = {};
  auto add_row = [&n_mat](const double row[6], double rhs)
  {
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
        n_mat[i][j] += row[i] * row[j];
      n_mat[i][6] += row[i] * rhs;
    }
  };
  for (const auto& p : pairs)
  {
    const double ux = std::cos(p.base_yaw);
    const double uy = std::sin(p.base_yaw);
    const double rx[6] = {p.base_x, -p.base_y, ux, -uy, 1.0, 0.0};
    const double ry[6] = {p.base_y, p.base_x, uy, ux, 0.0, 1.0};
    add_row(rx, p.dst_x);
    add_row(ry, p.dst_y);
  }
  // Gauss-Jordan with partial pivoting.
  for (int c = 0; c < 6; ++c)
  {
    int piv = c;
    for (int r = c + 1; r < 6; ++r)
      if (std::abs(n_mat[r][c]) > std::abs(n_mat[piv][c]))
        piv = r;
    if (std::abs(n_mat[piv][c]) < 1e-9)
      return std::nullopt;  // no heading variation / degenerate
    if (piv != c)
      for (int j = 0; j < 7; ++j)
        std::swap(n_mat[c][j], n_mat[piv][j]);
    const double inv = 1.0 / n_mat[c][c];
    for (int j = c; j < 7; ++j)
      n_mat[c][j] *= inv;
    for (int r = 0; r < 6; ++r)
    {
      if (r == c || n_mat[r][c] == 0.0)
        continue;
      const double f = n_mat[r][c];
      for (int j = c; j < 7; ++j)
        n_mat[r][j] -= f * n_mat[c][j];
    }
  }
  const double ar = n_mat[0][6], ai = n_mat[1][6], br = n_mat[2][6], bi = n_mat[3][6];
  LeverFit f;
  f.tx = n_mat[4][6];
  f.ty = n_mat[5][6];
  const double a2 = ar * ar + ai * ai;
  if (a2 < 1e-12)
    return std::nullopt;
  f.scale = std::sqrt(a2);
  f.theta = std::atan2(ai, ar);
  // lever = b / a = b * conj(a) / |a|^2
  f.lever_x = (br * ar + bi * ai) / a2;
  f.lever_y = (bi * ar - br * ai) / a2;
  double sse = 0.0;
  for (const auto& p : pairs)
  {
    const double ux = std::cos(p.base_yaw);
    const double uy = std::sin(p.base_yaw);
    const double px = ar * p.base_x - ai * p.base_y + br * ux - bi * uy + f.tx;
    const double py = ar * p.base_y + ai * p.base_x + br * uy + bi * ux + f.ty;
    sse += (px - p.dst_x) * (px - p.dst_x) + (py - p.dst_y) * (py - p.dst_y);
  }
  f.rms_m = std::sqrt(sse / static_cast<double>(pairs.size()));
  return f;
}

/// Planar pose sample in the lidar_map frame.
struct TimedPose
{
  double t = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

/// Linear interpolation of a time-ordered buffer at time t. Returns nullopt if
/// t is outside the buffer or the bracketing samples are more than max_gap_s
/// apart (a dropout — interpolating across it would invent motion).
inline std::optional<TimedPose> Interpolate(const std::deque<TimedPose>& buf,
                                            double t,
                                            double max_gap_s)
{
  if (buf.size() < 2 || t < buf.front().t || t > buf.back().t)
    return std::nullopt;
  const auto hi = std::lower_bound(buf.begin(),
                                   buf.end(),
                                   t,
                                   [](const TimedPose& p, double v)
                                   {
                                     return p.t < v;
                                   });
  if (hi == buf.end())
    return std::nullopt;
  if (hi->t == t)
    return *hi;
  if (hi == buf.begin())
    return std::nullopt;
  const auto lo = hi - 1;
  const double dt = hi->t - lo->t;
  if (dt <= 0.0 || dt > max_gap_s)
    return std::nullopt;
  const double a = (t - lo->t) / dt;
  TimedPose out;
  out.t = t;
  out.x = lo->x + a * (hi->x - lo->x);
  out.y = lo->y + a * (hi->y - lo->y);
  out.yaw = WrapAngle(lo->yaw + a * WrapAngle(hi->yaw - lo->yaw));
  return out;
}

/// Planar speed and yaw rate around time t from the bracketing samples
/// (used to reject pairs taken during fast motion, where a few ms of
/// timestamp skew between GNSS and LiDAR already costs centimetres).
inline std::optional<std::pair<double, double>> SpeedAndYawRate(const std::deque<TimedPose>& buf,
                                                                double t,
                                                                double window_s)
{
  if (buf.size() < 2)
    return std::nullopt;
  const TimedPose* a = nullptr;
  const TimedPose* b = nullptr;
  for (const auto& p : buf)
  {
    if (p.t >= t - window_s && a == nullptr)
      a = &p;
    if (p.t <= t + window_s)
      b = &p;
  }
  if (a == nullptr || b == nullptr || b->t - a->t < 1e-3)
    return std::nullopt;
  const double dt = b->t - a->t;
  return std::make_pair(std::hypot(b->x - a->x, b->y - a->y) / dt,
                        std::abs(WrapAngle(b->yaw - a->yaw)) / dt);
}

/// Antenna position from a base pose and the base-frame lever arm.
inline void AntennaFromBase(
    double x, double y, double yaw, double lever_x, double lever_y, double& ant_x, double& ant_y)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  ant_x = x + c * lever_x - s * lever_y;
  ant_y = y + s * lever_x + c * lever_y;
}

}  // namespace mowgli_localization::lidar_map_alignment
