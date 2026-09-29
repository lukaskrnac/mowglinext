// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cmath>
#include <deque>
#include <random>
#include <vector>

#include "mowgli_localization/lidar_map_alignment.hpp"
#include <gtest/gtest.h>

namespace lma = mowgli_localization::lidar_map_alignment;

namespace
{
// Figure-of-eight-ish trajectory in the lidar_map frame, mapped into the
// map frame through `truth`, with optional Gaussian noise on the map side.
std::vector<lma::PointPair> MakePairs(const lma::Rigid2D& truth,
                                      std::size_t n,
                                      double noise_m,
                                      unsigned seed = 42)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, noise_m);
  std::vector<lma::PointPair> out;
  for (std::size_t i = 0; i < n; ++i)
  {
    const double s = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
    lma::PointPair p;
    p.src_x = 12.0 + 8.0 * std::sin(s);
    p.src_y = -3.0 + 5.0 * std::sin(2.0 * s);
    truth.Apply(p.src_x, p.src_y, p.dst_x, p.dst_y);
    if (noise_m > 0.0)
    {
      p.dst_x += noise(rng);
      p.dst_y += noise(rng);
    }
    out.push_back(p);
  }
  return out;
}
}  // namespace

TEST(LidarMapAlignment, ExactRecovery)
{
  const lma::Rigid2D truth{0.7, -4.2, 11.5};
  const auto pairs = MakePairs(truth, 50, 0.0);
  const auto fit = lma::FitRigid2D(pairs);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->theta, truth.theta, 1e-9);
  EXPECT_NEAR(fit->tx, truth.tx, 1e-9);
  EXPECT_NEAR(fit->ty, truth.ty, 1e-9);
}

TEST(LidarMapAlignment, RecoversLargeRotationWithoutWrapIssues)
{
  const lma::Rigid2D truth{-3.0, 100.0, -50.0};
  const auto fit = lma::FitRigid2D(MakePairs(truth, 30, 0.0));
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(lma::WrapAngle(fit->theta - truth.theta), 0.0, 1e-9);
  EXPECT_NEAR(fit->tx, truth.tx, 1e-7);
  EXPECT_NEAR(fit->ty, truth.ty, 1e-7);
}

TEST(LidarMapAlignment, NoisyFitIsCentimetreAccurate)
{
  const lma::Rigid2D truth{0.3, 2.0, -1.0};
  const auto res = lma::RobustFitRigid2D(MakePairs(truth, 400, 0.015));
  ASSERT_TRUE(res.has_value());
  EXPECT_NEAR(res->transform.theta, truth.theta, 0.002);  // ~0.1 deg
  EXPECT_NEAR(res->transform.tx, truth.tx, 0.01);
  EXPECT_NEAR(res->transform.ty, truth.ty, 0.01);
  EXPECT_LT(res->rms_m, 0.03);
}

TEST(LidarMapAlignment, RobustFitRejectsOutliers)
{
  const lma::Rigid2D truth{1.2, -3.0, 4.0};
  auto pairs = MakePairs(truth, 300, 0.01);
  // 10 % gross outliers (a wrong-fix burst or an NDT mis-lock).
  for (std::size_t i = 0; i < pairs.size(); i += 10)
    pairs[i].dst_x += 2.5;
  const auto res = lma::RobustFitRigid2D(pairs);
  ASSERT_TRUE(res.has_value());
  EXPECT_NEAR(res->transform.theta, truth.theta, 0.003);
  EXPECT_NEAR(res->transform.tx, truth.tx, 0.02);
  EXPECT_NEAR(res->transform.ty, truth.ty, 0.02);
  EXPECT_LE(res->inliers, 270u);
  EXPECT_LT(res->rms_m, 0.03);
}

TEST(LidarMapAlignment, DegenerateInputsReturnNullopt)
{
  EXPECT_FALSE(lma::FitRigid2D({}).has_value());
  std::vector<lma::PointPair> one{{1, 2, 3, 4}};
  EXPECT_FALSE(lma::FitRigid2D(one).has_value());
  std::vector<lma::PointPair> same{{1, 1, 5, 5}, {1, 1, 5, 5}, {1, 1, 5, 5}};
  EXPECT_FALSE(lma::FitRigid2D(same).has_value());
}

TEST(LidarMapAlignment, InverseRoundTrips)
{
  const lma::Rigid2D t{0.9, 3.0, -7.0};
  const auto inv = t.Inverse();
  double x = 0, y = 0, bx = 0, by = 0;
  t.Apply(1.5, -2.5, x, y);
  inv.Apply(x, y, bx, by);
  EXPECT_NEAR(bx, 1.5, 1e-12);
  EXPECT_NEAR(by, -2.5, 1e-12);
}

TEST(LidarMapAlignment, SpreadOfStraightLineHasNoMinorAxis)
{
  std::vector<lma::PointPair> line;
  for (int i = 0; i < 100; ++i)
    line.push_back({i * 0.1, i * 0.1, 0, 0});
  const auto s = lma::SourceSpread(line);
  EXPECT_GT(s.major_std_m, 2.0);
  EXPECT_NEAR(s.minor_std_m, 0.0, 1e-6);

  const auto s8 = lma::SourceSpread(MakePairs({0, 0, 0}, 200, 0.0));
  EXPECT_GT(s8.minor_std_m, 3.0);
}

TEST(LidarMapAlignment, InterpolateAndGaps)
{
  std::deque<lma::TimedPose> buf{{0.0, 0.0, 0.0, 3.1}, {0.1, 1.0, 2.0, -3.1}, {1.0, 5.0, 5.0, 0.0}};
  const auto mid = lma::Interpolate(buf, 0.05, 0.2);
  ASSERT_TRUE(mid.has_value());
  EXPECT_NEAR(mid->x, 0.5, 1e-12);
  EXPECT_NEAR(mid->y, 1.0, 1e-12);
  // Yaw interpolates across the ±pi seam, not through 0.
  EXPECT_GT(std::abs(mid->yaw), 3.1);
  // 0.1 -> 1.0 is a 0.9 s dropout.
  EXPECT_FALSE(lma::Interpolate(buf, 0.5, 0.2).has_value());
  EXPECT_FALSE(lma::Interpolate(buf, -0.1, 0.2).has_value());
  EXPECT_FALSE(lma::Interpolate(buf, 1.1, 0.2).has_value());
}

TEST(LidarMapAlignment, SpeedAndYawRate)
{
  std::deque<lma::TimedPose> buf;
  for (int i = 0; i <= 10; ++i)
    buf.push_back({i * 0.1, i * 0.03, 0.0, i * 0.02});
  const auto v = lma::SpeedAndYawRate(buf, 0.5, 0.2);
  ASSERT_TRUE(v.has_value());
  EXPECT_NEAR(v->first, 0.3, 1e-9);
  EXPECT_NEAR(v->second, 0.2, 1e-9);
}

TEST(LidarMapAlignment, AntennaLeverArmRotatesWithYaw)
{
  double ax = 0, ay = 0;
  lma::AntennaFromBase(1.0, 1.0, M_PI / 2.0, 0.3, 0.0, ax, ay);
  EXPECT_NEAR(ax, 1.0, 1e-12);
  EXPECT_NEAR(ay, 1.3, 1e-12);
}

TEST(LidarMapAlignment, LeverArmDiagnosticRecoversRealLeverArm)
{
  // True antenna 0.3 m ahead of base; poses along a figure of eight with
  // changing heading.
  const lma::Rigid2D truth{2.8, -0.6, -0.2};
  std::vector<lma::PointPair> pairs;
  for (int i = 0; i < 200; ++i)
  {
    const double s = 2.0 * M_PI * i / 200.0;
    lma::PointPair p;
    p.base_x = 4.0 * std::sin(s);
    p.base_y = 2.5 * std::sin(2.0 * s);
    p.base_yaw = std::atan2(5.0 * std::cos(2.0 * s), 4.0 * std::cos(s));
    double ax = 0, ay = 0;
    lma::AntennaFromBase(p.base_x, p.base_y, p.base_yaw, 0.3, 0.0, ax, ay);
    truth.Apply(ax, ay, p.dst_x, p.dst_y);
    pairs.push_back(p);
  }
  const auto f = lma::FitWithLeverArm(pairs);
  ASSERT_TRUE(f.has_value());
  EXPECT_NEAR(f->lever_x, 0.3, 1e-6);
  EXPECT_NEAR(f->lever_y, 0.0, 1e-6);
  EXPECT_NEAR(f->scale, 1.0, 1e-6);
  EXPECT_NEAR(lma::WrapAngle(f->theta - truth.theta), 0.0, 1e-6);
  EXPECT_LT(f->rms_m, 1e-6);

  // Same data, LiDAR yaw reported off by pi: the lever arm comes out mirrored.
  for (auto& p : pairs)
    p.base_yaw = lma::WrapAngle(p.base_yaw + M_PI);
  const auto g = lma::FitWithLeverArm(pairs);
  ASSERT_TRUE(g.has_value());
  EXPECT_NEAR(g->lever_x, -0.3, 1e-6);
}
