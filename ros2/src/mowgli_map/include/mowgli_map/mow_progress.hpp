// Copyright (C) 2026 MowgliNext contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#ifndef MOWGLI_MAP__MOW_PROGRESS_HPP_
#define MOWGLI_MAP__MOW_PROGRESS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace mowgli_map
{

enum class MowProgressInhibitReason
{
  kActive,
  kBladeNotRequested,
  kTelemetryMissingOrStale,
  kBladeInactive,
  kRpmTooLow,
};

/// Decide whether a pose may be added to the "actually mowed" overlay.
///
/// This deliberately requires both command intent and fresh blade-controller
/// evidence.  A missing or stale telemetry timestamp is never treated as a
/// positive observation.
inline MowProgressInhibitReason GetMowProgressInhibitReason(bool blade_requested,
                                                            bool telemetry_fresh,
                                                            bool blade_active,
                                                            double blade_rpm,
                                                            double min_blade_rpm)
{
  if (!blade_requested)
  {
    return MowProgressInhibitReason::kBladeNotRequested;
  }
  if (!telemetry_fresh)
  {
    return MowProgressInhibitReason::kTelemetryMissingOrStale;
  }
  if (!blade_active)
  {
    return MowProgressInhibitReason::kBladeInactive;
  }
  if (!std::isfinite(blade_rpm) || blade_rpm < min_blade_rpm)
  {
    return MowProgressInhibitReason::kRpmTooLow;
  }
  return MowProgressInhibitReason::kActive;
}

inline const char* ToString(MowProgressInhibitReason reason)
{
  switch (reason)
  {
    case MowProgressInhibitReason::kActive:
      return "verified blade activity";
    case MowProgressInhibitReason::kBladeNotRequested:
      return "blade not requested";
    case MowProgressInhibitReason::kTelemetryMissingOrStale:
      return "blade telemetry missing or stale";
    case MowProgressInhibitReason::kBladeInactive:
      return "blade controller reports inactive";
    case MowProgressInhibitReason::kRpmTooLow:
      return "blade RPM below threshold";
  }
  return "unknown";
}

/// Decide whether the ~/mow_progress overlay is due for publication.
///
/// The overlay is a full-extent OccupancyGrid (bounding box of every area plus
/// a 5 m margin, at map resolution), so every publication is O(cells) on the
/// wire: ~300 kB for a small garden, >1 MB for a large one. It is therefore
/// published when coverage changed (at most once per `publish_period_s`), plus
/// a slow keep-alive every `republish_period_s` for subscribers that missed the
/// transient_local sample (a foxglove_bridge reconnect). Republishing an
/// unchanged grid at `publish_period_s` saturated the WiFi link of every open
/// GUI tab even with the robot idle on the dock.
///
/// `republish_period_s <= 0` disables the keep-alive.
inline bool ShouldPublishMowProgress(bool never_published,
                                     bool dirty,
                                     double since_last_publish_s,
                                     double publish_period_s,
                                     double republish_period_s)
{
  if (never_published)
  {
    return true;
  }
  if (dirty)
  {
    return since_last_publish_s >= publish_period_s;
  }
  return republish_period_s > 0.0 && since_last_publish_s >= republish_period_s;
}

/// Number of evenly spaced disc centres needed to cover a segment without a
/// gap larger than one grid cell. The endpoint is included by callers.
inline std::size_t SweepStepCount(double distance, double resolution)
{
  return std::max<std::size_t>(1,
                               static_cast<std::size_t>(
                                   std::ceil(distance / std::max(resolution, 1e-9))));
}

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__MOW_PROGRESS_HPP_
