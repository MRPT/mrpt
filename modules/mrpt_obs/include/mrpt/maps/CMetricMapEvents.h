/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/obs/obs_frwds.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/system/mrptEvent.h>

#include <optional>

namespace mrpt
{
namespace maps
{
/** Event emitted by a metric up upon call of clear()
 * \sa CMetricMap
 * \ingroup mrpt_obs_grp
 */
class mrptEventMetricMapClear : public mrpt::system::mrptEvent
{
 protected:
  /** Just to allow this class to be polymorphic */
  void do_nothing() override {}

 public:
  inline mrptEventMetricMapClear(const mrpt::maps::CMetricMap* smap) : source_map(smap) {}

  const mrpt::maps::CMetricMap* source_map;
};

/** Event emitted by a metric up upon a successful call to insertObservation()
 * \sa CMetricMap
 * \ingroup mrpt_obs_grp
 */
class mrptEventMetricMapInsert : public mrpt::system::mrptEvent
{
 protected:
  /** Just to allow this class to be polymorphic */
  void do_nothing() override {}

 public:
  mrptEventMetricMapInsert(
      const mrpt::maps::CMetricMap* smap,
      const mrpt::obs::CObservation* obs,
      const std::optional<const mrpt::poses::CPose3D>& robotPose) :
      source_map(smap), inserted_obs(obs), inserted_robotPose(robotPose)
  {
  }

  const mrpt::maps::CMetricMap* source_map;
  const mrpt::obs::CObservation* inserted_obs;
  const std::optional<const mrpt::poses::CPose3D> inserted_robotPose;
};

}  // namespace maps
}  // namespace mrpt
