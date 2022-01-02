/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
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
	inline mrptEventMetricMapClear(const mrpt::maps::CMetricMap* smap)
		: source_map(smap)
	{
	}

	const mrpt::maps::CMetricMap* source_map;
};

/** Event emitted by a metric up upon a succesful call to insertObservation()
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
		const mrpt::maps::CMetricMap* smap, const mrpt::obs::CObservation* obs,
		const std::optional<const mrpt::poses::CPose3D>& robotPose)
		: source_map(smap), inserted_obs(obs), inserted_robotPose(robotPose)
	{
	}

	const mrpt::maps::CMetricMap* source_map;
	const mrpt::obs::CObservation* inserted_obs;
	const std::optional<const mrpt::poses::CPose3D> inserted_robotPose;
};

}  // namespace maps
}  // namespace mrpt
