/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>

namespace mrpt::apps
{
/** Virtual interface for offline datasets (rawlog) or live sensors.
 *
 * \ingroup mrpt_apps_grp
 */
class BaseAppDataSource
{
   public:
	BaseAppDataSource() = default;
	virtual ~BaseAppDataSource() = default;

   protected:
	/** Get next sensory data. Return false on any error, true if success. */
	virtual bool impl_get_next_observations(
		mrpt::obs::CActionCollection::Ptr& action,
		mrpt::obs::CSensoryFrame::Ptr& observations,
		mrpt::obs::CObservation::Ptr& observation) = 0;
};

}  // namespace mrpt::apps
