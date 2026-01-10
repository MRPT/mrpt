/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
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
