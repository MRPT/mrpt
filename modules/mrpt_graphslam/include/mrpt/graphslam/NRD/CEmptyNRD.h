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

#include <mrpt/graphslam/interfaces/CNodeRegistrationDecider.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>

namespace mrpt::graphslam::deciders
{
/**\brief Empty Node Registration Decider
 *
 * Handy when you are testing other parts of the application but not the
 * specific registration procedure
 *
 * \ingroup mrpt_graphslam_grp
 */
template <class GRAPH_T = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CEmptyNRD : public mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>
{
  /**\brief Handy typedefs */
  /**\{*/
  using pose_t = typename GRAPH_T::constraint_t::type_value;
  using global_pose_t = typename GRAPH_T::global_pose_t;
  /**\}*/
 public:
  CEmptyNRD() = default;
  ~CEmptyNRD() override = default;

  bool updateState(
      mrpt::obs::CActionCollection::Ptr action,
      mrpt::obs::CSensoryFrame::Ptr observations,
      mrpt::obs::CObservation::Ptr observation) override
  {
    return false;
  }
  global_pose_t getCurrentRobotPosEstimation() const override
  {
    return typename GRAPH_T::global_pose_t();
  };

 private:
  void registerNewNode(){};
};
}  // namespace mrpt::graphslam::deciders
