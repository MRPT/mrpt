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

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>

#include "CRegistrationDeciderOrOptimizer.h"

namespace mrpt::graphslam::optimizers
{
/**\brief Interface for implementing graphSLAM optimizer classes.
 *
 * Class should provide a generic interface from which real optimizers should
 * inherit so that they abide to the necessary method calls used in the
 * CGraphSlamEngine class. For an example of inheriting from this class see
 * CLevMarqGSO.
 *
 * \note As a naming convention, all the implemented graphslam optimizer classes
 * are suffixed with the GSO acronym.
 *
 * \ingroup mrpt_graphslam_grp
 */
template <class GRAPH_t = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CGraphSlamOptimizer : public virtual mrpt::graphslam::CRegistrationDeciderOrOptimizer<GRAPH_t>
{
 public:
  using constraint_t = typename GRAPH_t::constraint_t;
  using pose_t = typename GRAPH_t::constraint_t::type_value;

  CGraphSlamOptimizer() = default;
  ~CGraphSlamOptimizer() override = default;
  /**\brief Used by the caller to query for possible full graph optimization
   * on the latest optimizer run
   */
  virtual bool justFullyOptimizedGraph() const { return false; }

 protected:
  /**\brief method called for optimizing the underlying graph.
   */
  virtual void optimizeGraph() = 0;
};
}  // namespace mrpt::graphslam::optimizers
