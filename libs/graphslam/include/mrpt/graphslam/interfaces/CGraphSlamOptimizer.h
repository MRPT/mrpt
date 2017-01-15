/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CGRAPHOPTIMIZER_H
#define CGRAPHOPTIMIZER_H

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/utils/TParameters.h>
#include <mrpt/utils/CTimeLogger.h>

#include <mrpt/graphslam/misc/CWindowManager.h>
#include "CRegistrationDeciderOrOptimizer.h"

namespace mrpt { namespace graphslam { namespace optimizers {

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
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CGraphSlamOptimizer : public mrpt::graphslam::CRegistrationDeciderOrOptimizer<GRAPH_t> {
	public:
		typedef typename GRAPH_t::constraint_t constraint_t; // type of underlying constraints
		typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)

		CGraphSlamOptimizer() { }
		~CGraphSlamOptimizer() { }

		/**\brief Generic method for fetching the incremental action/observation
		 * readings from the calling function.
		 *
		 * Implementations of this interface should use (part of) the specified
		 * parameters and call the optimizeGraph function if the decision is to
		 * optimize the provided graph
		 *
		 * \return True if the optimization procedure was executed.
		 */
		virtual bool updateState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation ) = 0;

	protected:
		/**\brief method called for optimizing the underlying graph.
		 */
		virtual void optimizeGraph() = 0;

};

} } } // end of namespaces

#endif /* end of include guard: CGRAPHOPTIMIZER_H */
