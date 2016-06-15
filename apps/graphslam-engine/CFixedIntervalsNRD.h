/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CFIXEDINTERVALSNRD_H
#define CFIXEDINTERVALSNRD_H



#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/types_simple.h>

#include "CNodeRegistrationDecider.h"

#include <iostream>

// TODO - change these
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::obs;

using namespace std;

namespace mrpt { namespace graphslam { namespace deciders {

	/**
 	 * Fixed intervals odometry edge insertion. Determine whether to insert a new
 	 * pose in the graph given the distance and angle thresholds
 	 *
 	 * Current Decider is a minimal, simple implementation of the
 	 * CNodeRegistrationDecider_t interface which can be used for 2D datasets.
 	 * Decider *does not guarrantee* thread safety. This is
 	 * handled by the CGraphSlamEngine_t class.
 	 */
	template<class GRAPH_t>
		class CFixedIntervalsNRD_t:
			public mrpt::graphslam::deciders::CNodeRegistrationDecider_t<GRAPH_t>
	{
		public:
			// Public functions
			//////////////////////////////////////////////////////////////

			typedef typename GRAPH_t::constraint_t constraint_t;
			typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)
			typedef mrpt::math::CMatrixFixedNumeric<double,
							constraint_t::state_length,
							constraint_t::state_length> InfMat;

			CFixedIntervalsNRD_t(GRAPH_t* graph);
			CFixedIntervalsNRD_t();
			~CFixedIntervalsNRD_t();

			/**
		 	 * Initialize the graph to be used for the node registration procedure
		 	 */
			void setGraphPtr(GRAPH_t* graph);

			/**
		 	 * Make use of the CActionCollection to update the odometry estimation from
		 	 * the last inserted pose
		 	 */
			virtual bool updateDeciderState( mrpt::obs::CActionCollectionPtr action,
					mrpt::obs::CSensoryFramePtr observations,
					mrpt::obs::CObservationPtr observation );

    	struct TParams: public mrpt::utils::CLoadableOptions {
    		public:
    			TParams();
    			~TParams();

    			void loadFromConfigFile(
    					const mrpt::utils::CConfigFileBase &source,
    					const std::string &section);
					void 	dumpToTextStream(mrpt::utils::CStream &out) const;

					// max values for new node registration
					double registration_max_distance;
					double registration_max_angle;
    	};

			// Public members
			// ////////////////////////////
    	TParams params;

		private:
			// Private functions
			//////////////////////////////////////////////////////////////
			/**
		 	 * If estimated position surpasses the registration max values since the
		 	 * previous registered node, register a new node in the graph.
		 	 *
		 	 * Returns new on successful registration.
		 	 */
			bool checkRegistrationCondition();
			bool registerNewNode();
			/**
		 	 * Initialization function to be called from the various constructors
		 	 */
			void initCFixedIntervalsNRD_t();

			// Private members
			//////////////////////////////////////////////////////////////
			GRAPH_t* m_graph;
			mrpt::gui::CDisplayWindow3D* m_win;
			// store the last registered node - not his pose since it will most likely
			// change during calls to the graph-optimization procedure /
			// dijkstra_node_estimation
			TNodeID m_prev_registered_node;

			constraint_t	m_since_prev_node_PDF;

			pose_t m_curr_estimated_pose;
			// pose_t using only odometry information. Handy for observation-only
			// rawlogs.
			pose_t m_curr_odometry_only_pose; 
			pose_t m_last_odometry_only_pose; 
			// variable to keep track of whether we are reading from an
			// observation-only rawlog file or from an action-observation rawlog
			bool m_observation_only_rawlog;
	};

} } } // end of namespaces

#include "CFixedIntervalsNRD_impl.h"
#endif /* end of include guard: CFIXEDINTERVALSNRD_H */
