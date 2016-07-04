/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CICPGOODNESSNRD_H
#define CICPGOODNESSNRD_H

#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/system/os.h>

#include "CNodeRegistrationDecider.h"
#include "CRangeScanRegistrationDecider.h"

#include <string>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::obs;

using namespace std;

namespace mrpt { namespace graphslam { namespace deciders {

/**
	* Map type: 2D
	* MRPT rawlog format: #1, #2
	* TODO - update this.
	* Observations: CObservation2DRangeScan, CObservation3DRangeScan
	* Node Registration Strategy: Fixed space intervals
	*
	* Node Registration scheme based on fixed intervals node regitration,
	* provided range scans (either standard 2D, or RGB-D 3D). If used offline,
	* use it with datasets in observation-only rawlog format.
	*
	* Current Decider is meant for adding nodes in 2D datasets recorded using
	* a laser range finder or RGB-D camera (e.g. Kinect). No odometry data from
	* encoders is needed.
	* Decider *does not guarrantee* thread safety when accessing the GRAPH_t
 	* resource. This is handled by the CGraphSlamEngine_t class.
	*/
template<class GRAPH_t>
	class CICPGoodnessNRD_t:
		public mrpt::graphslam::deciders::CNodeRegistrationDecider_t<GRAPH_t>,
		public mrpt::graphslam::deciders::CRangeScanRegistrationDecider_t<GRAPH_t>
{
	public:
		// Public functions
		//////////////////////////////////////////////////////////////

		typedef mrpt::graphslam::deciders::CNodeRegistrationDecider_t<GRAPH_t> superA;
		typedef mrpt::graphslam::deciders::CRangeScanRegistrationDecider_t<GRAPH_t> superB;

		typedef typename GRAPH_t::constraint_t constraint_t;
		typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)
		typedef mrpt::math::CMatrixFixedNumeric<double,
						constraint_t::state_length,
						constraint_t::state_length> InfMat;
		typedef mrpt::graphslam::deciders::CRangeScanRegistrationDecider_t<GRAPH_t> range_scanner_t;
		typedef CICPGoodnessNRD_t<GRAPH_t> decider_t;


		CICPGoodnessNRD_t();
		~CICPGoodnessNRD_t();

		/**
		 	* Initialize the graph to be used for the node registration procedure
		 	*/
		void setGraphPtr(GRAPH_t* graph);
		void loadParams(const std::string& source_fname);
		void printParams() const; 

		bool updateDeciderState( mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );

    struct TParams: public mrpt::utils::CLoadableOptions {
    	public:
    		TParams(decider_t& d);
    		~TParams();

				decider_t& decider; // reference to outer class

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
		bool checkRegistrationCondition();

		void registerNewNode();
		void initCICPGoodnessNRD_t();

		// Private members
		//////////////////////////////////////////////////////////////
		GRAPH_t* m_graph;

		// Tracking the PDF of the current position of the robot with regards to
		// the *previous* registered node
		constraint_t	m_since_prev_node_PDF;

		// variable to keep track of whether we are reading from an
		// observation-only rawlog file or from an action-observation rawlog
		bool m_observation_only_rawlog;
		// if invalid format instance number surpasses this threshold print a
		// warning message to the user
		bool m_num_invalid_format_instances;

		bool m_first_time_call2D;
		bool m_first_time_call3D;
		bool m_is_using_3DScan;

		// handy laser scans to use in the class methods
    CObservation2DRangeScanPtr m_last_laser_scan2D;
    CObservation2DRangeScanPtr m_curr_laser_scan2D;

    CObservation3DRangeScanPtr m_last_laser_scan3D;
    CObservation3DRangeScanPtr m_curr_laser_scan3D;

    // last insertede node in the graph
		mrpt::utils::TNodeID m_nodeID_max;

		typename superB::TSlidingWindow sliding_win;

};


} } }

#include "CICPGoodnessNRD_impl.h"
#endif /* end of include guard: CICPGOODNESSNRD_H */
