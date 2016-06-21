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

	 * TODO - implement a filter for using the Odometry data as well, if given
	 * TODO - add to this description
	 */
	template<class GRAPH_t>
		class CICPGoodnessNRD_t:
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


		  CICPGoodnessNRD_t();
		  ~CICPGoodnessNRD_t();

			/**
		 	 * Initialize the graph to be used for the node registration procedure
		 	 */
			void setGraphPtr(GRAPH_t* graph);
			bool updateDeciderState( mrpt::obs::CActionCollectionPtr action,
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

					// ICP object for aligning laser scans
					mrpt::slam::CICP icp;
					// threshold for considering the ICP procedure as correct
					double ICP_goodness_thresh;

					// parameters for conversion 3D=>2D Range Scan
					std::string conversion_sensor_label;
					double conversion_angle_sup;
					double conversion_angle_inf;
					double conversion_oversampling_ratio;

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
			/**
			 * allign the 2D range scans provided and fill the potential edge that
			 * can transform the one into the other
			 */
			double getICPEdge(
					const CObservation2DRangeScan& prev_laser_scan,
					const CObservation2DRangeScan& curr_laser_scan,
					constraint_t* rel_edge );
			/**
			 * Method that updates the m_last_laser_scan2D variable given a
			 * CObservation3DRangeScan acquired from an RGB-D camera. If inserted
			 * 3DRangeScan does not contain valid data, a warning message is printed.
			 */
			void convert3DTo2DRangeScan(
					/*from = */ CObservation3DRangeScanPtr& scan3D_in,
					/*to   = */ CObservation2DRangeScanPtr* scan2D_out=NULL);

			// Private members
			//////////////////////////////////////////////////////////////
			GRAPH_t* m_graph;
			mrpt::gui::CDisplayWindow3D* m_win;

			// Tracking the PDF of the current position of the robot with regards to the
			// *previous* registered node
			constraint_t	m_since_prev_node_PDF;

			pose_t m_curr_estimated_pose;
			// variable to keep track of whether we are reading from an
			// observation-only rawlog file or from an action-observation rawlog
			bool m_observation_only_rawlog;
			// if invalid format instance number surpasses this threshold print a
			// warning message to the user
			bool m_num_invalid_format_instances;

			bool m_first_time_call;

			// handy laser scans to use in the class methods
    	CObservation2DRangeScanPtr m_last_laser_scan2D;
    	CObservation2DRangeScanPtr m_curr_laser_scan2D;

    	CObservation3DRangeScanPtr m_curr_laser_scan3D;
    	
    	// last insertede node in the graph
			mrpt::utils::TNodeID m_nodeID_max;

	};


} } }

#include "CICPGoodnessNRD_impl.h"
#endif /* end of include guard: CICPGOODNESSNRD_H */
