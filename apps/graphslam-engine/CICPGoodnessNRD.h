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
#include <mrpt/poses/CRobot2DPoseEstimator.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>

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

/**\brief Fixed Intervals ICP-based Node Registration
 * \b Description
 *
 * Current Decider is meant for adding nodes in 2D datasets recorded using
 * a laser range finder or RGB-D camera (e.g. Kinect). No odometry data from
 * encoders is needed. Using ICP to match consecutive RangeScan measurements,
 * the decider keeps track of the pose transformation since the last registered
 * node. If the norm or the angle of the latter surpasses certain thresholds
 * (which are read from an external .ini file) then a new node is added to the
 * graph)
 * \sa loadParams, TParams::loadFromConfigFile
 *
 * Decider *does not guarrantee* thread safety when accessing the GRAPH_t
 * resource. This is handled by the CGraphSlamEngine_t class.
 *
 * \b Specifications
 *
 * - Map type: 2D
 * - MRPT rawlog format: #1, #2
 * - Observations Used: CObservation2DRangeScan, CObservation3DRangeScan
 * - Node Registration Strategy: Fixed Intervals
 *
 * \ingroup mrpt_graphslam_grp
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

		/**\brief type of graph constraints */
		typedef typename GRAPH_t::constraint_t constraint_t;
		/**\brief type of underlying poses (2D/3D). */
		typedef typename GRAPH_t::constraint_t::type_value pose_t;

		typedef mrpt::math::CMatrixFixedNumeric<double,
						constraint_t::state_length,
						constraint_t::state_length> InfMat;
		/**\brief Typedef for accessing methods of the RangeScanRegistrationDecider_t parent class. */
		typedef mrpt::graphslam::deciders::CRangeScanRegistrationDecider_t<GRAPH_t> range_scanner_t;
		typedef CICPGoodnessNRD_t<GRAPH_t> decider_t; /**< self type - Handy typedef */

		/**\brief Class constructor */
		CICPGoodnessNRD_t();
		/**\brief Class destructor */
		~CICPGoodnessNRD_t();

		void setGraphPtr(GRAPH_t* graph);
		void loadParams(const std::string& source_fname);
		void printParams() const; 
		void getDescriptiveReport(std::string* report_str) const; 

		/**\brief Update the decider state using the latest dataset measurements.
		 *
		 * \note Depending on the observations at hand, update of the state is
		 * handled either by updateState2D, or by updateState3D methods. This
		 * helps in separating the 2D, 3D RangeScans handling altogether, which in
		 * turn simplifies the overall procedure
		 *
		 * \sa updateState2D, updateState3D
		 */
		bool updateState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );
		/**\brief Specialized updateState method used solely when dealing with
		 * 2DRangeScan information.
		 * \sa updateState3D
		 */
		bool updateState2D(
				mrpt::obs::CObservation2DRangeScanPtr observation); 
		/**\brief Specialized updateState method used solely when dealing with
		 * 3DRangeScan information.
		 * \sa updateState2D
		 */
		bool updateState3D(
				mrpt::obs::CObservation3DRangeScanPtr observation);

    struct TParams: public mrpt::utils::CLoadableOptions {
    	public:
    		TParams(decider_t& d);
    		~TParams();

				decider_t& decider; /**< Reference to outer decider class */

    		void loadFromConfigFile(
    				const mrpt::utils::CConfigFileBase &source,
    				const std::string &section);
				void 	dumpToTextStream(mrpt::utils::CStream &out) const;

				double registration_max_distance; /**< Maximum distance for new node registration */
				double registration_max_angle; /**< Maximum angle difference for new node registration */
    };

		// Public members
		// ////////////////////////////
		TParams params;

	private:
		// Private functions
		//////////////////////////////////////////////////////////////
		bool checkRegistrationCondition();
		/**\brief Specialized checkRegistrationCondtion method used solely when dealing with
		 * 2DRangeScan information
		 * \sa checkRegistrationCondition3D
		 */
		bool checkRegistrationCondition2D();
		/**\brief Specialized checkRegistrationCondition method used solely when dealing with
		 * 3DRangeScan information
		 * \sa checkRegistrationCondition2D
		 */
		bool checkRegistrationCondition3D();

		void registerNewNode();
		/**\brief General initialization method to call from the Class Constructors*/
		void initCICPGoodnessNRD_t();

		// Private members
		//////////////////////////////////////////////////////////////
		GRAPH_t* m_graph; /**<\brief Pointer to the graph under construction */

		/**\brief Tracking the PDF of the current position of the robot with regards to
		 * the \b previous registered node
		 */
		constraint_t	m_since_prev_node_PDF;

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

		// sliding window for managing the ICP goodness values
		typename superB::TSlidingWindow m_ICP_sliding_win;
		mrpt::system::TTimeStamp m_curr_timestamp;
		mrpt::system::TTimeStamp m_prev_timestamp;

		mrpt::utils::COutputLogger m_out_logger; /**<Output logger instance */
		mrpt::utils::CTimeLogger m_time_logger; /**<Time logger instance */

		// criterions for adding new a new node
		bool m_use_angle_difference_node_reg = true;
		bool m_use_distance_node_reg = true;


};


} } }

#include "CICPGoodnessNRD_impl.h"
#endif /* end of include guard: CICPGOODNESSNRD_H */
