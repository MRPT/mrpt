/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CICPCRITERIANRD_H
#define CICPCRITERIANRD_H

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
#include <mrpt/obs/CObservationOdometry.h>
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

#include <mrpt/graphslam/interfaces/CNodeRegistrationDecider.h>
#include <mrpt/graphslam/misc/CRangeScanOps.h>
#include <mrpt/graphslam/misc/TSlidingWindow.h>

#include <string>
#include <math.h>

namespace mrpt { namespace graphslam { namespace deciders {

/**\brief ICP-based Fixed Intervals Node Registration
 *
 * ## Description
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
 * Decider *does not guarantee* thread safety when accessing the GRAPH_T
 * resource. This is handled by the CGraphSlamEngine class.
 *
 * ### Specifications
 *
 * - Map type: 2D
 * - MRPT rawlog format: #1, #2
 * - Graph Type: CPosePDFGaussianInf
 * - Observations Used: CObservation2DRangeScan, CObservation3DRangeScan
 * - Node Registration Strategy: Fixed Intervals
 *
 * ### .ini Configuration Parameters
 *
 * \htmlinclude config_params_preamble.txt
 *
 * - \b class_verbosity
 *   + \a Section       : NodeRegistrationDeciderParameters
 *   + \a default value : 1 (LVL_INFO)
 *   + \a Required      : FALSE
 *
 * - \b registration_max_distance
 *  + \a Section       : NodeRegistrationDeciderParameters
 *  + \a Default value : 0.5 // meters
 *  + \a Required      : FALSE
 *
 * - \b registration_max_angle
 *  + \a Section       : NodeRegistrationDeciderParameters
 *  + \a Default value : 10 // degrees
 *  + \a Required      : FALSE
 *
 * \note Since the decider inherits from the CRangeScanOps
 * class, it parses the configuration parameters of the latter as well from the
 * "ICP" section. Refer to the CRangeScanOps documentation for
 * its list of configuration
 * parameters
 *
 * \note Class contains an instance of the TSlidingWindow class and it parses
 * the configuration parameters of the latter from the
 * "NodeRegistrationDeciderParameters" section. Refer to TSlidingWindow
 * documentation for its list of configuration parameters
 *
 * \ingroup mrpt_graphslam_grp
 */
template<class GRAPH_T>
class CICPCriteriaNRD:
	public virtual mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>,
	public mrpt::graphslam::deciders::CRangeScanOps<GRAPH_T>
{
	public:
		// Public functions
		//////////////////////////////////////////////////////////////
		/**\brief Handy typedefs */
		/**\{*/
		/**\brief type of graph constraints */
		typedef typename GRAPH_T::constraint_t constraint_t;
		/**\brief type of underlying poses (2D/3D). */
		typedef typename GRAPH_T::constraint_t::type_value pose_t;
		typedef typename GRAPH_T::global_pose_t global_pose_t;

		typedef mrpt::math::CMatrixFixedNumeric<double,
						constraint_t::state_length,
						constraint_t::state_length> InfMat;
		/**\brief Typedef for accessing methods of the RangeScanRegistrationDecider
		 * parent class.
		 */
		typedef mrpt::graphslam::deciders::CRangeScanOps<GRAPH_T>
			range_ops_t;
		typedef CICPCriteriaNRD<GRAPH_T> decider_t; /**< self type - Handy typedef */
		/**\brief Node Registration Decider */
		typedef mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T> parent_t;
		/**\}*/

		/**\brief Class constructor */
		CICPCriteriaNRD();
		/**\brief Class destructor */
		~CICPCriteriaNRD();

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
		 * Order of calls:
		 * updateState (calls) ==> updateState2D/3D ==> 
		 * checkRegistrationCondition2D/3D ==> CheckRegistrationCondition
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
		bool updateState2D(mrpt::obs::CObservation2DRangeScanPtr observation);
		/**\brief Specialized updateState method used solely when dealing with
		 * 3DRangeScan information.
		 * \sa updateState2D
		 */
		bool updateState3D(mrpt::obs::CObservation3DRangeScanPtr observation);

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

	protected:
		// protected functions
		//////////////////////////////////////////////////////////////
		bool checkRegistrationCondition();
		/**\brief Specialized checkRegistrationCondtion method used solely when
		 * dealing with 2DRangeScan information
		 * \sa checkRegistrationCondition3D
		 */
		bool checkRegistrationCondition2D();
		/**\brief Specialized checkRegistrationCondition method used solely when
		 * dealing with 3DRangeScan information
		 * \sa checkRegistrationCondition2D
		 */
		bool checkRegistrationCondition3D();

		// protected members
		//////////////////////////////////////////////////////////////
		bool m_is_using_3DScan;

		/**\brief handy laser scans to use in the class methods
		 */
		 /**\{ */
		/**\brief 2D LaserScan corresponding to the latest registered node in the graph */
		mrpt::obs::CObservation2DRangeScanPtr m_last_laser_scan2D;
		/**\brief Current LaserScan. Set during the new measurements acquisition in
		 * updateState method
		 */
		mrpt::obs::CObservation2DRangeScanPtr m_curr_laser_scan2D;

		mrpt::obs::CObservation3DRangeScanPtr m_last_laser_scan3D;
		mrpt::obs::CObservation3DRangeScanPtr m_curr_laser_scan3D;
		 /**\} */

		/**\brief Odometry rigid-body transformation since the last accepted LaserScan.
		 *
		 * Decider can use it to smoothen the trajectory in the case of high noise
		 * in the laser measurements
		 */
		constraint_t m_latest_odometry_PDF;
		/**\brief pose_t estimation using only odometry information.
		 * \note Utilized only in observation-only rawlogs.
		 *
		 */
		pose_t m_curr_odometry_only_pose;
		/**\brief pose_t estimation using only odometry information.
		 * \note Utilized only in observation-only rawlogs.
		 *
		 * Resets next time an ICP edge/Odometry measurement is utilized for
		 * updating the estimated robot position.
		 */
		pose_t m_last_odometry_only_pose;
		/**\brief Keeps track of the last N measurements between the ICP edge and
		 * the corresponding odometry measurements.
		 *
		 * Use the last odometry rigid body transformation instead of the
		 * ICP edge if the mahalanobis distance between them is greater than this
		 * limit.
		 */
		TSlidingWindow m_mahal_distance_ICP_odom;

		// criteria for adding new a new node
		bool m_use_angle_difference_node_reg;
		bool m_use_distance_node_reg;

		/**How many times we used the ICP Edge instead of Odometry edge*/
		int m_times_used_ICP;
		/**How many times we used the Odometry Edge instead of the ICP edge */
		int m_times_used_odom;
};


} } } // end of namespaces

#include "CICPCriteriaNRD_impl.h"
#endif /* end of include guard: CICPCRITERIANRD_H */
