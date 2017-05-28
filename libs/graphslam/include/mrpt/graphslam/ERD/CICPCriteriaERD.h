/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CICPCRITERIAERD_H
#define CICPCRITERIAERD_H

#include <mrpt/math/CMatrix.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/utils/TColor.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/threads.h>

#include <stdlib.h> // abs

#include <iostream>
#include <map>
#include <string>

#include <mrpt/graphslam/interfaces/CRangeScanEdgeRegistrationDecider.h>


namespace mrpt { namespace graphslam { namespace deciders {

/**\brief ICP-based Edge Registration.
 *
 * ## Description
 *
 * Register new edges in the graph with the last inserted node. Criterion for
 * adding new edges should be the goodness of the candidate ICP edge. The
 * nodes for ICP are picked <em>based on the distance from the last
 * inserted node</em>.
 * \sa  getNearbyNodesOf
 *
 * ### Specifications
 *
 * - Map type: 2D
 * - MRPT rawlog format: #1, #2
 * - Graph Type: CPosePDFGaussianInf
 * - Observations: CObservation2DRangeScan, CObservation3DRangeScan
 * - Edge Registration Strategy: ICP Goodnesss threshold
 *
 * ### .ini Configuration Parameters
 *
 * \htmlinclude graphslam-engine_config_params_preamble.txt
 *
 * - \b class_verbosity
 *   + \a Section       : EdgeRegistrationDeciderParameters
 *   + \a default value : 1 (LVL_INFO)
 *   + \a Required      : FALSE
 *
 * - \b LC_min_nodeid_diff
 *  + \a Section       : GeneralConfiguration
 *  + \a Default value : 30
 *  + \a Required      : FALSE
 *  + \a Description   : Minimum NodeID difference for an edge to be considered
 *  a loop closure.
 *
 * - \b ICP_max_distance
 *  + \a Section       : EdgeRegistrationDeciderParameters
 *  + \a Default value : 10 // meters
 *  + \a Required      : FALSE
 *  + \a Description   : Maximum distance for scan-matching. Decider tries to
 *  align the laser scans of the current node and each of the previous nodes
 *  that are found within the designated ICP_max_distance.
 *
 * - \b ICP_goodness_thresh
 *   + \a Section       : EdgeRegistrationDeciderParameters
 *   + \a Default value : 0.75
 *   + \a Required      : FALSE
 *   + \a Description   : Threshold for accepting a scan-matching edge between
 *   the current and previous nodes
 *
 * - \b visualize_laser_scans
 *   + \a Section       : VisualizationParameters
 *   + \a Default value : TRUE
 *   + \a Required      : FALSE
 *
 * - \b scans_img_external_dir
 *   + \a Section       : EdgeRegistrationDeciderParameters
 *   + \a Default value : . (current directory)
 *   + \a Required      : FALSE
 *   + \a Description   : Only applicable in datasets with 3DRangeScans that
 *   are externally stored (not stored in the given .rawlog file).
 *
 * \ingroup mrpt_graphslam_grp
 */
template<class GRAPH_T=typename mrpt::graphs::CNetworkOfPoses2DInf >
class CICPCriteriaERD :
	public mrpt::graphslam::deciders::CRangeScanEdgeRegistrationDecider<GRAPH_T>
{
	public:
		/**\brief Handy typedefs */
		/**\{*/
		/**\brief type of graph constraints */
		typedef typename GRAPH_T::constraint_t constraint_t;
		/**\brief type of underlying poses (2D/3D). */
		typedef typename GRAPH_T::constraint_t::type_value pose_t;
		typedef CRangeScanEdgeRegistrationDecider<GRAPH_T> parent_t;
		typedef typename parent_t::range_ops_t range_ops_t;
		typedef CICPCriteriaERD<GRAPH_T> decider_t; /**< self type - Handy typedef */
		typedef typename parent_t::nodes_to_scans2D_t nodes_to_scans2D_t;
		/**\}*/

		// Public methods
		//////////////////////////////////////////////////////////////
		CICPCriteriaERD();
		~CICPCriteriaERD();

		bool updateState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );

		void notifyOfWindowEvents(
				const std::map<std::string, bool>& events_occurred);
		void getEdgesStats(
				std::map<std::string, int>* edge_types_to_num) const;

		void initializeVisuals();
		void updateVisuals();
		void loadParams(const std::string& source_fname);
		void printParams() const;

		struct TParams: public mrpt::utils::CLoadableOptions {
			public:
				TParams(decider_t& d);
				~TParams();


				void loadFromConfigFile(
						const mrpt::utils::CConfigFileBase &source,
						const std::string &section);
				void 	dumpToTextStream(mrpt::utils::CStream &out) const;

				decider_t& decider;
				// maximum distance for checking other nodes for ICP constraints
				double ICP_max_distance;
				// threshold for accepting an ICP constraint in the graph
				double ICP_goodness_thresh;
				size_t LC_min_nodeid_diff;
				bool visualize_laser_scans;
				// keystroke to be used for the user to toggle the LaserScans from
				// the CDisplayWindow
				std::string keystroke_laser_scans;

				std::string scans_img_external_dir;

				bool has_read_config;
		};
		void getDescriptiveReport(std::string* report_str) const;

		// Public variables
		// ////////////////////////////
		TParams params;

	protected:
		// protected functions
		//////////////////////////////////////////////////////////////
		void checkRegistrationCondition2D(
				const std::set<mrpt::utils::TNodeID>& nodes_set);
		void checkRegistrationCondition3D(
				const std::set<mrpt::utils::TNodeID>& nodes_set);
		void registerNewEdge(
				const mrpt::utils::TNodeID& from,
				const mrpt::utils::TNodeID& to,
				const constraint_t& rel_edge );
		void checkIfInvalidDataset(mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );
		/**\brief Get a list of the nodeIDs whose position is within a certain
		 * distance to the specified nodeID
		 */
		void getNearbyNodesOf(
				std::set<mrpt::utils::TNodeID> *nodes_set,
				const mrpt::utils::TNodeID& cur_nodeID,
				double distance );
		/**\brief togle the LaserScans visualization on and off
		 */
		void toggleLaserScansVisualization();
		void dumpVisibilityErrorMsg(std::string viz_flag,
				int sleep_time=500 /* ms */);

		// protected variables
		//////////////////////////////////////////////////////////////


		mrpt::utils::TColor m_search_disk_color; //!< see Constructor for initialization
		mrpt::utils::TColor m_laser_scans_color; //!< see Constructor for initialization
		double m_offset_y_search_disk;
		int m_text_index_search_disk;


		std::map<mrpt::utils::TNodeID,
			mrpt::obs::CObservation2DRangeScanPtr> m_nodes_to_laser_scans2D;
		std::map<mrpt::utils::TNodeID,
			mrpt::obs::CObservation3DRangeScanPtr> m_nodes_to_laser_scans3D;
		std::map<std::string, int> m_edge_types_to_nums;
		bool m_is_using_3DScan;

		mrpt::obs::CObservation2DRangeScanPtr m_last_laser_scan2D;
		mrpt::obs::CObservation3DRangeScanPtr m_last_laser_scan3D;
		// fake 2D laser scan generated from corresponding 3DRangeScan for
		// visualization reasons
		mrpt::obs::CObservation2DRangeScanPtr m_fake_laser_scan2D;

};

} } } // end of namespaces

#include "CICPCriteriaERD_impl.h"
#endif /* end of include guard: CICPCRITERIAERD_H */
