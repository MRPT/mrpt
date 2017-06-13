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
 * adding new edges should be the goodness of the candidate ICP edge. The nodes
 * for ICP are picked <em>based on the distance from the last inserted
 * node</em>.
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
 *
 * - \b use_mahal_distance
 *   + \a section       : EdgeRegistrationDeciderParameters
 *   + \a Default value : TRUE
 *   + \a Required      : FALSE
 *   + \a Description   : If True, the decider uses the mahalanobis distance of
 *   the suggested ICP constraint and the current constraint between a set of
 *   nodes to decide if adding such constraint makes sense or if it is an
 *   outlier
 *
 * - \b scans_img_external_dir
 *   + \a Section       : EdgeRegistrationDeciderParameters
 *   + \a Default value : . (current directory)
 *   + \a Required      : FALSE
 *   + \a Description   : Only applicable in datasets with 3DRangeScans that
 *   are externally stored (not stored in the given .rawlog file).
 *
 * \note Since the decider inherits from the CRangeScanEdgeRegistrationDecider
 * class, it parses the configuration parameters of the latter as well from the
 * "ICP" section. Refer to the latter for its list of configuration parameters
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
		typedef typename parent_t::nodes_to_scans3D_t nodes_to_scans3D_t;
		/**\}*/

		// Public methods
		//////////////////////////////////////////////////////////////
		CICPCriteriaERD();
		~CICPCriteriaERD();

		bool updateState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );
		/**\brief Fetch nodes that are in a radious of the latest registered nodeID.
		 *
		 * \note This basically provides a wrapper around the getNearbyNodesOf
		 * method
		 */
		void fetchNodeIDsForScanMatching(
				const mrpt::utils::TNodeID& curr_nodeID,
				std::set<mrpt::utils::TNodeID>* nodes_set);

		void notifyOfWindowEvents(
				const std::map<std::string, bool>& events_occurred);
		void initializeVisuals();
		void updateVisuals();
		void loadParams(const std::string& source_fname);
		void printParams() const;
		void getDescriptiveReport(std::string* report_str) const;

	protected:
		// protected functions
		//////////////////////////////////////////////////////////////

		void registerNewEdge(
				const mrpt::utils::TNodeID& from,
				const mrpt::utils::TNodeID& to,
				const constraint_t& rel_edge );
		void checkIfInvalidDataset(mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );
		/**\brief Get a list of the nodeIDs whose position is within a certain
		 * distance to the specified nodeID
		 *
		 * \param[in] distance Radious cenetered at curr_nodeID to check ICP
		 * against.
		 */
		void getNearbyNodesOf(
				const mrpt::utils::TNodeID& curr_nodeID,
				std::set<mrpt::utils::TNodeID> *nodes_set,
				const double distance);

		// TODO - format this for the 3D case - Disk => Sphere
		// TODO - Override this for the mr_case
		void initICPDistanceVizualization();
		// TODO - format this for the 3D case - Disk => Sphere
		// TODO - Override this for the mr_case
		void updateICPDistanceVizualization();

		// protected variables
		//////////////////////////////////////////////////////////////


		mrpt::utils::TColor m_search_disk_color; //!< see Constructor for initialization
		double m_offset_y_search_disk;
		int m_text_index_search_disk;

		std::string m_ICP_max_distance_obj_name;

		double m_ICP_max_distance;
		/** Minimum node ID difference to consider an edge as a loop closure */
		size_t m_LC_min_nodeid_diff;


};

} } } // end of namespaces

#include "CICPCriteriaERD_impl.h"
#endif /* end of include guard: CICPCRITERIAERD_H */
