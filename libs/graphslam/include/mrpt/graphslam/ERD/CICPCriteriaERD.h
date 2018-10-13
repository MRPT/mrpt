/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/img/TColor.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/slam/CICP.h>

#include <map>
#include <set>
#include <string>

#include <mrpt/graphslam/interfaces/CRangeScanEdgeRegistrationDecider.h>

namespace mrpt::graphslam::deciders
{
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
 *   + \a default value : 1 (mrpt::system::LVL_INFO)
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
template <class GRAPH_T = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CICPCriteriaERD
	: public mrpt::graphslam::deciders::CRangeScanEdgeRegistrationDecider<
		  GRAPH_T>
{
   public:
	/**\brief Handy typedefs */
	/**\{*/
	/**\brief type of graph constraints */
	using constraint_t = typename GRAPH_T::constraint_t;
	/**\brief type of underlying poses (2D/3D). */
	using pose_t = typename GRAPH_T::constraint_t::type_value;
	using parent_t = CRangeScanEdgeRegistrationDecider<GRAPH_T>;
	using range_ops_t = typename parent_t::range_ops_t;
	/** self type - Handy type */
	using decider_t = CICPCriteriaERD<GRAPH_T>;
	using nodes_to_scans2D_t = typename parent_t::nodes_to_scans2D_t;
	/**\}*/

	// Public methods
	//////////////////////////////////////////////////////////////
	CICPCriteriaERD();
	~CICPCriteriaERD() override = default;

	bool updateState(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation) override;

	void notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred) override;
	void getEdgesStats(
		std::map<std::string, int>* edge_types_to_num) const override;

	void initializeVisuals() override;
	void updateVisuals() override;
	void loadParams(const std::string& source_fname) override;
	void printParams() const override;

	struct TParams : public mrpt::config::CLoadableOptions
	{
	   public:
		TParams(decider_t& d);
		~TParams() override;

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;
		void dumpToTextStream(std::ostream& out) const override;

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
	void getDescriptiveReport(std::string* report_str) const override;

	// Public variables
	// ////////////////////////////
	TParams params;

   protected:
	// protected functions
	//////////////////////////////////////////////////////////////
	void checkRegistrationCondition2D(
		const std::set<mrpt::graphs::TNodeID>& nodes_set);
	void checkRegistrationCondition3D(
		const std::set<mrpt::graphs::TNodeID>& nodes_set);
	void registerNewEdge(
		const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to,
		const constraint_t& rel_edge) override;
	void checkIfInvalidDataset(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation);
	/**\brief Get a list of the nodeIDs whose position is within a certain
	 * distance to the specified nodeID
	 */
	void getNearbyNodesOf(
		std::set<mrpt::graphs::TNodeID>* nodes_set,
		const mrpt::graphs::TNodeID& cur_nodeID, double distance);
	/**\brief togle the LaserScans visualization on and off
	 */
	void toggleLaserScansVisualization();
	void dumpVisibilityErrorMsg(
		std::string viz_flag, int sleep_time = 500 /* ms */);

	// protected variables
	//////////////////////////////////////////////////////////////

	/** see Constructor for initialization */
	mrpt::img::TColor m_search_disk_color;
	/** see Constructor for initialization */
	mrpt::img::TColor m_laser_scans_color;
	double m_offset_y_search_disk;
	int m_text_index_search_disk;

	std::map<mrpt::graphs::TNodeID, mrpt::obs::CObservation2DRangeScan::Ptr>
		m_nodes_to_laser_scans2D;
	std::map<mrpt::graphs::TNodeID, mrpt::obs::CObservation3DRangeScan::Ptr>
		m_nodes_to_laser_scans3D;
	std::map<std::string, int> m_edge_types_to_nums;
	bool m_is_using_3DScan{false};

	mrpt::obs::CObservation2DRangeScan::Ptr m_last_laser_scan2D;
	mrpt::obs::CObservation3DRangeScan::Ptr m_last_laser_scan3D;
	// fake 2D laser scan generated from corresponding 3DRangeScan for
	// visualization reasons
	mrpt::obs::CObservation2DRangeScan::Ptr m_fake_laser_scan2D;
};
}  // namespace mrpt::graphslam::deciders
#include "CICPCriteriaERD_impl.h"
