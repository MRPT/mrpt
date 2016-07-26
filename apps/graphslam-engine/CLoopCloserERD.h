/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CLOOPCLOSERERD_H
#define CLOOPCLOSERERD_H


// TODO - remove the ones not needed.
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/utils/TColor.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>


#include <algorithm>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <set>
#include <sstream>

#include "CEdgeRegistrationDecider.h"
#include "CRangeScanRegistrationDecider.h"


namespace mrpt { namespace graphslam { namespace deciders {

/** Edge Registration Decider scheem specialized in Loop Closing.
 *
 * Scheme is implemented based on the following two papers:
 *
 * TODO - split this line if possible
 * <a href="http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=1641810&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D1641810">Consistent Observation Grouping for Generating Metric-Topological Maps that Improves Robot Localization</a> - J. blanco, J. Gonzalez, J. Antonio Fernandez Madrigal
 * - We split the under-construction graph into groups of nodes. The groups are
 *   formatted basd on the observations gathered each node. the actuall split
 *   between the groups is decided by the minimum normalized Cut (minNcut) as
 *   described in the aformentioned paper
 *
 * <a href="https://april.eecs.umich.edu/pdfs/olson2009ras.pdf">Recognizing places using spectrally clustered local matches</a> - E. Olson, 2009
 * - Having the groups already assembled, we generate all the hypotheses in
 *   each group and evaluate each set using its corresponding Pairwise
 *   consistency matrix.
 *
 * \b Description
 *
 * // TODO - add here...
 * // TODO - explain the process/flow of method calls when an LC is detected
 * The Edge registration procedure is implemented as described below:
 *
 *
 * \b Specifications
 *
 * Map type: 2D
 * MRPT rawlog format: #1, #2
 * Observations: CObservation2DRangeScan
 * Edge Registration Strategy: Pairwise Consistency of ICP Edges
 *
 * \ingroup mrpt_graphslam_grp
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf >
class CLoopCloserERD:
	public mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_t>,
	public mrpt::graphslam::deciders::CRangeScanRegistrationDecider<GRAPH_t>
{
	public:
		/**\brief type of graph constraints */
		typedef typename GRAPH_t::constraint_t constraint_t;
		/**\brief type of underlying poses (2D/3D). */
		typedef typename GRAPH_t::constraint_t::type_value pose_t;
		/**\brief Typedef for accessing methods of the RangeScanRegistrationDecider_t parent class. */
		typedef mrpt::graphslam::deciders::CRangeScanRegistrationDecider<GRAPH_t> range_scanner_t;
		typedef CLoopCloserERD<GRAPH_t> decider_t; /**< self type - Handy typedef */
		/**\brief New typedef for splitting the nodes into groups */
		typedef std::vector<mrpt::vector_uint> partitions_t;
		typedef std::map<const mrpt::utils::TNodeID, mrpt::obs::CObservation2DRangeScanPtr> nodes_to_scans2D_t;

		// Public methods
		//////////////////////////////////////////////////////////////
		CLoopCloserERD();
		~CLoopCloserERD();

		bool updateState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );


		void setGraphPtr(GRAPH_t* graph);
		void setRawlogFname(const std::string& rawlog_fname);
		void setWindowManagerPtr(mrpt::graphslam::CWindowManager* win_manager);
		void notifyOfWindowEvents(
				const std::map<std::string, bool>& events_occurred);
		void getEdgesStats(
				std::map<const std::string, int>* edge_types_to_num) const;

		void initializeVisuals();
		void updateVisuals();
		bool justInsertedLoopClosure() const;
		void loadParams(const std::string& source_fname);
		void printParams() const;


		void getDescriptiveReport(std::string* report_str) const;

		// Public variables
		// ////////////////////////////

	private:
		// Private functions
		//////////////////////////////////////////////////////////////

		/**\brief Struct for storing together the parameters needed for ICP
		 * matching, laser scans visualization etc.
		 */
		struct TLaserParams: public mrpt::utils::CLoadableOptions {
			public:
				TLaserParams();
				~TLaserParams();

				void loadFromConfigFile(
						const mrpt::utils::CConfigFileBase &source,
						const std::string &section);
				void 	dumpToTextStream(mrpt::utils::CStream &out) const;

				mrpt::slam::CICP icp;
				// threshold for accepting an ICP constraint in the graph
				double ICP_goodness_thresh;
				int prev_nodes_for_ICP; // how many nodes back to check ICP against?

 				/** see Constructor for initialization */
				mrpt::utils::TColor laser_scans_color;
				bool visualize_laser_scans;
				// keystroke to be used by the user to toggle the LaserScans from
				// the CDisplayWindow
				std::string keystroke_laser_scans;

				bool has_read_config;
		};

		/**\brief Struct for storing together the loop-closing related parameters.
		 */
		struct TLoopClosureParams: public mrpt::utils::CLoadableOptions {
			public:
				TLoopClosureParams();
				~TLoopClosureParams();

				void loadFromConfigFile(
						const mrpt::utils::CConfigFileBase &source,
						const std::string &section);
				void 	dumpToTextStream(mrpt::utils::CStream &out) const;


				/**\brief nodeID difference for detecting potential loop closure in a
		 		 * partition.
		 		 *
		 		 * If this difference is surpassed then the partition should be
		 		 * investigated for loop closures using Olson's strategy.
		 		 */
				int LC_min_nodeid_diff;
				bool visualize_map_partitions;
				std::string keystroke_map_partitions;

				double offset_y_map_partitions;
				int text_index_map_partitions;

				// map partitioning  - visualization window parameters
				const double balloon_elevation;
				const double balloon_radius;
				const mrpt::utils::TColor balloon_std_color;
				const mrpt::utils::TColor balloon_curr_color;
				const mrpt::utils::TColor connecting_lines_color;

				bool has_read_config;


		};
		TLaserParams m_laser_params;
		TLoopClosureParams m_lc_params;

		/** \brief Initialization function to be called from the various
		 * constructors.
		 */
		void initCLoopCloserERD();
		void registerNewEdge(
				const mrpt::utils::TNodeID& from,
				const mrpt::utils::TNodeID& to,
				const constraint_t& rel_edge );
		/**\brief Addd ICP constraints from X previous nodeIDs up to the given
		 * nodeID.
		 *
		 * X is set by the user in the .ini configuration file (see
		 * TLaserParams::prev_nodes_for_ICP)
		 */
		void addScanMatchingEdges(mrpt::utils::TNodeID curr_nodeID);

		void initLaserScansVisualization();
		void updateLaserScansVisualization();
		/**\brief togle the LaserScans visualization on and off
		 */
		void toggleLaserScansVisualization();
		void dumpVisibilityErrorMsg(std::string viz_flag,
				int sleep_time=500 /* ms */);
		void checkIfInvalidDataset(mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );
		/**\brief Split the currently registered graph nodes into partitions.  */
		void updateMapPartitions(bool full_update=false);
		/**\brief Initialize the visualization of the map partition objects. */
		void initMapPartitionsVisualization();
		/**\brief Update the map partitions visualization. */
		void updateMapPartitionsVisualization();
		/**\brief Toggle the map partitions visualization objects.
		 *
		 * To be called  upon relevant keystroke press by the user (see \b
		 * TLoopClosureParams::keystroke_map_partitions)
		 */
		void toggleMapPartitionsVisualization();

		void computeCentroidOfNodesVector(const vector_uint& nodes_list,
				std::pair<double, double>* centroid_coords);

		template<class T>
		void printVectorOfVectors(const T& t) const;
		template<class T>
		void printVector(const T& t) const;

		/**\brief Check the registered so far partitions for potential loop
		 * closures.
		 *
		 * Practically checks whether there exist nodes in a single partition whose
		 * distance surpasses the minimum loop closure nodeID distance. The latter
		 * is read from a .ini external file, thus specified by the user (see \b
		 * TLoopClosureParams.LC_min_nodeid_diff
		 *
		 * \sa evaluatePartitionsForLC
		 */
		void checkPartitionsForLC(partitions_t* partitions_for_LC);
		/**\brief Evaluate the given partitions for loop closures.
		 *
		 * Call this method when you have identified potential loop closures - e.g.
		 * far away nodes in the same partitions - and you want to evaluate the
		 * potential hypotheses in the group
		 *
		 * \sa checkPartitionsForLC
		 */
		void evaluatePartitionsForLC(const partitions_t& partitions);
		/**\brief Calculate the Pairwise consistency matrix in the given partition
		 * set.
		 *
		 * \sa generatePWConsistencyMatrix
		 */
		void generatePWConsistencyMatrix(const vector_uint& partition,
				mrpt::math::CMatrixDouble* consist_matrix);
		/**\brief Return the pairwise consistency between the observations of the
		 * given nodes.
		 *
		 * Use the dijkstra link between a1, a2, and b1, b2 pairs and use an ICP
		 * hypotheses between a1, b1 and a2, b2 pairs
		 *
		 * \sa generatePWConsistencyMatrix
		 */
		void generatePWConsistencyElement(
				const mrpt::utils::TNodeID& a1,
				const mrpt::utils::TNodeID& a2,
				const mrpt::utils::TNodeID& b1,
				const mrpt::utils::TNodeID& b2,
				mrpt::math::CMatrixDouble33 consistency_elem );

		// Private variables
		//////////////////////////////////////////////////////////////
		/**\brief Instance responsible for partitioning the map */
		mrpt::slam::CIncrementalMapPartitioner m_partitioner;

		GRAPH_t* m_graph; /**<\brief Pointer to the graph under construction */
		mrpt::gui::CDisplayWindow3D* m_win;
		mrpt::graphslam::CWindowManager* m_win_manager;
		mrpt::graphslam::CWindowObserver* m_win_observer;

		std::string m_rawlog_fname;

		bool m_initialized_visuals;
		bool m_just_inserted_loop_closure;

		/**\brief Keep track of the registered edge types.
		 *
		 * Handy for displaying them in the Visualization window.
		 */
		std::map<const std::string, int> m_edge_types_to_nums;
 		/**\brief Keep track of the total number of registered nodes since the last
 		 * time class method was called */
		int m_last_total_num_of_nodes;
		/**\brief Surpass this to start adding edges */
		int m_threshold_to_start;

		/**\brief Map for keeping track of the observation recorded at each graph
		 * position
		 */
		nodes_to_scans2D_t  m_nodes_to_laser_scans2D;
		/**\brief Keep the last laser scan for visualization purposes */
		mrpt::obs::CObservation2DRangeScanPtr m_last_laser_scan2D;

		/**\brief Previous partitions vector */
		partitions_t m_last_partitions;
		/**\brief Current partitions vector */
		partitions_t m_curr_partitions;
		/**\brief Indicate whether the partitions have been updated recently */
		bool m_partitions_full_update;

		// find out if decider is invalid for the given dataset
		bool m_checked_for_usuable_dataset;
		size_t m_consecutive_invalid_format_instances;
		const size_t m_consecutive_invalid_format_instances_thres;

		mrpt::utils::COutputLogger m_out_logger; /**<Output logger instance */
		mrpt::utils::CTimeLogger m_time_logger; /**<Time logger instance */



};

} } } // end of namespaces


#include "CLoopCloserERD_impl.h"
#endif /* end of include guard: CLOOPCLOSERERD_H */

