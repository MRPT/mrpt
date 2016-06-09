/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// Sun May 22 12:48:25 EEST 2016, nickkouk
// General TODO list:
// TODO: Make class generic - so that it handles 3D datasets
// TODO: Add functionality to be able to use different optimizers
// TODO: Plot x^2 ,x^2/s, time, iteration in the viz. window

#ifndef GRAPHSLAMENGINE_H
#define GRAPHSLAMENGINE_H

#include <mrpt/system/filesystem.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/mrpt_stdint.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CMetricMapBuilder.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam.h>
#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // It's in the lib mrpt-maps now

#include <string>
#include <sstream>
#include <map>
#include <cerrno>
#include <cmath> // fabs function

#include "EdgeCounter.h"
#include "CWindowObserver.h"
//#include "CNodeRegistrationDecider.h"
#include "CFixedIntervalsNRD.h"

bool verbose = true;
#define VERBOSE_COUT	if (verbose) std::cout << "[graphslam_engine] "

template< class GRAPH_t, class NODE_REGISTRATOR >
class CGraphSlamEngine_t {
	public:

		typedef std::map<std::string, mrpt::utils::CFileOutputStream*> fstreams_out;
		typedef std::map<std::string, mrpt::utils::CFileOutputStream*>::
			iterator fstreams_out_it;
		typedef std::map<std::string, mrpt::utils::CFileOutputStream*>::
			const_iterator fstreams_out_cit;

		typedef std::map<std::string, mrpt::utils::CFileInputStream*> fstreams_in;
		typedef std::map<std::string, mrpt::utils::CFileInputStream*>::
			iterator fstreams_in_it;
		typedef std::map<std::string, mrpt::utils::CFileInputStream*>::
			const_iterator fstreams_in_cit;

		typedef typename GRAPH_t::constraint_t constraint_t;
		// type of underlying poses (2D/3D)
		typedef typename GRAPH_t::constraint_t::type_value pose_t; 

		typedef mrpt::math::CMatrixFixedNumeric<double,
						constraint_t::state_length, constraint_t::state_length> InfMat;


		// Ctors, Dtors
		//////////////////////////////////////////////////////////////
		CGraphSlamEngine_t(const std::string& config_file,
				mrpt::gui::CDisplayWindow3D* win = NULL,
				CWindowObserver* win_observer = NULL,
				std::string rawlog_fname = "");
		~CGraphSlamEngine_t();

		// Public function definitions
		//////////////////////////////////////////////////////////////
		/**
		 * Wrapper fun around the GRAPH_t corresponding method
		 */
		void saveGraph() const {
			MRPT_START

			if (!m_has_read_config) {
				THROW_EXCEPTION("Config file has not been provided yet.\nExiting...");
			}
			std::string fname = m_output_dir_fname + "/" + m_save_graph_fname;
			saveGraph(fname);

			MRPT_END
		}
		/**
		 * Wrapper fun around the GRAPH_t corresponding method
		 */
		void saveGraph(const std::string& fname) const {
			MRPT_START

			m_graph.saveToTextFile(fname);
			VERBOSE_COUT << "Saved graph to text file: " << fname <<
				" successfully." << std::endl;

			MRPT_END
		}
		/**
		 * Read the configuration file specified and fill in the corresponding
		 * class members
		 */
	 void readConfigFile(const std::string& fname);
		/**
		 * Print the problem parameters (usually fetched from a configuration file)
		 * to the console for verification
		 *
		 * \sa CGraphSlamEngine_t::parseRawlogFile
		 */
		void printProblemParams() const;
		/**
		 * TODO - Make this a function template so that it can handle camera
		 * images, laser scan files, etc.
		 * Reads the file provided and builds the graph. Method returns false if
		 * user issues termination coe (Ctrl+c) otherwise true
		 **/
		bool parseRawlogFile();
		/**
		 * CGraphSlamEngine_t::optmizeGraph
		 *
		 * Optimize the under-construction graph
		 */
		inline void optimizeGraph(GRAPH_t* graph);
		/**
		 * CGraphSlamEngine_t::visualizeGraph
		 *
		 * Called internally for updating the vizualization scene for the graph
		 * building procedure
		 */
		inline void visualizeGraph(const GRAPH_t& gr);
		/**
		 * GRAPH_t getter function - return reference to own graph
		 * Handy function for visualization, printing purposes
		 */
		const GRAPH_t& getGraph() const { return m_graph; }

	private:
		// Private function definitions
		//////////////////////////////////////////////////////////////

		/**
		 * General initialization method to call from the different Ctors
		 */
		void initGraphSlamEngine();
		/**
		 * Initialize (clean up and create new files) the output directory
		 * Also provides cmd line arguements for the user to choose the desired
		 * action.
		 * \sa CGraphSlamEngine_t::initResultsFile
		 */
		void initOutputDir();
		/**
		 * Method to automate the creation of the output result files
		 * Open and write an introductory message using the provided fname
		 */
		void initResultsFile(const std::string& fname);
		/**
		 * CGraphSlamEngine_t::getICPEdge
		 *
		 * Align the laser scans of the given poses and compute a constraint
		 * between them. Return the goodness of the ICP operation and let the user
		 * decide if they want to keep it as an edge or not
		 */
		inline double getICPEdge(const mrpt::utils::TNodeID& from, 
				const mrpt::utils::TNodeID& to, constraint_t *rel_edge );
		/**
		 * assignTextMessageParameters
		 *
		 * Assign the next available offset_y and text_index for the textMessage under
		 * construction. Then increment the respective current counters
		 */
		inline void assignTextMessageParameters(double* offset_y, int* text_index) {
			*offset_y = m_curr_offset_y;
			m_curr_offset_y += kOffsetYStep;

			*text_index = m_curr_text_index;
			m_curr_text_index += kIndexTextStep;
		}
		inline void updateCurPosViewport(const GRAPH_t& gr);
		inline void updateTotalOdometryDistance() const;

		/**
		 * BuildGroundTruthMap
		 *
		 * Parse the ground truth .txt file and fill in the corresponding
		 * timestamp_to_pose2d map. Return true if operation was successful
		 * Call the function in the constructor if the visualize_GT flag is set to
		 * true. 
		 */
		inline void BuildGroundTruthMap(const std::string& rawlog_fname_GT);
		/**
		 * autofitObjectInView
		 *
		 * Set the camera parameters of the CDisplayWindow3D so that the whole
		 * graph is viewed in the window.
		 */
		inline void autofitObjectInView(const mrpt::opengl::CSetOfObjectsPtr& gr);
		/**
		 * queryObserverForEvents
		 *
		 * Query the given observer for any events (keystrokes, mouse clicks, that
		 * may have occured in the CDisplayWindow3D  and fill in the corresponding
		 * class variables
		 */
		inline void queryObserverForEvents();
		/**
		 * getSetOfNodes
		 *
		 * Fetch a set of neighborhood nodes given either a number of nodes or a
		 * distance relative to the current node.
		 */
		 void getNearbyNodesOf(std::set<mrpt::utils::TNodeID> *lstNodes, 
				const mrpt::utils::TNodeID& cur_nodeID,
				void* num_nodes_or_distance, 
				bool use_distance_criterion = true);


		// VARIABLES
		//////////////////////////////////////////////////////////////

		// the graph object to be built and optimized
		GRAPH_t m_graph;
		NODE_REGISTRATOR m_node_registrator; 

		/**
		 * Problem parameters.
		 * Most are imported from a .ini config file
		 * \sa CGraphSlamEngine_t::readConfigFile
		 */
		std::string	m_config_fname;

		std::string	m_rawlog_fname;
		std::string	m_fname_GT;
		std::string	m_output_dir_fname;
		bool		m_user_decides_about_output_dir;
		bool		m_do_debug;
		std::string	m_debug_fname;
		std::string	m_save_graph_fname;

		bool		m_do_pose_graph_only;
		std::string	m_optimizer;

		std::string	m_loop_closing_alg;
		double	m_loop_closing_min_nodeid_diff;

		bool		m_has_read_config;

		/**
		 * FileStreams
		 * variables that keeps track of the out fstreams so that they can be closed
		 * (if still open) in the class Dtor.
		 */
		fstreams_out m_out_streams;
		fstreams_in m_in_streams;

		// visualization objects
		mrpt::gui::CDisplayWindow3D* m_win;

		mrpt::utils::TParametersDouble m_optimized_graph_viz_params;
		bool m_visualize_optimized_graph;
		bool m_visualize_odometry_poses;
		bool m_visualize_GT;

		/**
		 * textMessage Parameters
		 *
		 */
		std::string m_font_name;
		int m_font_size;

		// textMessage vertical text position
		const double kOffsetYStep;
		double m_curr_offset_y;
		double m_offset_y_graph;
		double m_offset_y_odometry;
		double m_offset_y_timestamp;
		double m_offset_y_GT;

		// textMessage index
		const int kIndexTextStep;
		int m_curr_text_index;
		int m_text_index_graph;
		int m_text_index_odometry;
		int m_text_index_timestamp;
		int m_text_index_GT;

		// instance to keep track of all the edges + visualization related
		// functions
		EdgeCounter_t m_edge_counter;
		int m_num_of_edges_for_collapse;

		// std::maps to store information about the graph(s)
		std::map<const GRAPH_t*, std::string> graph_to_name;
		std::map<const GRAPH_t*, mrpt::utils::TParametersDouble*> graph_to_viz_params;

		// pose_t vectors
		std::vector<pose_t*> m_odometry_poses;
		std::vector<pose_t*> m_GT_poses;

		// PointCloud colors
		mrpt::utils::TColorf m_odometry_color; // see Ctor for initialization
		mrpt::utils::TColorf m_GT_color;
		mrpt::utils::TColor m_robot_model_color;

		bool m_is3D;
		mrpt::utils::TNodeID m_nodeID_max;

		// ICP configuration
		float m_ICP_goodness_thres;

		double m_ICP_max_distance;
		int m_ICP_prev_nodes; // add ICP constraints with m_prev_nodes_for_ICP nodes back
		bool m_ICP_use_distance_criterion;
		std::map<const mrpt::utils::TNodeID, mrpt::obs::CObservation2DRangeScanPtr> m_nodes_to_laser_scans;
		mrpt::slam::CICP m_ICP;


		// graph optimization
		mrpt::utils::TParametersDouble m_optimization_params;
		pose_t m_curr_estimated_pose;

		// Use mutliple threads for visualization and graph optimization
		mrpt::system::TThreadHandle m_thread_optimize;
		mrpt::system::TThreadHandle m_thread_visualize;

		// mark graph modification/accessing explicitly for multithreaded implementation
		mrpt::synch::CCriticalSection m_graph_section;

		// Interaction with the CDisplayWindow - use of CWindowObserver
		bool m_autozoom_active, m_request_to_exit;
		CWindowObserver* m_win_observer;

};

// pseudo-split the definition and implementation of template
#include "CGraphSlamEngine_impl.h"

#endif /* end of include guard: GRAPHSLAMENGINE_H */
