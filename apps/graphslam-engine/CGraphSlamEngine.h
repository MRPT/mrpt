/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef GRAPHSLAMENGINE_H
#define GRAPHSLAMENGINE_H

#include <mrpt/math/CQuaternion.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam.h>
#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // It's in the lib mrpt-maps now
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CMetricMapBuilder.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/mrpt_stdint.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/utils/TColor.h>
#include <mrpt/utils/CImage.h>

#include <cstdlib>
#include <string>
#include <sstream>
#include <map>
#include <cerrno>
#include <cmath> // fabs function, power
#include <set>
#include <algorithm>
#include <cstdlib>

#include "CEdgeCounter.h"
#include "CWindowObserver.h"
#include "CWindowManager.h"

namespace mrpt { namespace graphslam {

/**
 * Given a dataset of measurements build a graph of nodes (keyframes) and
 * constraints (edges) and solve it to find an estimation of the actual robot
 * path.
 * \todo - add here...
 *
 * The template arguments are listed below:
 * - \em GRAPH_t: The type of Graph to be constructed and optimized.
 * - \em NODE_REGISTRAR: Class responsible of adding new nodes in the graph.
 *   Class should at least implement the deciders::CNodeRegistrationDecider_t
 *   interface provided in CNodeRegistrationDecider.h file.
 * - \em EDGE_REGISTRAR: Class responsible of adding new edges in the graph.
 *   Class should at least implement the deciders::CEdgeRegistrationDecider_t
 *   interface provided in CEdgeRegistrationDecider.h file.
 * - \em OPTIMIZER: Class responsible of optimizing the graph. Class should at
 *   least implement the optimizers::CGraphSlamOptimizer_t interface provided
 *   in CGraphslamOptimizer.h file.
 *
 * The GRAPH_t resource is accessed after having locked the relevant section
 * \em m_graph_section. Critical section is also <em> locked prior to the calls
 * to the deciders/optimizers </em>.
 *
 * \ingroup mrpt_graphslam_grp
 * \note Implementation can be found in the file \em CGraphSlamEngine_impl.h
 */
template<
		class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf,
		class NODE_REGISTRAR=typename mrpt::graphslam::deciders::CFixedIntervalsNRD<GRAPH_t>,
		class EDGE_REGISTRAR=typename mrpt::graphslam::deciders::CICPGoodnessERD<GRAPH_t>,
		class OPTIMIZER=typename mrpt::graphslam::optimizers::CLevMarqGSO<GRAPH_t> >
class CGraphSlamEngine {
	public:

		/**\{*/
		/**\brief Handy typedef for managing output file streams.*/
		typedef std::map<std::string, mrpt::utils::CFileOutputStream*> fstreams_out;
		/**\brief Handy typedef for managing output file streams.*/
		typedef std::map<std::string, mrpt::utils::CFileOutputStream*>::iterator fstreams_out_it;
		/**\}*/

		/**\brief type of graph constraints */
		typedef typename GRAPH_t::constraint_t constraint_t;
		/**\brief type of underlying poses (2D/3D). */
		typedef typename GRAPH_t::constraint_t::type_value pose_t;

		/**\brief Constructor of CGraphSlamEngine class template.
		 *
		 * \param[in] config_file .ini file containing the configuration
		 * parameters for the CGraphSlamEngine as well as the deciders/optimizer
		 * classes that CGraphSlamEngine is using
		 * \param[in] win CDisplayWindow3D for visualizing the graphSLAM operation.
		 * \param[in] win_observer CObserver instance for monitoring keyboard and
		 * mouse events issued by the user
		 * \param[in] rawlog_fname .rawlog dataset file, containing the robot
		 * measurements. CGraphSlamEngine supports both
		 * <a href="http://www.mrpt.org/Rawlog_Format"> MRPT rwalog formats </a>
		 * but in order for graphSLAM to work as expected the rawlog foromat has to
		 * be supported by the every decider/optimizer class that
		 * CGraphSlamEngine makes use of.
		 * \param[in] fname_GT Textfile containing the ground truth path of the
		 * robot. Currently the class can read ground truth files corresponding
		 * either to <em>RGBD - TUM datasets</em> or to rawlog files generated with
		 * the \em GridMapNavSimul MRPT application.
		 *
		 * \note If caller doesn't provide an mrpt::gui::CDisplayWindow3D instance,
		 * the application runs on <em> headless mode </em>.  In this case, no
		 * visual feedback is given but application receives a big boost in
		 * performance
		 */
		CGraphSlamEngine(const std::string& config_file,
				const std::string rawlog_fname="",
				const std::string fname_GT="",
				bool enable_visuals=true);
		/**\brief Default Destructor. */
		~CGraphSlamEngine();

		// Public function definitions
		//////////////////////////////////////////////////////////////
		/**\brief Wrapper method around the CGraphSlamEngine::saveGraph method.
		 *
		 * Output .graph filename is set either by the user via the .ini
		 * save_graph_fname variable or (if not specified in the .ini file) it is
		 * set to "output_graph.graph"
		 *
		 * \sa save3DScene
		 * */
		void saveGraph() const;
		/**\brief Wrapper method around the GRAPH_t::saveToTextFile method.
		 *
		 * Method saves the graph in the format used by TORO & HoG-man strategies
		 *
		 * \sa save3DScene, http://www.mrpt.org/Robotics_file_formats
		 */
		void saveGraph(const std::string& fname) const;
		/**\brief Wrapper method around the mrpt::opengl::COpenGLScene::saveToFile method.
		 */
		void save3DScene() const;
		/**\brief Wrapper method around the COpenGLScene::saveToFile method.
		 */
		void save3DScene(const std::string& fname) const;
		/**\brief Read the configuration variables from the .ini file specified by
		 * the user.
		 *
		 * Method is automatically called, upon CGraphSlamEngine initialization
		 */
		void readConfigFile(const std::string& fname);
		/**\brief Fill in the provided string with the class configuration parameters.
		 *
		 * \sa printParams
		 */
		void getParamsAsString(std::string* params_out) const;
		/**\brief Wrapper around getParamsAsString.
		 * Returns the generated string instead of passing it as an argument to the
		 * call
		 *
		 * \sa printParams
		 */
		std::string getParamsAsString() const;
		/**\brief Print the problem parameters to the console for verification.
		 *
		 * Method is a wrapper around CGraphSlamEngine::getParamsAsString method
		 * \sa getParamsAsString
		 */
		void printParams() const;
		/**\brief Main Class method responsible for reading the .rawlog file.
		 *
		 * Reads the dataset file and builds the graph. Method returns false if
		 * user terminates execution (<em>Ctrl+c</em> is pressed) otherwise true.
		 **/
		bool parseRawlogFile();
		/**\brief Return a reference to the underlying GRAPH_t instance. */
		const GRAPH_t& getGraph() const { return m_graph; }
		/**\brief Return the filename of the used rawlog file.*/
		inline std::string getRawlogFname() {return m_rawlog_fname;}

		/**\brief Parse the ground truth .txt file and fill in the corresponding
		 * m_GT_poses vector.
		 *
		 * It is assumed that the rawlog, thererfore the groundtruth file has been
		 * generated using the <em>GridMapNavSimul</em> MRPT application.
		 * \sa readGTFileRGBD_TUM
		 *
		 * \param[in] fname_GT Ground truth filename from which the measurements
		 * are to be read
		 * \param[out] gt_poses std::vector which is to contain the
		 * 2D ground truth poses.
		 * \param[out] gt_timestamps std::vector which is to contain the timestamps
		 * for the corresponding ground truth poses. Ignore this argument if
		 * timestamps are not needed.
		 */
		static void readGTFileNavSimulOutput(
				const std::string& fname_GT,
				std::vector<pose_t>* gt_poses,
				std::vector<mrpt::system::TTimeStamp>* gt_timestamps=NULL);
		/**\brief Parse the ground truth .txt file and fill in the corresponding
		 * m_GT_poses vector. The poses returned are given with regards to the
		 * MRPT reference frame.
		 *
		 * It is assumed that the groundtruth file has been generated using the
		 * <em>rgbd_dataset2rawlog</em> MRPT tool.
		 *
		 * \param[in] fname_GT Ground truth filename from which the measurements
		 * are to be read
		 * \param[out] gt_poses std::vector which is to contain the
		 * 2D ground truth poses.
		 * \param[out] gt_timestamps std::vector which is to contain the timestamps
		 * for the corresponding ground truth poses. Ignore this argument if
		 * timestamps are not needed.
		 *
		 * \sa readGTFileNavSimulOutput,
		 * http://www.mrpt.org/Collection_of_Kinect_RGBD_datasets_with_ground_truth_CVPR_TUM_2011
		 */
		static void readGTFileRGBD_TUM(
				const std::string& fname_GT,
				std::vector<pose_t>* gt_poses,
				std::vector<mrpt::system::TTimeStamp>* gt_timestamps=NULL);

	private:
		// Private function definitions
		//////////////////////////////////////////////////////////////

		/**\brief General initialization method to call from the Class Constructors*/
		void initCGraphSlamEngine();
		/**\brief Initialize (clean up and create new files) the output directory.
		 *
		 * If directory already exists (most probably from previous runs), the user
		 * is given 3 options:
		 * - Remove the current directory contents
		 * - Rename (and keep) the current directory contents
		 * - Manually handle the conflict
		 *
		 * User can also set the .ini parameter user_decides_about_output_dir  flag
		 * to false, if he doesn't care about the previous results directory. In
		 * this case the 1st choice is picked.
		 *
		 * \sa CGraphSlamEngine::initResultsFile
		 */
		void initOutputDir();
		/**\brief Automate the creation and initialization of a results file relevant to
		 * the application.
		 *
		 * Open the file (corresponding to the provided filename) and write an
		 * introductory message.
		 *
		 * \sa initOutputDir
		 */
		void initResultsFile(const std::string& fname);
		/**\brief Fill the provided string with a detailed report of the class state
		 *
		 * Report includes the following:
		 * - Timing of important methods
		 * - Properties fo class at the current time
		 * - Logging of commands until current time
		 *
		 * \note Decider/Optimizer classes should also implement a getDescriptiveReport
		 * method for printing information on their part of the execution
		 */
		void getDescriptiveReport(std::string* report_str) const;
		/**
		 *\brief Generate and write to disk a report For each of the respective
		 * self/decider/optimizer classes.
		 *
		 * Report files are generated in the output directory as set by the user in
		 * the .ini configuration file [default = graphslam_engine_results/]
		 *
		 * \sa getDescriptiveReport
		 */
		void generateReportFiles();

		/** \name Initialization of Visuals
		 * Methods used for initializing various visualization features relevant to
		 * the application at hand. If the visual feature is specified by the user
		 * (via the .ini file) and if it is relevant to the application then the
		 * corresponding method is called in the initCGraphSlamEngine class method
		 */
		/**\{*/

		/**\brief If \b m_enable_visuals is specified then initialize the primitive
		 * openGL/gui objects related to visualization.
		 */
		void initVisualization();

		void initRangeImageViewport();
		void initIntensityImageViewport();
		void initCurrPosViewport();
		void initGTVisualization();
		void initOdometryVisualization();
		void initEstimatedTrajectoryVisualization();
		void initSlamMetricVisualization();
		/**\}*/

		/** \name Update of Visuals
		 * Methods used for updating various visualization features relevant to
		 * the application at hand. If relevant to the application at hand update
		 * is periodically scheduled inside the parseRawlogFile method
		 */
		/**\{*/
		/**\brief In RGB-D TUM Datasets update the Range image displayed in a
		 * seperate viewport
		 */
		void updateRangeImageViewport();
		/**\brief In RGB-D TUM Datasets update the Intensity image displayed in a
		 * seperate viewport
		 */
		void updateIntensityImageViewport();
		/**\brief Update the viewport responsible for displaying the graph-building
		 * procedure in the estimated position of the robot
		 */
		inline void updateCurrPosViewport();
		/**\brief Update the map visualization based on the current graphSLAM
		 * state.
		 *
		 * Map is produced by arranging the range scans based on the estimated
		 * robot trajectory.
		 *
		 * \sa updateEstimatedTrajectoryVisualization
		 */
		void updateMapVisualization(const GRAPH_t& gr,
				std::map<const mrpt::utils::TNodeID,
				mrpt::obs::CObservation2DRangeScanPtr> m_nodes_to_laser_scans,
				bool full_update=false );
		/**\brief Display the next ground truth position in the visualization window.
		 *
		 * \sa updateOdometryVisualization
		 */
		void updateGTVisualization();
		/**\brief Update odometry-only cloud with latest odometry estimation.
		 *
		 * \sa updateGTVisualization
		 * */
		void updateOdometryVisualization();
		/**\brief Update the Edstimated robot trajectory with the latest estimated
		 * robot position.
		 *
		 * Update CSetOfLines visualization object with the latest graph node
		 * position. If full update is asked, method clears the CSetOfLines
		 * object and redraws all the lines based on the updated (optimized)
		 * positions of the nodes
		 */
		void updateEstimatedTrajectoryVisualization(bool full_update=false);
		/**\brief Update the displayPlots window with the new information with
		 * regards to the metric
		 */
		void updateSlamMetricVisualization();
		/**\}*/
		/** \name Toggling of Visuals
		 * Methods used for toggling various visualization features relevant to
		 * the application at hand.
		 */
		/**\{*/
		void toggleOdometryVisualization();
		void toggleGTVisualization();
		void toggleMapVisualization();
		void toggleEstimatedTrajectoryVisualization();
		/**\}*/

		/**\brief Cut down on the size of the given laser scan.
		 *
		 * Handy for reducing the size of the resulting mrpt::opengl::CSetOfObjects
		 * that would be inserted in the visualization scene. Increase the
		 * decimation rate - keep-every_n_entries - to reduce the computational
		 * cost of updating the map visualization
		 *
		 * \sa updateMapVisualization
		 */
		void decimateLaserScan(
				mrpt::obs::CObservation2DRangeScan& laser_scan_in,
				mrpt::obs::CObservation2DRangeScan* laser_scan_out,
				const int keep_every_n_entries = 2);
		void alignOpticalWithMRPTFrame(); // TODO - either use it or remove it
		/**\brief Query the observer instance for any user events.
		 *
		 * Query the given observer for any events (keystrokes, mouse clicks,
		 * that may have occured in the CDisplayWindow3D  and fill in the
		 * corresponding class variables
		 */
		inline void queryObserverForEvents();

		/** \brief Comapre the SLAM result (estimated trajectory) with the GT path.
		 *
		 * See <a href="http://europa.informatik.uni-freiburg.de/files/burgard09iros.pdf">
		 * A Comparison of SLAM Algorithms Based on a Graph of Relations</a>
		 * for more details on this.
		 */
		void computeSlamMetric(mrpt::utils::TNodeID nodeID,
				size_t gt_index);

		/**\brief Wrapper method that makes use of the COutputLogger instance.
		 *
		 * Used for printing error messages in a consistent manner when toggling certain
		 * visual features in the display window
		 */
		void dumpVisibilityErrorMsg(std::string viz_flag,
				int sleep_time=500 /* ms */);

		// VARIABLES
		//////////////////////////////////////////////////////////////

		mrpt::utils::COutputLogger m_out_logger; /**<Output logger instance */
		mrpt::utils::CTimeLogger m_time_logger; /**<Time logger instance */

		// the graph object to be built and optimized
		GRAPH_t m_graph;

		// registrator instances
		NODE_REGISTRAR m_node_registrar;
		EDGE_REGISTRAR m_edge_registrar;
		OPTIMIZER m_optimizer;

		std::string	m_config_fname;
		std::string	m_rawlog_fname;

		std::string	m_fname_GT;
 		/**\brief Determine if we are to enable visualizatio support or not. */
		bool m_enable_visuals;

		size_t m_GT_poses_index; /**\brief Counter for reading back the GT_poses. */
		size_t m_GT_poses_step; //**\brief Rate at which to read the GT poses. */

		// parameters related to the application generated files
		std::string	m_output_dir_fname;
		bool m_user_decides_about_output_dir;
		bool m_save_graph;
		std::string	m_save_graph_fname;
		bool m_save_3DScene;
		std::string	m_save_3DScene_fname;

		bool m_has_read_config;
		bool m_observation_only_rawlog;

		// keeps track of the out fstreams so that they can be
		// closed (if still open) in the class Dtor.
		fstreams_out m_out_streams;

		// visualization objects
		mrpt::gui::CDisplayWindow3D* m_win; /**<Pointer to the display window at hand */
																				/**< It is supported by the calling */
																				/**< application */
		mrpt::graphslam::CWindowObserver* m_win_observer; /**<Pointer ot a CWindowObserver instance */
																								/**<If not on headless mode, */
																								/**< this should be passed as a*/
																								/**< class member of a */
																								/**< CWindowManager instance */
		mrpt::gui::CDisplayWindowPlots* m_win_plot;	 /**< DisplayPlots instance for visualizing the evolution of the SLAM metric */
		mrpt::graphslam::CWindowManager m_win_manager;/**<Pointer to the window manager*/
																							/**< It is supported by the calling */
																							/**< application */

		/** \name Visualization - related variables
		 * \brief Flags for visualizing various trajectories/objects of interest.
		 *
		 * These are set from the .ini configuration file. The actual visualization
		 * of these objects can be overriden if the user issues the corresponding
		 * keystrokes in the CDisplayWindow3D. In order for them to have any
		 * effect, a pointer to CDisplayWindow3D has to be given first.
		 */
		/**\{*/
		bool m_visualize_odometry_poses;
		bool m_visualize_GT;
		bool m_visualize_map;
		bool m_visualize_estimated_trajectory;
		bool m_visualize_SLAM_metric;
		bool m_enable_curr_pos_viewport;
		bool m_enable_intensity_viewport;
		bool m_enable_range_viewport;
		/**\}*/

		bool m_request_to_exit;
		bool m_program_paused;

		/**\name textMessage-related Paarameters
		 * Parameters relevant to the textMessages appearing in the visualization
		 * window. These are divided into
		 * - Y offsets: vertical position of the textMessage, starting from the top
		 *   side.
		 * - Indices: Unique ID number of each textMessage, used for updating it
		 */
		/**\{*/
		// textMessage vertical text position
		double m_offset_y_odometry;
		double m_offset_y_GT;
		double m_offset_y_estimated_traj;
		double m_offset_y_timestamp;

		// textMessage index
		int m_text_index_odometry;
		int m_text_index_GT;
		int m_text_index_estimated_traj;
		int m_text_index_timestamp;
		/**\}*/

		/**\name User available keystrokes
		 * Keystrokes for toggling the corresponding objects in the CDisplayWindow
		 * upon user press
		 */
		/**\{*/
		std::string m_keystroke_pause_exec;
		std::string m_keystroke_odometry;
		std::string m_keystroke_GT;
		std::string m_keystroke_estimated_trajectory;
		std::string m_keystroke_map;
		/**\}*/

		mrpt::graphslam::CEdgeCounter m_edge_counter; /**< Instance to keep track of all the edges + visualization related operations */
		int m_num_of_edges_for_collapse;

		/**\brief Flag for specifying if we are going to use ground truth data at all.
		 *
		 * This is set to true either if the evolution of the SLAM metric or the
		 * ground truth visualization is set to true.
		 */
		bool m_use_GT;

		// pose_t vectors
		std::vector<pose_t*> m_odometry_poses;
		std::vector<pose_t> m_GT_poses;
		std::string m_GT_file_format;

		std::map<const mrpt::utils::TNodeID,
			mrpt::obs::CObservation2DRangeScanPtr> m_nodes_to_laser_scans2D;
		mrpt::obs::CObservation2DRangeScanPtr m_last_laser_scan2D;

		mrpt::obs::CObservation3DRangeScanPtr m_last_laser_scan3D;

		/**\name Trajectories colors */
		/**\{*/
		mrpt::utils::TColor m_odometry_color;
		mrpt::utils::TColor m_GT_color;
		mrpt::utils::TColor m_estimated_traj_color;
		mrpt::utils::TColor m_optimized_map_color;
		/**\}*/

		// frame transformation from the RGBD_TUM GrountTruth to the MRPT
		// reference frame
		// TODO - either use it or lose it...
		CMatrixDouble33  m_rot_TUM_to_MRPT;

		size_t m_robot_model_size; /**< How big are the robots going to be in the scene */

		/**\brief Internal counter for querrying for the number of nodeIDs.
		 *
		 * Handy for not locking the m_graph resource
		 */
		mrpt::utils::TNodeID m_nodeID_max;

		/** Mark graph modification/accessing explicitly for multithreaded
		 * implementation
		 */
		mrpt::synch::CCriticalSection m_graph_section;

		// keep track of the storage directory for the 3DRangeScan depth/range
		// images
		std::string m_img_external_storage_dir;
		std::string m_img_prev_path_base;

		/**\name Slam Metric related variables */
		/**\{*/
		/** Map from nodeIDs to their corresponding closest GT pose index.  Keep
		 * track of the nodeIDs instead of the node positions as the latter are
		 * about to change in the Edge Registaration / Loop closing procedures
		 */
		std::map<mrpt::utils::TNodeID, size_t> m_nodeID_to_gt_indices;
		double m_curr_deformation_energy;
		std::vector<double> m_deformation_energy_vec;
		/** How much should I scale up the deformation energy values so that they
		 * can be visualized appropriately in the displayPlots window
		 */
		size_t m_deformation_energy_plot_scale;
		/**\}*/

		/**\brief Struct responsible for keeping the parameters of the .info file
		 * in RGBD related datasets
		 */
		struct TRGBDInfoFileParams {

			TRGBDInfoFileParams();
			TRGBDInfoFileParams(std::string rawlog_fname);

			void initTRGBDInfoFileParams();
			/**\brief Parse the RGBD information file to gain information about the rawlog
			 * file contents
			 */
			void parseFile();
			void setRawlogFile(std::string rawlog_fname);

			std::map<std::string, std::string> fields; /**< Format for the parameters in the info file:*/
																								 /**< <em>string literal - related value</em> (kept in a string representation)*/

			std::string info_fname;


		} m_info_params;

		/**\brief Time it took to record the dataset.
		 * Processing time should (at least) be equal to the grab time for the
		 * algorithm to run in real-time
		 */
		double m_dataset_grab_time;

		const std::string m_class_name;
};

} } // end of namespaces

// pseudo-split the definition and implementation of template
#include "CGraphSlamEngine_impl.h"

#endif /* end of include guard: GRAPHSLAMENGINE_H */
