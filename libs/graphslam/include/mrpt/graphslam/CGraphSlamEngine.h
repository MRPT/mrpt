/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef GRAPHSLAMENGINE_H
#define GRAPHSLAMENGINE_H

#include <mrpt/math/CQuaternion.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/COctoMap.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // It's in the lib mrpt-maps now
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/obs_utils.h>
#include <mrpt/utils/CProbabilityDensityFunction.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
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
#include <mrpt/utils/COutputLogger.h>

#include <mrpt/graphslam/misc/CEdgeCounter.h>
#include <mrpt/graphslam/interfaces/CNodeRegistrationDecider.h>
#include <mrpt/graphslam/interfaces/CEdgeRegistrationDecider.h>
#include <mrpt/graphslam/interfaces/CGraphSlamOptimizer.h>

#include <cstdlib>
#include <string>
#include <sstream>
#include <map>
#include <cerrno>
#include <cmath> // fabs function, power
#include <set>
#include <algorithm>
#include <cstdlib>

namespace mrpt { namespace graphslam {

/**\brief Main file for the GraphSlamEngine.
 *
 * ## Description
 *
 * Given a dataset of measurements build a graph of nodes (keyframes) and
 * constraints (edges) and solve it to find an estimation of the actual robot
 * trajectory.
 *
 * // TODO - change this description
 * The template arguments are listed below:
 * - \em GRAPH_T: The type of Graph to be constructed and optimized. Currently
 *   CGraphSlamEngine works only with CPosePDFGaussianInf GRAPH_T instances.
 * - \em NODE_REGISTRAR: Class responsible of adding new nodes in the graph.
 *   Class should at least implement the deciders::CNodeRegistrationDecider
 *   interface provided in CNodeRegistrationDecider.h file.
 * - \em EDGE_REGISTRAR: Class responsible of adding new edges in the graph.
 *   Class should at least implement the deciders::CEdgeRegistrationDecider
 *   interface provided in CEdgeRegistrationDecider.h file.
 * - \em OPTIMIZER: Class responsible of optimizing the graph. Class should at
 *   least implement the optimizers::CGraphSlamOptimizer interface provided
 *   in CGraphSlamOptimizer.h file.
 *
 * \note The GRAPH_T resource is accessed after having locked the relevant section
 * \em m_graph_section. Critical section is also <em> locked prior to the calls
 * to the deciders/optimizers </em>.
 *
 * ### .ini Configuration Parameters
 *
 * \htmlinclude graphslam-engine_config_params_preamble.txt
 *
 * - \b output_dir_fname
 *   + \a Section       : GeneralConfiguration
 *   + \a Default value : 1 (LVL_INFO)
 *   + \a Required      : FALSE
 *
 * - \b user_decides_about_output_dir
 *   + \a Section       : GeneralConfiguration
 *   + \a Default value : FALSE
 *   + \a Required      : FALSE
 *   + \a Description   : If flag true and in case of name conflict with output directory of the
 *   previous execution, a command-line is presented to the user to decide what
 *   to do about the new output directory. By default output directory from
 *   previous run is overwritten by the directory of the current run.
 *
 * - \b ground_truth_file_format
 *   + \a Section       : GeneralConfiguration
 *   + \a Default value : NavSimul
 *   + \a Required      : FALSE
 *   + \a Description   : Specify the format of the ground-truth file if one is
 *   provided. Currently CGraphSlamEngine supports ground truth files generated
 *   by the GridMapNavSimul tool or ground truth files corresponding to
 *   RGBD-TUM datasets.
 *   + \a Available Options: NavSimul, RGBD_TUM
 *
 * - \b class_verbosity
 *   + \a Section       : GeneralConfiguration
 *   + \a Default value : 1 (LVL_INFO)
 *   + \a Required      : FALSE
 *
 *
 * - \b visualize_map
 *   + \a Section       : VisualizationParameters
 *   + \a Default value : TRUE
 *   + \a Required      : FALSE
 *
 * - \b visualize_odometry_poses
 *   + \a Section       : VisualizationParameters
 *   + \a Default value : TRUE
 *   + \a Required      : FALSE
 *
 * - \b visualize_estimated_trajectory
 *   + \a Section       : VisualizationParameters
 *   + \a Default value : TRUE
 *   + \a Required      : FALSE
 *
 * - \b visualize_GT
 *   + \a Section       : VisualizationParameters
 *   + \a Default value : TRUE
 *   + \a Required      : FALSE
 *
 * - \b visualize_SLAM_metric
 *   + \a Section       : VisualizationParameters
 *   + \a Default value : TRUE
 *   + \a Required      : FALSE
 *
 * - \b enable_curr_pos_viewport
 *   + \a Section       : VisualizationParameters
 *   + \a Default value : TRUE
 *   + \a Required      : FALSE
 *   + \a Description   : Applicable only when dealing with RGB-D datasets
 *
 * - \b enable_range_viewport
 *   + \a Section       : VisualizationParameters
 *   + \a Default value : TRUE
 *   + \a Required      : FALSE
 *   + \a Description   : Applicable only when dealing with RGB-D datasets
 *
 * - \b enable_intensity_viewport
 *   + \a Section       : VisualizationParameters
 *   + \a Default value : FALSE
 *   + \a Required      : FALSE
 *   + \a Description   : Applicable only when dealing with RGB-D datasets
 *
 *
 * \note Implementation can be found in the file \em CGraphSlamEngine_impl.h
 * \ingroup mrpt_graphslam_grp
 */
template<class GRAPH_T=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CGraphSlamEngine : public mrpt::utils::COutputLogger {
	public:

		/**\brief Handy typedefs */
		/**\{*/
		/**\brief Map for managing output file streams.*/
		typedef std::map<std::string, mrpt::utils::CFileOutputStream*> fstreams_out;
		/**\brief Map for iterating over output file streams.*/
		typedef std::map<std::string, mrpt::utils::CFileOutputStream*>::iterator fstreams_out_it;

		/**\brief Type of graph constraints */
		typedef typename GRAPH_T::constraint_t constraint_t;
		/**\brief Type of underlying poses (2D/3D). */
		typedef typename GRAPH_T::constraint_t::type_value pose_t;
		typedef typename GRAPH_T::global_pose_t global_pose_t;
		typedef std::map<
			mrpt::utils::TNodeID,
			mrpt::obs::CObservation2DRangeScanPtr> nodes_to_scans2D_t;
		/**\}*/

		/**\brief Constructor of CGraphSlamEngine class template.
		 *
		 * // TODO - remove the deprecated arguments
		 * \param[in] config_file .ini file containing the configuration
		 * parameters for the CGraphSlamEngine as well as the deciders/optimizer
		 * classes that CGraphSlamEngine is using
		 * \param[in] win_manager CwindowManager instance that includes a pointer
		 * to a CDisplayWindow3D and a CWindowObserver instance for properly
		 * interacting with the display window
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
		 * // TODO add the deciders/optimizer
		 *
		 *
		 * \note If a NULL CWindowManager pointer is porovided, the application
		 * runs on <em> headless mode </em>. In this case, no visual feedback is
		 * given but application receives a big boost in performance
		 */
		CGraphSlamEngine(
				const std::string& config_file,
				const std::string& rawlog_fname="",
				const std::string& fname_GT="",
				mrpt::graphslam::CWindowManager* win_manager=NULL,
				mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>* node_reg=NULL,
				mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_T>* edge_reg=NULL,
				mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_T>* optimizer=NULL
				);
		/**\brief Default Destructor. */
		virtual ~CGraphSlamEngine();

		// Public function definitions
		//////////////////////////////////////////////////////////////
		/**\brief Query CGraphSlamEngine instance for the current estimated robot
		 * position
		 */
		global_pose_t getCurrentRobotPosEstimation() const;
		/***\brief Get the estimated trajectory of the robot given by the running
		 * graphSLAM algorithm.
		 * \param[out] graph_nodes Nodes of the graph that have been registered so
		 * far. graph_nodes contains a map of nodeIDs to their corresponding poses.
		 */
		virtual void getRobotEstimatedTrajectory(
				typename GRAPH_T::global_poses_t* graph_poses) const;
		/**\brief Return the list of nodeIDs which make up robot trajectory
		 * \sa updateEstimatedTrajectoryVisualization
		 */
		virtual void getNodeIDsOfEstimatedTrajectory(
				std::set<mrpt::utils::TNodeID>* nodes_set) const;
		/**\brief Wrapper method around the GRAPH_T::saveToTextFile method.
		 * Method saves the graph in the format used by TORO & HoG-man strategies
		 *
		 * \param[in] fname_in Name of the generated graph file - Defaults to "output_graph" if not
		 * set by the user
		 *
		 * \sa save3DScene, http://www.mrpt.org/Robotics_file_formats
		 */
		void saveGraph(const std::string* fname_in=NULL) const;
		/**\brief Wrapper method around the COpenGLScene::saveToFile method.
		 *
		 * \param[in] Name of the generated graph file - Defaults to "output_graph" if not
		 * set by the user
		 *
		 * \sa saveGraph
		 */
		void save3DScene(const std::string* fname_in=NULL) const;
		/**\brief Read the configuration variables from the <em>.ini file</em> specified by
		 * the user.
		 * Method is automatically called, upon CGraphSlamEngine initialization
		 *
		 */
		void loadParams(const std::string& fname);
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
		/** \name Map computation and acquisition methods
		 * \brief Fill the given map based on the observations that have been
		 * recorded so far.
		 */
		/**\{*/
		/* \brief Method is a wrapper around the computeMap method
		 * \param[out] map Pointer to the map instance that is
		 * to be filled
		 * \param[out] acquisition_time Timestamp that the map was computed at.
		 * This does not (necessarily) matches with the query time since the
		 * cached version is used as long as a new node has not been registered
		 * since the last time the gridmap was computed.
		 *
		 * \sa computeMap
		 */
		void getMap(mrpt::maps::COccupancyGridMap2DPtr map,
				mrpt::system::TTimeStamp* acquisition_time=NULL) const;
		void getMap(mrpt::maps::COctoMapPtr map,
				mrpt::system::TTimeStamp* acquisition_time=NULL) const;
		/**\brief	Compute the map of the environment based on the
		 * recorded measurements.
		 *
		 * \warning Currently only mrpt::obs::2DRangeScans are supported
		 * \sa getMap
		 */
		void computeMap() const;
		/**\}*/
		/**\brief Print the problem parameters to the console for verification.
		 *
		 * Method is a wrapper around CGraphSlamEngine::getParamsAsString method
		 * \sa getParamsAsString
		 */
		void printParams() const;
		/**\brief Wrapper method around _execGraphSlamStep.
		 *
		 * Handy for not having to specify any action/observations objects
		 * \return False if the user has requested to exit the graphslam execution
		 * (e.g. pressed ctrl-c), True otherwise
		 */
		bool execGraphSlamStep(
				mrpt::obs::CObservationPtr& observation,
				size_t& rawlog_entry);
		/**\brief Main class method responsible for parsing each measurement and
		 * for executing graphSLAM.
		 *
		 * \note Method reads each measurement seperately, so the application that
		 * invokes it is responsibe for fetching the measurements (e.g. from a
		 * rawlog file).
		 *
		 * \return False if the user has requested to exit the graphslam execution
		 * (e.g. pressed ctrl-c), True otherwise
		 **/
		virtual bool _execGraphSlamStep(
				mrpt::obs::CActionCollectionPtr& action,
				mrpt::obs::CSensoryFramePtr& observations,
				mrpt::obs::CObservationPtr& observation,
				size_t& rawlog_entry);

		/**\brief Return a reference to the underlying GRAPH_T instance. */
		const GRAPH_T& getGraph() const { return m_graph; }
		/**\brief Return the filename of the used rawlog file.*/
		inline std::string getRawlogFname() {return m_rawlog_fname;}

		/**\name ground-truth parsing methods */
		/**\{*/
		/**\brief Parse the ground truth .txt file and fill in the corresponding
		 * gt_poses vector.
		 *
		 * It is assumed that the rawlog, thererfore the groundtruth file has been
		 * generated using the <em>GridMapNavSimul</em> MRPT application. If not
		 * user should abide the ground-truth file format to that of the files
		 * generated by the GridMapNavSimul app.
		 *
		 * \sa readGTFileRGBD_TUM
		 *
		 * \param[in] fname_GT Ground truth filename from which the measurements
		 * are to be read
		 * \param[out] gt_poses std::vector which is to contain the 2D ground truth
		 * poses.
		 * \param[out] gt_timestamps std::vector which is to contain the timestamps
		 * for the corresponding ground truth poses. Ignore this argument if
		 * timestamps are not needed.
		 */
		static void readGTFile(
				const std::string& fname_GT,
				std::vector<mrpt::poses::CPose2D>* gt_poses,
				std::vector<mrpt::system::TTimeStamp>* gt_timestamps=NULL);
		static void readGTFile(
				const std::string& fname_GT,
				std::vector<mrpt::poses::CPose3D>* gt_poses,
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
		 * \sa readGTFile,
		 * http://www.mrpt.org/Collection_of_Kinect_RGBD_datasets_with_ground_truth_CVPR_TUM_2011
		 */
		static void readGTFileRGBD_TUM(
				const std::string& fname_GT,
				std::vector<mrpt::poses::CPose2D>* gt_poses,
				std::vector<mrpt::system::TTimeStamp>* gt_timestamps=NULL);

		/**\}*/

		/**\brief Generate and write to a corresponding report for each of the
		 * respective self/decider/optimizer classes.
		 *
		 * \param[in] output_dir_fname directory name to generate the files in.
		 * Directory must be crated prior to this call
		 *
		 * \sa getDescriptiveReport, CGraphSlamHandler::initOutputDir
		 */
		void generateReportFiles(const std::string& output_dir_fname_in);
		/**\brief Fill the given vector with the deformation energy values computed
		 * for the SLAM evaluation metric
		 *
		 * \param[out] vec_out deformation energy vector to be filled
		 * \sa m_deformation_energy_vec
		 */
		void getDeformationEnergyVector(std::vector<double>* vec_out) const;
		/**\brief Fill the given maps with stats regarding the overall execution of
		 * graphslam.
		 */
		bool getGraphSlamStats(
				std::map<std::string, int>* node_stats,
				std::map<std::string, int>* edge_stats,
				mrpt::system::TTimeStamp* timestamp=NULL);

		/**\name pause/resume execution */
		/**\{ */
		bool isPaused() const {
			return m_is_paused;
		}

		void togglePause() {
			if (isPaused()) {
				this->resumeExec();
			}
			else {
				this->pauseExec();
			}
		}
		void resumeExec() const {
			if (!isPaused()) { return; }
			MRPT_LOG_INFO_STREAM( "Program resumed.");
			m_is_paused = false;

			if (m_enable_visuals) {
				this->m_win->addTextMessage(
						0.3, 0.8, "",
						mrpt::utils::TColorf(1.0, 0, 0),
						m_text_index_paused_message);
			}
		}

		void pauseExec() {
			if (isPaused()) { return; }
			MRPT_LOG_WARN_STREAM("Program is paused. "
				<< "Press \"" << m_keystroke_pause_exec << 
				" or \"" << system::upperCase(m_keystroke_pause_exec) << 
				"\" in the dipslay window to resume");
			m_is_paused = true;
			if (m_enable_visuals) {
				this->m_win->addTextMessage(
						0.3, 0.8, m_paused_message,
						mrpt::utils::TColorf(1.0, 0, 0),
						m_text_index_paused_message);
			}

			while (this->isPaused()) {
				mrpt::system::sleep(1000);
				this->queryObserverForEvents();
			}
		}
		/**\} */

	protected:
		// Private function definitions
		//////////////////////////////////////////////////////////////
		/**\brief General initialization method to call from the Class
		 * Constructors.
		 *
		 * \note Method is automatically called in the class constructor
		 */
		void initClass();
		/**\brief Automate the creation and initialization of a results file relevant to
		 * the application.
		 *
		 * Open the file (corresponding to the provided filename) and write an
		 * introductory message.
		 *
		 * \sa CGraphSlamHandler::initOutputDir
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
		 * method for printing information on their part of the execution.
		 */
		void getDescriptiveReport(std::string* report_str) const;
		/** \name Initialization of Visuals
		 * Methods used for initializing various visualization features relevant to
		 * the application at hand. If the visual feature is specified by the user
		 * (via the .ini file) and if it is relevant to the application then the
		 * corresponding method is called in the initClass class method
		 */
		/**\{*/
		void initVisualization();

		void initRangeImageViewport();
		void initIntensityImageViewport();

		mrpt::opengl::CSetOfObjectsPtr initRobotModelVisualization();
		/**\brief Method to help overcome the partial template specialization
		 * restriction of C++. Apply polymorphism by overloading function arguments instead
		 */
		/**\{ */
		mrpt::opengl::CSetOfObjectsPtr initRobotModelVisualizationInternal(
				const mrpt::poses::CPose2D& p_unused);
		mrpt::opengl::CSetOfObjectsPtr initRobotModelVisualizationInternal(
				const mrpt::poses::CPose3D& p_unused);
		/**\} */

		void initCurrPosViewport();
		void initGTVisualization();
		void initOdometryVisualization();
		void initEstimatedTrajectoryVisualization();
		void initSlamMetricVisualization();
		/**\}*/

		/** \name Update of Visuals
		 * Methods used for updating various visualization features relevant to
		 * the application at hand. If relevant to the application at hand update
		 * is periodically scheduled inside the execGraphSlam method
		 */
		/**\{*/
		/**\brief Wrapper around the deciders/optimizer updateVisuals methods
		 */
		void updateAllVisuals();
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
		virtual void updateCurrPosViewport();
		/**\brief return the 3D Pose of a LaserScan that is to be visualized.
		 *
		 * Used during the computeMap call for the occupancy gridmap
		 */
		virtual mrpt::poses::CPose3D getLSPoseForGridMapVisualization(
				const mrpt::utils::TNodeID nodeID) const;
		/**\brief Set the properties of the map visual object based on the nodeID that
		 * it was produced by.
		 * Derived classes may override this method if they want to have different
		 * visual properties (color, shape etc.) for different nodeIDs.
		 *
		 * \note Base class method sets only the color of the object
		 */
		virtual void setObjectPropsFromNodeID(
				const mrpt::utils::TNodeID nodeID,
				mrpt::opengl::CSetOfObjectsPtr& viz_object);
		void initMapVisualization();
		/**\brief Update the map visualization based on the current graphSLAM
		 * state.
		 *
		 * Map is produced by arranging the range scans based on the estimated
		 * robot trajectory.
		 *
		 * \sa updateEstimatedTrajectoryVisualization
		 */
		void updateMapVisualization(
				const std::map<
					mrpt::utils::TNodeID,
					mrpt::obs::CObservation2DRangeScanPtr>& nodes_to_laser_scans2D,
				bool full_update=false);
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
		/**\brief Update the Esstimated robot trajectory with the latest estimated
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
		 * that may have occurred in the CDisplayWindow3D  and fill in the
		 * corresponding class variables
		 */
		inline void queryObserverForEvents();

		/** \brief Compare the SLAM result (estimated trajectory) with the GT path.
		 *
		 * See <a href="http://europa.informatik.uni-freiburg.de/files/burgard09iros.pdf">
		 * A Comparison of SLAM Algorithms Based on a Graph of Relations</a>
		 * for more details on this.
		 */
		void computeSlamMetric(mrpt::utils::TNodeID nodeID,
				size_t gt_index);

		/**\brief Wrapper method that used for printing error messages in a
		 * consistent manner 
		 *
		 * Makes use of the COutputLogger instance. Prints error message when
		 * toggling illegal visual features in the display window
		 */
		void dumpVisibilityErrorMsg(std::string viz_flag,
				int sleep_time=500 /* ms */);
		// TODO - move this to a different module - to be accessed globally
		/**\brief Fill the TTimestamp in a consistent manner.
		 *
		 * Method can be used in both MRPT Rawlog formats
		 *
		 * \param[in] action_ptr Pointer to the action (action-observations format)
		 * \param[in] observations Pointer to list of observations (action-observations format)
		 * \param[in] observation Pointer to single observation (observation-only format)
		 *
		 * \note if both action_ptr and observation_ptr contains valid timestamps, the
		 * action is preferred.
		 *
		 * \return mrpt::system::TTimeStamp
		 */
		static mrpt::system::TTimeStamp getTimeStamp(
				const mrpt::obs::CActionCollectionPtr action,
				const mrpt::obs::CSensoryFramePtr observations,
				const mrpt::obs::CObservationPtr observation);
		// TODO - move these somewhere else.
		/**\name Class specific supplementary functions.
		 */
		/**\{*/
		static double accumulateAngleDiffs(
				const mrpt::poses::CPose2D &p1,
				const mrpt::poses::CPose2D &p2);
		static double accumulateAngleDiffs(
				const mrpt::poses::CPose3D &p1,
				const mrpt::poses::CPose3D &p2);
		/**\}*/
		/**\brief Set the opengl model that indicates the latest position of the
		 * trajectory at hand
		 *
		 * \param[in] model_name Name of the resulting opengl object.
		 * \param[in] model_color Color of the object.
		 * \param[in] model_size Scaling of the object.
		 * \param[in] init_pose Initial position of the object.
		 *
		 * \todo Use an airplane/quad model for 3D operations
		 *
		 * \returns CSetOfObjectsPtr instance.
		 *
		 * \note Different model is used depending on whether we are running 2D or
		 * 3D SLAM.
		 */
		mrpt::opengl::CSetOfObjectsPtr setCurrentPositionModel(
				const std::string& model_name,
				const mrpt::utils::TColor& model_color=mrpt::utils::TColor(0,0,0),
				const size_t model_size=1,
				const pose_t& init_pose=pose_t());

		/**\brief Assert that the given nodes number matches the registered graph
		 * nodes, otherwise throw exception
		 *
		 * \note Method locks the graph internally.
		 *
		 * \raise logic_error if the expected node count mismatches with the
		 * graph current node count.
		 */
		virtual void monitorNodeRegistration(
				bool registered=false,
				std::string class_name="Class");
		/**\brief Wrapper around the GRAPH_T::dijkstra_nodes_estimate
		 *
		 * Update the global position of the nodes
		 */
		void execDijkstraNodesEstimation();

		// VARIABLES
		//////////////////////////////////////////////////////////////
		mrpt::utils::CTimeLogger m_time_logger; /**<Time logger instance */

		/**\brief The graph object to be built and optimized. */
		GRAPH_T m_graph;

		/**\name Decider/Optimizer instances. Delegating the GRAPH_T tasks to these
		 * classes makes up for a modular and configurable design
		 */
		/**\{*/
		mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>* m_node_reg;
		mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_T>* m_edge_reg;
		mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_T>* m_optimizer;
		/**\}*/

 		/**\brief Determine if we are to enable visualization support or not. */
		const bool m_enable_visuals;

		std::string	m_config_fname;

		/**\brief Rawlog file from which to read the measurements.
		 *
		 * If string is empty, process is to be run online
		 */
		std::string	m_rawlog_fname;

		std::string	m_fname_GT;

 		/**\brief Counter for reading back the GT_poses. */
		size_t m_GT_poses_index;
		/**\brief Rate at which to read the GT poses. */
		size_t m_GT_poses_step;

		bool m_user_decides_about_output_dir;

		bool m_has_read_config;
		bool m_observation_only_dataset;

		/**\brief keeps track of the out fstreams so that they can be closed (if
		 * still open) in the class Dtor.
		 */
		fstreams_out m_out_streams;

		/**\name Visualization - related objects */
		/**\{*/
		mrpt::graphslam::CWindowManager* m_win_manager;
		mrpt::gui::CDisplayWindow3D* m_win;
		mrpt::graphslam::CWindowObserver* m_win_observer;
		/**\brief DisplayPlots instance for visualizing the evolution of the SLAM
		 * metric
		 */
		mrpt::gui::CDisplayWindowPlots* m_win_plot;
		/**\}*/

		/** \name Visualization - related flags
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

		/**\brief Indicated if program is temporarily paused by the user
		 */
		mutable bool m_is_paused;

		/**\brief Message to be displayed while paused. */
		const std::string m_paused_message;

		/**\name textMessage - related Parameters
		 * Parameters relevant to the textMessages appearing in the visualization
		 * window. These are divided into
		 * - Y offsets: vertical position of the textMessage, starting from the top
		 *   side.
		 * - Indices: Unique ID number of each textMessage, used for updating it
		 */
		/**\{*/
		/**\brief Offset from the \a left side of the canvas.
		 * Common for all the messages that are displayed on that side.
		 */
		double m_offset_x_left;

		double m_offset_y_odometry;
		double m_offset_y_GT;
		double m_offset_y_estimated_traj;
		double m_offset_y_timestamp;
		double m_offset_y_current_constraint_type;
		double m_offset_y_paused_message;

		int m_text_index_odometry;
		int m_text_index_GT;
		int m_text_index_estimated_traj;
		int m_text_index_timestamp;
		int m_text_index_current_constraint_type;
		int m_text_index_paused_message;
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

		/**\brief Instance to keep track of all the edges + visualization related
 		 * operations
 		 */
		mrpt::graphslam::detail::CEdgeCounter m_edge_counter;

		/**\brief Flag for specifying if we are going to use ground truth data at all.
		 *
		 * This is set to true either if the evolution of the SLAM metric or the
		 * ground truth visualization is set to true.
		 */
		bool m_use_GT;

		// pose_t vectors
		std::vector<pose_t> m_odometry_poses;
		std::vector<pose_t> m_GT_poses;

		std::string m_GT_file_format;

		/**\brief Map of NodeIDs to their corresponding LaserScans.
		 */
		nodes_to_scans2D_t m_nodes_to_laser_scans2D;
		/**\brief Last laser scan that the current class instance received.
		 */
		mrpt::obs::CObservation2DRangeScanPtr m_last_laser_scan2D;
		/**\brief First recorded laser scan - assigned to the root */
		mrpt::obs::CObservation2DRangeScanPtr m_first_laser_scan2D;
		/**\brief Last laser scan that the current class instance received.
		 */
		mrpt::obs::CObservation3DRangeScanPtr m_last_laser_scan3D;

		/**\name Trajectories colors */
		/**\{*/
		mrpt::utils::TColor m_odometry_color;
		mrpt::utils::TColor m_GT_color;
		mrpt::utils::TColor m_estimated_traj_color;
		mrpt::utils::TColor m_optimized_map_color;
		mrpt::utils::TColor m_current_constraint_type_color;
		/**\}*/

		// frame transformation from the RGBD_TUM GrountTruth to the MRPT
		// reference frame
		// TODO - either use it or lose it...
		mrpt::math::CMatrixDouble33  m_rot_TUM_to_MRPT;
 		/** How big are the robots going to be in the scene */
		size_t m_robot_model_size;
		/**\brief Internal counter for querying for the number of nodeIDs.
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
		/**\brief Map from nodeIDs to their corresponding closest GT pose index. 
		 * Keep track of the nodeIDs instead of the node positions as the latter
		 * are about to change in the Edge Registration / Loop closing procedures
		 */
		std::map<mrpt::utils::TNodeID, size_t> m_nodeID_to_gt_indices;
		double m_curr_deformation_energy;
		std::vector<double> m_deformation_energy_vec;
		/**\}*/

		/**\brief Struct responsible for keeping the parameters of the .info file
		 * in RGBD related datasets
		 */
		struct TRGBDInfoFileParams {
			TRGBDInfoFileParams();
			TRGBDInfoFileParams(const std::string& rawlog_fname);
			~TRGBDInfoFileParams();

			void initTRGBDInfoFileParams();
			/**\brief Parse the RGBD information file to gain information about the rawlog
			 * file contents
			 */
			void parseFile();
			void setRawlogFile(const std::string& rawlog_fname);

			/**\brief Format for the parameters in the info file:
			 * <em>string literal - related value</em> (kept in a string representation)
			 */
			std::map<std::string, std::string> fields;

			std::string info_fname;


		} m_info_params;

		/**\brief Time it took to record the dataset.
		 * Processing time should (at least) be equal to the grab time for the
		 * algorithm to run in real-time
		 */
		double m_dataset_grab_time;

		/**\brief First recorded timestamp in the dataset. */
		mrpt::system::TTimeStamp m_init_timestamp;
		/**\brief Current dataset timestamp */
		mrpt::system::TTimeStamp m_curr_timestamp;
		/**\brief Current robot position based solely on odometry */
		pose_t m_curr_odometry_only_pose;

		/**\brief Indicate whether the user wants to exit the application (e.g.
		 * pressed by pressign ctrl-c)
		 */
		bool m_request_to_exit;

		/**\name Map-related objects
		 * \brief Cached version and corresponding flag of map
		 */
		/**\{*/
		mutable mrpt::maps::COccupancyGridMap2DPtr m_gridmap_cached;
		/**\brief Acquired map in CSimpleMap representation */
		mutable mrpt::maps::CSimpleMap m_simple_map_cached;
		mutable mrpt::maps::COctoMapPtr m_octomap_cached;
		/**\brief Indicates if the map is cached.
		 *
		 *\note Common var for both 2D and 3D maps.
		 */
		mutable bool m_map_is_cached;
		/**\brief Timestamp at which the map was computed
		 *
		 *\note Common var for both 2D and 3D maps.
		 */
		mutable mrpt::system::TTimeStamp m_map_acq_time;
		/**\}*/

		std::string m_class_name;
		/**\brief Track the first node registration occurance
		 *
		 * Handy so that we can assign a measurement to the root node as well.
		 */
		bool m_is_first_time_node_reg;

		/**\brief MRPT CNetworkOfPoses constraint classes that are currently
		 * supported
		 */
		std::vector<std::string> m_supported_constraint_types;
		/**\brief Type of constraint currently in use.
		 */
		std::string m_current_constraint_type;

		/**\brief Separator string to be used in debugging messages
		 */
		static const std::string header_sep;
		static const std::string report_sep;
};

} } // end of namespaces

// pseudo-split the definition and implementation of template
#include "CGraphSlamEngine_impl.h"

#endif /* end of include guard: GRAPHSLAMENGINE_H */
