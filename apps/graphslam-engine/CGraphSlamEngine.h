/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// Sun May 22 12:48:25 EEST 2016, nickkouk
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

bool verbose = true;

template< 
  	class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf,
  	class NODE_REGISTRAR=typename mrpt::graphslam::deciders::CFixedIntervalsNRD_t<GRAPH_t>, 
  	class EDGE_REGISTRAR=typename mrpt::graphslam::deciders::CICPGoodnessERD_t<GRAPH_t>,
  	class OPTIMIZER=typename mrpt::graphslam::optimizers::CLevMarqGSO_t<GRAPH_t> >
	class CGraphSlamEngine_t {
		public:

			typedef std::map<std::string, mrpt::utils::CFileOutputStream*> fstreams_out;
			typedef std::map<std::string, mrpt::utils::CFileOutputStream*>::iterator fstreams_out_it;
			typedef std::map<std::string, mrpt::utils::CFileInputStream*> fstreams_in;
			typedef std::map<std::string, mrpt::utils::CFileInputStream*>::iterator fstreams_in_it;

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
					std::string rawlog_fname = "",
					std::string fname_GT = "");
			~CGraphSlamEngine_t();

			// Public function definitions
			//////////////////////////////////////////////////////////////
			/**
		 		* Wrapper method around the GRAPH_t corresponding method
		 		*/
			void saveGraph() const;
			/**
		 		* Wrapper method around the GRAPH_t corresponding method
		 		*/
			void saveGraph(const std::string& fname) const;
			/**
				* Wrapper method around the COpenGLScene::saveToFile method
				*/
			void save3DScene() const;
			/**
				* Wrapper method around the COpenGLScene::saveToFile method
				*/
			void save3DScene(const std::string& fname) const;
			/**
		 		* Read the configuration file specified and fill in the corresponding
		 		* class members
		 		*/
	 		void readConfigFile(const std::string& fname);
			/**
		 		* Print the problem parameters (usually fetched from a configuration
		 		* file) to the console for verification
		 		*
		 		* \sa CGraphSlamEngine_t::parseRawlogFile
		 		*/
			void printProblemParams() const;
			/**
		 		* Reads the file provided and builds the graph. Method returns false if
		 		* user issues termination coe (Ctrl+c) otherwise true
		 		**/
			bool parseRawlogFile();
			/**
		 		* GRAPH_t getter function - return reference to own graph
		 		* Handy function for visualization, printing purposes
		 		*/
			const GRAPH_t& getGraph() const { return m_graph; }
			/**
				* Return the filename of the rawlog file
				*/
			inline std::string getRawlogFname() {return m_rawlog_fname;}

		private:
			// Private function definitions
			//////////////////////////////////////////////////////////////

			/**
		 		* General initialization method to call from the different Ctors
		 		*/
			void initCGraphSlamEngine();
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
			void initRangeImageViewport();
			/**
				* In RGB-D TUM Datasets update the Range image displayed in a seperate
				* viewport
				*/
			void updateRangeImageViewport();
			void initIntensityImageViewport();
			/**
				* In RGB-D TUM Datasets update the Intensity image displayed in a seperate
				* viewport
				*/
			void updateIntensityImageViewport();

			void initCurrPosViewport();
			/**
 				* Udpate the viewport responsible for displaying the graph-building
 				* procedure in the estimated position of the robot
 				*/
			inline void updateCurrPosViewport();
			/**
				* Visualize the estimated path of the robot along with the produced
				* map
				*/
			void updateMapVisualization(const GRAPH_t& gr, 
					std::map<const mrpt::utils::TNodeID, 
					mrpt::obs::CObservation2DRangeScanPtr> m_nodes_to_laser_scans,
					bool full_update=false );
			/**
				* Cut down on the size of the given laser scan. Handy for reducing the
				* size of the resulting CSetOfObject that would be inserted in the
				* visualization scene
				*/
			void decimateLaserScan(
					mrpt::obs::CObservation2DRangeScan& laser_scan_in,
					mrpt::obs::CObservation2DRangeScan* laser_scan_out,
					const int keep_every_n_entries = 2); 
			/**
		 		* Parse the ground truth .txt file and fill in the corresponding
		 		* m_GT_poses vector. Ground Truth file has to be generated from the
		 		* GridMapNavSimul MRPT application.
		 		*/
			void readGTFileNavSimulOutput(
					const std::string& fname_GT, 
					std::vector<pose_t>* gt_poses,
					std::vector<mrpt::system::TTimeStamp>* gt_timestamps=NULL);
			/**
		 		* Parse the ground truth .txt file and fill in the corresponding
		 		* m_GT_poses vector. The poses returned are given with regards to the
		 		* MRPT reference frame. Transformation from the RGB-D optical frame to
		 		* the MRPT frame is applied.
		 		* Ground Truth file has to be generated from the rgbd_dataset2rawlog
		 		* MRPT tool.
		 		*/
			void readGTFileRGBD_TUM(
					const std::string& fname_GT, 
					std::vector<pose_t>* gt_poses,
					std::vector<mrpt::system::TTimeStamp>* gt_timestamps=NULL);
			void initGTVisualization();
			/**
				* Display the next ground truth position in the visualization window
				*/
			void updateGTVisualization();
			void initOdometryVisualization();
			/**
				* Update odometry-only cloud with latest odometry estimation
				*/
			void updateOdometryVisualization();
			void initEstimatedTrajectoryVisualization();
			/**
				* update CSetOfLines viuslization object with the latest graph node
				* position. If full update is asked, method clears the CSetOfLines
				* object and redraws all the lines based on the updated (optimized)
				* positions of the nodes
				*/
			void updateEstimatedTrajectoryVisualization(bool full_update=false);
			/**
		 		* Query the given observer for any events (keystrokes, mouse clicks,
		 		* that may have occured in the CDisplayWindow3D  and fill in the
		 		* corresponding class variables
		 		*/
			inline void queryObserverForEvents();

			// methods related with the toggling of specific visualization objects in
			// the CDisplayWindow3D
			void toggleOdometryVisualization();
			void toggleGTVisualization();
			void toggleMapVisualization();
			void toggleEstimatedTrajectoryVisualization();

			/*
			* comapre the SLAM result (estimated trajectory) with the GT path
			* see "A Comparison of SLAM Algorithms Based on a Graph of Relations"
			* for more details on this.
			*/
			void computeSlamMetric(mrpt::utils::TNodeID nodeID, 
					size_t gt_index);
			void initSlamMetricVisualization();
			void updateSlamMetricVisualization();
			
			// wrapper around std::cout for specifically printing error messages in
			// the visibility toggling from the CDisplayWindow3D
			void dumpVisibilityErrorMsg(std::string viz_flag, 
					int sleep_time=500 /* ms */);

			// VARIABLES
			//////////////////////////////////////////////////////////////

			// logger
			mrpt::utils::COutputLogger_t m_logger;

			// the graph object to be built and optimized
			GRAPH_t m_graph;

			// registrator instances
			NODE_REGISTRAR m_node_registrar; 
			EDGE_REGISTRAR m_edge_registrar; 
			OPTIMIZER m_optimizer;

			/**
		 		* Problem parameters.
		 		* Most are imported from a .ini config file
		 		* \sa CGraphSlamEngine_t::readConfigFile
		 		*/
			std::string	m_config_fname;
			std::string	m_rawlog_fname;

			std::string	m_fname_GT;
			size_t m_GT_poses_index; // counter for reading back the GT_poses
			size_t m_GT_poses_step; // rate at which to read the GT poses

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
			mrpt::gui::CDisplayWindow3D* m_win;
			mrpt::gui::CWindowObserver* m_win_observer;
			mrpt::gui::CDisplayWindowPlots* m_win_plot;
			mrpt::gui::CWindowManager_t m_win_manager;

			// flags for visualizing various trajectories/objects of interest. These
			// are set from the .ini configuration file. The actual visualization of
			// these objects can be overriden if the user issues the corresponding
			// keystrokes in the CDisplayWindow3D.
			bool m_visualize_odometry_poses;
			bool m_visualize_GT;
			bool m_visualize_map;
			bool m_visualize_estimated_trajectory;
			bool m_visualize_SLAM_metric;
			bool m_enable_curr_pos_viewport;
			bool m_enable_intensity_viewport;
			bool m_enable_range_viewport;

			bool m_request_to_exit;
			bool m_program_paused;

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

			// keystrokes for toggling the corresponding objects in the
			// CDisplayWindow when user issues the specified keystroke
			std::string m_keystroke_pause_exec;
			std::string m_keystroke_odometry;
			std::string m_keystroke_GT;
			std::string m_keystroke_estimated_trajectory;
			std::string m_keystroke_map;

			// instance to keep track of all the edges + visualization related
			// functions
			CEdgeCounter_t m_edge_counter;
			int m_num_of_edges_for_collapse;

			// pose_t vectors
			std::vector<pose_t*> m_odometry_poses;
			std::vector<pose_t> m_GT_poses;
			std::string m_GT_file_format;

			std::map<const mrpt::utils::TNodeID, 
				mrpt::obs::CObservation2DRangeScanPtr> m_nodes_to_laser_scans2D;
			mrpt::obs::CObservation2DRangeScanPtr m_last_laser_scan2D;
			std::map<const mrpt::utils::TNodeID, 
				mrpt::obs::CObservation3DRangeScanPtr> m_nodes_to_laser_scans3D;
			mrpt::obs::CObservation3DRangeScanPtr m_last_laser_scan3D;


			// Trajectories colors
			mrpt::utils::TColor m_odometry_color; // see Ctor for initialization
			mrpt::utils::TColor m_GT_color;
			mrpt::utils::TColor m_estimated_traj_color;

			// frame transformation from the RGBD_TUM GrountTruth to the MRPT
			// reference frame
			CMatrixDouble33  m_rot_TUM_to_MRPT;

			size_t m_robot_model_size;

			// internal counter for querrying for the number of nodeIDs.
			// Handy for not locking the m_graph resource
			mrpt::utils::TNodeID m_nodeID_max; 

			// mark graph modification/accessing explicitly for multithreaded
			// implementation
			mrpt::synch::CCriticalSection m_graph_section;

			// keep track of the storage directory for the 3DRangeScan depth/range
			// images
			std::string m_img_external_storage_dir;
			std::string m_img_prev_path_base;

			// Slam Metric related variables
			// map from nodeIDs to their corresponding closest GT pose intex
			// Keep track of the nodeIDs instead of the node positions as the latter
			// are about to change in the Edge Registaration / Loop closing
			// procedures
			std::map<mrpt::utils::TNodeID, size_t> m_nodeID_to_gt_indices;
			double m_curr_deformation_energy;
			std::vector<double> m_deformation_energy_vec;
			size_t m_deformation_energy_plot_scale;
			

			// user-set flag for indicating whether to compute and visualize the SLAM
			// evaluation metric
			// TODO - read them from .ini
			// TODO - write them in printproblemparams
			bool m_compute_SLAM_metric;

			// struct to hold the parameters of the info file generated during the
			// conversion of RGBD TUM dataset .rosbags to rawlog format
			struct TRGBDInfoFileParams {

				TRGBDInfoFileParams();
				TRGBDInfoFileParams(std::string rawlog_fname);

				void initTRGBDInfoFileParams();
				/**
				 	* Parse the RGBD information file to gain information about the rawlog
				 	* file contents
				 	*/
				void parseFile();
				void setRawlogFile(std::string rawlog_fname);

				// format for the parameters in the info file:
				// string literal - related value (kept in a string)
				std::map<std::string, std::string> fields;

				std::string info_fname;


			} m_info_params;
	};

} } // end of namespaces

// pseudo-split the definition and implementation of template
#include "CGraphSlamEngine_impl.h"

#endif /* end of include guard: GRAPHSLAMENGINE_H */
