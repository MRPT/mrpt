/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/vision.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/opengl.h>
#include <mrpt/gui.h>
#include <iostream>
#include "legend.xpm"


class CDifodoDatasets : public mrpt::vision::CDifodo {
public:
	
	mrpt::poses::CPose3D gt_pose;		//!< Groundtruth camera pose
	mrpt::poses::CPose3D gt_oldpose;	//!< Groundtruth camera previous pose

	mrpt::gui::CDisplayWindow3D	window;	
	mrpt::slam::CRawlog	dataset;		
	std::ifstream		f_gt;
	std::ofstream		f_res;

	unsigned int rawlog_count;
	bool first_pose;
	bool save_results;

	/** Constructor. */
	CDifodoDatasets() : mrpt::vision::CDifodo() 
	{ 
		save_results = 0;
		sum_exec_time = 0;
		acu_rel_error_tras = 0;
		acu_rel_error_rot = 0;
		num_iter = 0;
		first_pose = false;
	}

	/** Initializes the visual odometry method and loads the rawlog file */
	void loadConfiguration( const mrpt::utils::CConfigFileBase &ini );

	/** Loads the depth image and the corresponding groundtruth pose */
	void loadFrame();

	/** Creates a file to save some results */
	void CreateResultsFile();

	/** Initializes opengl scene */
	void initializeScene();

	/** Updates the opengl scene */
	void updateScene();

	/** Obtains the filtered speed, updates the pose and saves some statistics */
	void filterSpeedAndPoseUpdate();

	/** A pre-step that should be performed before starting to estimate the camera speed
	  * As a couple of frames are necessary to estimate the camera motion, this methods loads the first frame
	  * before any motion can be estimated.*/
	void reset();

	/** Shows the average frame to frame error made by Difodo, as well as its absolute pose error
	  * and its average execution time. */
	void showStatistics();

private:

	// Used to ignore consecutive frames with exactly the same depth image (it happens...)
	bool dtzero_before;

	// Used to advoid saving results when the groundtruth is missing
	bool groundtruth_ok;
	bool last_groundtruth_ok;

	float rel_error[6];			//!< Relative frame to frame error in (vx, vy, vz, rel_yaw, rel_pitch, rel_roll)
	float abs_error_tras;		//!< Absolute pose error in traslation (m)
	float acu_rel_error_tras;	//!< Cumulative relative frame to frame error in traslation (m), used in "showStatistics()"
	float abs_error_rot;		//!< Absolute pose error in rotation (rad)
	float acu_rel_error_rot;	//!< Cumulative relative frame to frame error in rotation (rad), used in "showStatistics()"
	float sum_exec_time;		//!< Cumulative execution time, used in "showStatistics()"

	mrpt::opengl::COpenGLScenePtr	scene;	//!< Opengl scene

	unsigned int num_iter;		//!< Iteration count		
	double last_groundtruth;	//!< Timestamp of the last groundtruth read
	double last_gt_data[7];		//!< Last ground truth read (x y z qx qy qz w)

	/** Saves the following data for each observation (distance in meters and angles in radians):
	  * timestamp of the groundtruth(s) - relative error (vx vy vz local_yaw local_pitch local_roll) - ...
	  * ... - abs_error_traslation - abs_error_rotation - execution_time - num_valid_points 
	  
	  * Clarification: the timestamp belongs to the last groundtruth read, but the groundtruth pose is
	  * calculated interpolating between this and the previous one*/
	void writeToLogFile();

};
