/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */


#include <mrpt/vision.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/opengl.h>
#include <mrpt/gui/CDisplayWindow3D.h>
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
	bool save_results;

	/** Constructor. */
	CDifodoDatasets() : mrpt::vision::CDifodo() 
	{ 
		save_results = 0;
		sum_exec_time = 0;
		acu_rel_error_tras = 0;
		acu_rel_error_rot = 0;
		num_iter = 0;
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

	/** Saves the following data for each observation (distance in meters and angles in radians):
	  * timestamp of the groundtruth(s) - relative error (vx vy vz local_yaw local_pitch local_roll) - ...
	  * ... - abs_error_traslation - abs_error_rotation - execution_time - num_valid_points 
	  
	  * Clarification: the timestamp belongs to the last groundtruth read, but the groundtruth pose is
	  * calculated interpolating between this and the previous one*/
	void writeToLogFile();

};
