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
#include <OpenNI.h>
#include "legend.xpm"



class CDifodoCamera : public mrpt::vision::CDifodo {
public:

	mrpt::gui::CDisplayWindow3D	window;			
	std::ofstream		f_res;

	bool save_results;

	/** Constructor. */
	CDifodoCamera() : mrpt::vision::CDifodo() 
	{ 
		save_results = 0;
	}

	/** Initializes the visual odometry method and loads the rawlog file */
	void loadConfiguration( const mrpt::utils::CConfigFileBase &ini );

	/** Open camera */
	bool openCamera();

	/** Close camera */
	void closeCamera();

	/** Capture a new depth frame */
	void loadFrame();

	/** Creates a file to save some results */
	void CreateResultsFile();

	/** Initializes opengl scene */
	void initializeScene();

	/** Updates the opengl scene */
	void updateScene();

	/** Obtains the filtered speed, updates the pose and saves some statistics */
	void filterSpeedAndPoseUpdate();

	/** A pre-step that should be performed before starting to estimate the camera speed,
	  * and can also be called to reset the estimated trajectory and pose */
	void reset();

private:

	mrpt::opengl::COpenGLScenePtr	scene;	//!< Opengl scene

	// OpenNI variables to manage the camera
	openni::Status		rc;
	openni::Device		device;
	openni::VideoMode	video_options;
	openni::VideoStream depth_ch;

	/** Clock used to save the timestamp */
	mrpt::utils::CTicTac clock;

	/** Saves the following data for each observation (distance in meters and angles in radians):
	  * timestamp(s) - x - y - z - yaw - pitch - roll */
	void writeToLogFile();

};
