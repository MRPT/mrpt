/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/vision/CDifodo.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <iostream>

#if defined(MRPT_OS_LINUX) && !defined(linux)
#   define linux 1   // Seems to be required by OpenNI.h
#endif
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
