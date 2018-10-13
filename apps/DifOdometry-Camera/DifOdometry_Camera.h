/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/vision/CDifodo.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/img/CImage.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/core/round.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <iostream>

#if defined(MRPT_OS_LINUX) && !defined(linux)
#define linux 1  // Seems to be required by OpenNI.h
#endif
#include <OpenNI.h>
#include "legend.xpm"

class CDifodoCamera : public mrpt::vision::CDifodo
{
   public:
	mrpt::gui::CDisplayWindow3D window;
	std::ofstream f_res;

	bool save_results;

	/** Constructor. */
	CDifodoCamera() : mrpt::vision::CDifodo() { save_results = false; }
	/** Initialize the visual odometry method */
	void loadConfiguration(const mrpt::config::CConfigFileBase& ini);

	/** Open camera */
	bool openCamera();

	/** Close camera */
	void closeCamera();

	/** Capture a new depth frame */
	void loadFrame() override;

	/** Create a file to save the estimated trajectory */
	void CreateResultsFile();

	/** Initialize opengl scene */
	void initializeScene();

	/** Refresh the opengl scene */
	void updateScene();

	/** A pre-step that should be performed before starting to estimate the
	 * camera velocity.
	 * It can also be called to reset the estimated trajectory and pose */
	void reset();

	/** Save the pose estimation following the format of the TUM datasets:
	 *
	 * 'timestamp tx ty tz qx qy qz qw'
	 *
	 * Please visit
	 * http://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats for
	 * further details.*/
	void writeTrajectoryFile();

   private:
	unsigned int repr_level;

	/** Opengl scene */
	mrpt::opengl::COpenGLScene::Ptr scene;

	// OpenNI variables to manage the camera
	openni::Status rc;
	openni::Device device;
	openni::VideoMode video_options;
	openni::VideoStream depth_ch;

	/** Clock used to save the timestamp */
	mrpt::system::CTicTac clock;
};
