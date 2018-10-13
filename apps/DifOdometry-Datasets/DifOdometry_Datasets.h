/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/vision/CDifodo.h>
#include <mrpt/math/types_math.h>  // Eigen (with MRPT "plugin" in BaseMatrix<>)
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/img/CImage.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/gui.h>
#include <iostream>

class CDifodoDatasets : public mrpt::vision::CDifodo
{
   public:
	/** Groundtruth camera pose */
	mrpt::poses::CPose3D gt_pose;
	/** Groundtruth camera previous pose */
	mrpt::poses::CPose3D gt_oldpose;

	/** Opengl scene */
	mrpt::opengl::COpenGLScene::Ptr scene;
	mrpt::gui::CDisplayWindow3D window;
	mrpt::obs::CRawlog dataset;
	std::ifstream f_gt;
	std::ofstream f_res;

	unsigned int repr_level;
	unsigned int rawlog_count;
	bool first_pose;
	bool save_results;
	bool dataset_finished;

	/** Constructor. */
	CDifodoDatasets() : mrpt::vision::CDifodo()
	{
		save_results = false;
		first_pose = false;
		dataset_finished = false;
	}

	/** Initialize the visual odometry method and loads the rawlog file */
	void loadConfiguration(const mrpt::config::CConfigFileBase& ini);

	/** Load the depth image and the corresponding groundtruth pose */
	void loadFrame() override;

	/** Create a file to save the trajectory estimates */
	void CreateResultsFile();

	/** Initialize the opengl scene */
	void initializeScene();

	/** Update the opengl scene */
	void updateScene();

	/** A pre-step that should be performed before starting to estimate the
	 * camera speed
	 * As a couple of frames are necessary to estimate the camera motion, this
	 * methods loads the first frame
	 * before any motion can be estimated.*/
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
	// Used to interpolate grountruth poses
	bool groundtruth_ok;
	bool last_groundtruth_ok;

	/** Timestamp of the last groundtruth read */
	double last_groundtruth;
	/** Timestamp of the last observation */
	double timestamp_obs;
	/** Last ground truth read (x y z qx qy qz w) */
	double last_gt_data[7];
};
