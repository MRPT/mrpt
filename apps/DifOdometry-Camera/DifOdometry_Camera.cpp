/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "DifOdometry_Camera.h"
#include <mrpt/utils/types_math.h> // Eigen (with MRPT "plugin" in BaseMatrix<>)
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/datetime.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/stock_objects.h>

using namespace Eigen;
using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::gui;


void CDifodoCamera::loadConfiguration(const utils::CConfigFileBase &ini )
{
	cam_mode = ini.read_int("DIFODO_CONFIG", "cam_mode", 2, true);
	downsample = ini.read_int("DIFODO_CONFIG", "downsample", 2, true);
	rows = ini.read_int("DIFODO_CONFIG", "rows", 50, true);
	cols = ini.read_int("DIFODO_CONFIG", "cols", 60, true);
	fps = ini.read_int("DIFODO_CONFIG", "fps", 30, false);


	//			Resize Matrices and adjust parameters
	//=========================================================
	const unsigned int resh = 640/(cam_mode*downsample);
	const unsigned int resv = 480/(cam_mode*downsample);

	depth.setSize(rows,cols);
	depth_old.setSize(rows,cols);
	depth_inter.setSize(rows,cols);
	depth_ft.setSize(resv,resh);
	depth_wf.setSize(resv,resh);

	du.setSize(rows,cols);
	dv.setSize(rows,cols);
	dt.setSize(rows,cols);
	xx.setSize(rows,cols);
	xx_inter.setSize(rows,cols);
	xx_old.setSize(rows,cols);
	yy.setSize(rows,cols);
	yy_inter.setSize(rows,cols);
	yy_old.setSize(rows,cols);

	border.setSize(rows,cols);
	border.assign(0);
	null.setSize(rows,cols);
	null.assign(0);
	est_cov.assign(0);

	x_incr = 2.0*f_dist*(floor(float(resh)/float(cols))*cols/float(resh))*tan(0.5*fovh)/(cols-1);	//In meters
	y_incr = 2.0*f_dist*(floor(float(resv)/float(rows))*rows/float(resv))*tan(0.5*fovv)/(rows-1);	//In meters																				//In Hz

	//Depth thresholds
	const int dz = floor(float(resv)/float(rows));
	const int dy = floor(float(resh)/float(cols));

	duv_threshold = 0.001*(dz + dy)*(cam_mode*downsample);
	dt_threshold = 0.2*fps;
	dif_threshold = 0.001*(dz + dy)*(cam_mode*downsample);
	difuv_surroundings = 0.0022*(dz + dy)*(cam_mode*downsample);
	dift_surroundings = 0.01*fps*(dz + dy)*(cam_mode*downsample);
}


bool CDifodoCamera::openCamera()
{
	const char* deviceURI = openni::ANY_DEVICE;

	rc = openni::OpenNI::initialize();

	printf("Initialization:\n %s\n", openni::OpenNI::getExtendedError());
	rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	//								Create Depth channel
	//========================================================================================

	rc = depth_ch.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = depth_ch.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth_ch.destroy();
		}
	}
	else
		printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());

	if (!depth_ch.isValid())
	{
		printf("Camera: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 1;
	}

	//						Configure some properties (resolution)
	//========================================================================================

	//Display video modes
	openni::VideoMode vm;
	printf("Camera depth modes are listed:\n");
	for(int i=0; i<depth_ch.getSensorInfo().getSupportedVideoModes().getSize(); i++)
	{
		vm = depth_ch.getSensorInfo().getSupportedVideoModes()[i];
		cout << "depth mode :" << vm.getResolutionX() << "x" << vm.getResolutionY() << "@" << vm.getFps() << "Hz using format " << vm.getPixelFormat() << endl;
	}

	//Set video mode
	const unsigned int resh = 640/cam_mode;
	const unsigned int resv = 480/cam_mode;
	video_options = depth_ch.getVideoMode();
	video_options.setResolution(resh,resv);
	video_options.setFps(fps);
	rc = depth_ch.setVideoMode(video_options);
	rc = depth_ch.setMirroringEnabled(false);

	video_options = depth_ch.getVideoMode();
	printf("\nResolution (%d, %d) \n", video_options.getResolutionX(), video_options.getResolutionY());

	return 0;
}


void CDifodoCamera::loadFrame()
{
	openni::VideoFrameRef framed;
	depth_ch.readFrame(&framed);

	const int height = framed.getHeight();
	const int width = framed.getWidth();
	const unsigned int index_incr = downsample;

	//Read one frame
	const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)framed.getData();
	int rowSize = framed.getStrideInBytes() / sizeof(openni::DepthPixel);

	for (int yc = height-1; yc >= 0; --yc)
	{
		const openni::DepthPixel* pDepth = pDepthRow;
		for (int xc = width-1; xc >= 0; --xc, ++pDepth)
			if ((yc%downsample == 0)&&(xc%downsample == 0))
				depth_wf(yc/downsample,xc/downsample) = 0.001*(*pDepth);

		pDepthRow += rowSize;
	}
}

void CDifodoCamera::CreateResultsFile()
{
	try
	{
		// Open file, find the first free file-name.
		char	aux[100];
		int     nFile = 0;
		bool    free_name = false;

		system::createDirectory("./difodo.results");

		while (!free_name)
		{
			nFile++;
			sprintf(aux, "./difodo.results/experiment_%03u.txt", nFile );
			free_name = !system::fileExists(aux);
		}

		// Open log file:
		f_res.open(aux);

		clock.Tic();

		printf(" Saving results to file: %s \n", aux);
	}
	catch (...)
	{
		printf("Exception found trying to create the 'results file' !!\n");
	}
}


void CDifodoCamera::initializeScene()
{
	global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 1000000;
	window.resize(1000,900);
	window.setPos(900,0);
	window.setCameraZoom(16);
	window.setCameraAzimuthDeg(0);
	window.setCameraElevationDeg(90);
	window.setCameraPointingToPoint(0,0,0);
	window.setCameraPointingToPoint(0,0,1);

	scene = window.get3DSceneAndLock();

	// Lights:
	scene->getViewport()->setNumberOfLights(1);
	CLight & light0 = scene->getViewport()->getLight(0);
	light0.light_ID = 0;
	light0.setPosition(0,0,1,1);

	//Grid (ground)
	CGridPlaneXYPtr ground = CGridPlaneXY::Create();
	scene->insert( ground );

	//Reference
	CSetOfObjectsPtr reference = stock_objects::CornerXYZ();
	scene->insert( reference );

	//					Cameras and points
	//------------------------------------------------------

	//DifOdo camera
	CBoxPtr camera_odo = CBox::Create(math::TPoint3D(-0.02,-0.1,-0.01),math::TPoint3D(0.02,0.1,0.01));
	camera_odo->setPose(cam_pose);
	camera_odo->setColor(0,1,0);
	scene->insert( camera_odo );

	//Frustum
	opengl::CFrustumPtr FOV = opengl::CFrustum::Create(0.3, 5, 57.3*fovh, 57.3*fovv, 1.5f, true, false);
	FOV->setPose(cam_pose);
	scene->insert( FOV );

	//Reference cam
	CSetOfObjectsPtr reference_gt = stock_objects::CornerXYZ();
	reference_gt->setScale(0.2);
	reference_gt->setPose(cam_pose);
	scene->insert( reference_gt );

	//Camera points
	CPointCloudPtr cam_points = CPointCloud::Create();
	cam_points->setColor(1,0,0);
	cam_points->setPointSize(2);
	cam_points->enablePointSmooth(1);
	cam_points->setPose(cam_pose);
	scene->insert( cam_points );

	//Border points
	CPointCloudPtr border_points = CPointCloud::Create();
	border_points->setColor(0,0,1);
	border_points->setPointSize(3);
	border_points->enablePointSmooth(1);
	border_points->setPose(cam_pose);
	scene->insert( border_points );

	//					Trajectories and covarianze
	//-------------------------------------------------------------

	//Dif Odometry
	CSetOfLinesPtr traj_lines_odo = CSetOfLines::Create();
	traj_lines_odo->setLocation(0,0,0);
	traj_lines_odo->setColor(0,0,0);
	traj_lines_odo->setLineWidth(3);
	scene->insert( traj_lines_odo );
	CPointCloudPtr traj_points_odo = CPointCloud::Create();
	traj_points_odo->setColor(0,0.5,0);
	traj_points_odo->setPointSize(4);
	traj_points_odo->enablePointSmooth(1);
	scene->insert( traj_points_odo );

	//Ellipsoid showing covariance
	math::CMatrixFloat33 cov3d(est_cov.topLeftCorner(3,3));
	CEllipsoidPtr ellip = CEllipsoid::Create();
	ellip->setCovMatrix(cov3d);
	ellip->setQuantiles(2.0);
	ellip->setColor(1.0, 1.0, 1.0, 0.5);
	ellip->enableDrawSolid3D(true);
	ellip->setPose(cam_pose);
	scene->insert( ellip );

	//User-interface information
	utils::CImage img_legend;
	img_legend.loadFromXPM(legend_xpm);
	COpenGLViewportPtr legend = scene->createViewport("legend");
	legend->setViewportPosition(20, 20, 348, 200);
	legend->setImageView(img_legend);

	window.unlockAccess3DScene();
	window.repaint();
}


void CDifodoCamera::updateScene()
{
	scene = window.get3DSceneAndLock();

	//Reference gt
	CSetOfObjectsPtr reference_gt = scene->getByClass<CSetOfObjects>(1);
	reference_gt->setPose(cam_pose);

	//Camera points
	CPointCloudPtr cam_points = scene->getByClass<CPointCloud>(0);
	cam_points->clear();
	cam_points->setPose(cam_pose);
	for (unsigned int y=0; y<cols; y++)
		for (unsigned int z=0; z<rows; z++)
		{
			cam_points->insertPoint(depth_inter(z,y), xx_inter(z,y), yy_inter(z,y));
		}

	//Border points
	CPointCloudPtr border_points = scene->getByClass<CPointCloud>(1);
	border_points->clear();
	border_points->setPose(cam_pose);
	for (unsigned int y=0; y<cols; y++)
		for (unsigned int z=0; z<rows; z++)
			if (border(z,y) == 1)
				border_points->insertPoint(depth_inter(z,y), xx_inter(z,y), yy_inter(z,y));

	//DifOdo camera
	CBoxPtr camera_odo = scene->getByClass<CBox>(0);
	camera_odo->setPose(cam_pose);

	//Frustum
	CFrustumPtr FOV = scene->getByClass<CFrustum>(0);
	FOV->setPose(cam_pose);

	//Difodo traj lines
	CSetOfLinesPtr traj_lines_odo = scene->getByClass<CSetOfLines>(0);
	traj_lines_odo->appendLine(cam_oldpose.x(), cam_oldpose.y(), cam_oldpose.z(), cam_pose.x(), cam_pose.y(), cam_pose.z());

	//Difodo traj points
	CPointCloudPtr traj_points_odo = scene->getByClass<CPointCloud>(2);
	traj_points_odo->insertPoint(cam_pose.x(), cam_pose.y(), cam_pose.z());

	//Ellipsoid showing covariance
	math::CMatrixFloat33 cov3d(est_cov.topLeftCorner(3,3));
	CEllipsoidPtr ellip = scene->getByClass<CEllipsoid>(0);
	ellip->setCovMatrix(cov3d);
	ellip->setPose(cam_pose);

	window.unlockAccess3DScene();
	window.repaint();
}


void CDifodoCamera::closeCamera()
{
	depth_ch.destroy();
	openni::OpenNI::shutdown();
}


void CDifodoCamera::reset()
{
	//Reset Difodo
	loadFrame();
	filterAndDownsample();
	calculateCoord();
	calculateDepthDerivatives();
	findNullPoints();
	findBorders();
	findValidPoints();

	cam_pose.setFromValues(0,0,1.5,0,0,0);
	cam_oldpose = cam_pose;

	//Reset scene
	scene = window.get3DSceneAndLock();
	CSetOfLinesPtr traj_lines_odo = scene->getByClass<CSetOfLines>(0);
	traj_lines_odo->clear();
	CPointCloudPtr traj_points_odo = scene->getByClass<CPointCloud>(2);
	traj_points_odo->clear();
	window.unlockAccess3DScene();

	updateScene();
}


void CDifodoCamera::filterSpeedAndPoseUpdate()
{
	//-------------------------------------------------------------------------
	//								Filter speed
	//-------------------------------------------------------------------------

	utils::CTicTac clock;
	clock.Tic();

	//		Calculate Eigenvalues and Eigenvectors
	//----------------------------------------------------------
	Eigen::SelfAdjointEigenSolver<MatrixXf> eigensolver(est_cov);
	if (eigensolver.info() != Eigen::Success)
	{
		printf("Eigensolver couldn't find a solution. Pose is not updated");
		return;
	}

	//First, we have to describe both the new linear and angular speeds in the "eigenvector" basis
	//-------------------------------------------------------------------------------------------------
	MatrixXf Bii, kai_b;
	Bii.setSize(6,6); kai_b.setSize(6,1);
	Bii = eigensolver.eigenvectors();

	kai_b = Bii.colPivHouseholderQr().solve(kai_solver);

	//Second, we have to describe both the old linear and angular speeds in the "eigenvector" basis too
	//-------------------------------------------------------------------------------------------------
	math::CMatrixDouble33 inv_trans;
	math::CMatrixFloat31 v_loc_old, w_loc_old;

	//Express them in the local reference frame first
	cam_pose.getRotationMatrix(inv_trans);
	v_loc_old = inv_trans.inverse().cast<float>()*kai_abs.topRows(3);
	w_loc_old = inv_trans.inverse().cast<float>()*kai_abs.bottomRows(3);

	//Then transform that local representation to the "eigenvector" basis
	MatrixXf kai_b_old;
	kai_b_old.setSize(6,1);
	math::CMatrixFloat61 kai_loc_old;
	kai_loc_old.topRows<3>() = v_loc_old;
	kai_loc_old.bottomRows<3>() = w_loc_old;

	kai_b_old = Bii.colPivHouseholderQr().solve(kai_loc_old);

	//Filter speed
	const float c = 400.0;
	MatrixXf kai_b_fil;
	kai_b_fil.setSize(6,1);
	for (unsigned int i=0; i<6; i++)
	{
		kai_b_fil(i,0) = (kai_b(i,0) + (c*eigensolver.eigenvalues()(i,0) + 0.2)*kai_b_old(i,0))/(1.0 + c*eigensolver.eigenvalues()(i,0) + 0.2);
		//kai_b_fil_d(i,0) = (kai_b_d(i,0) + 0.2*kai_b_old_d(i,0))/(1.0 + 0.2);
	}

	//Transform filtered speed to local and then absolute reference systems
	MatrixXf kai_loc_fil;
	math::CMatrixFloat31 v_abs_fil, w_abs_fil;
	kai_loc_fil.setSize(6,1);
	kai_loc_fil = Bii.inverse().colPivHouseholderQr().solve(kai_b_fil);

	cam_pose.getRotationMatrix(inv_trans);
	v_abs_fil = inv_trans.cast<float>()*kai_loc_fil.topRows(3);
	w_abs_fil = inv_trans.cast<float>()*kai_loc_fil.bottomRows(3);

	kai_abs.topRows<3>() = v_abs_fil;
	kai_abs.bottomRows<3>() = w_abs_fil;


	//-------------------------------------------------------------------------
	//							Update pose (DIFODO)
	//-------------------------------------------------------------------------

	cam_oldpose = cam_pose;

	//Rotations and translations (could be better, and I tried it but there isn't any significant difference...)
	double yaw,pitch,roll;
	math::CMatrixDouble31 w_euler_d;

	cam_pose.getYawPitchRoll(yaw,pitch,roll);
	w_euler_d(0,0) = kai_loc_fil(4,0)*sin(roll)/cos(pitch) + kai_loc_fil(5,0)*cos(roll)/cos(pitch);
	w_euler_d(1,0) = kai_loc_fil(4,0)*cos(roll) - kai_loc_fil(5,0)*sin(roll);
	w_euler_d(2,0) = kai_loc_fil(3,0) + kai_loc_fil(4,0)*sin(roll)*tan(pitch) + kai_loc_fil(5,0)*cos(roll)*tan(pitch);

	//Update pose
	cam_pose.x_incr(v_abs_fil(0,0)/fps);
	cam_pose.y_incr(v_abs_fil(1,0)/fps);
	cam_pose.z_incr(v_abs_fil(2,0)/fps);
	cam_pose.setYawPitchRoll(yaw + w_euler_d(0,0)/fps, pitch + w_euler_d(1,0)/fps, roll + w_euler_d(2,0)/fps);

	execution_time += 1000*clock.Tac();

	if (save_results == 1)
		writeToLogFile();

}


void CDifodoCamera::writeToLogFile()
{
	f_res << clock.Tac() << " ";
	f_res << cam_pose[0] << " ";
	f_res << cam_pose[1] << " ";
	f_res << cam_pose[2] << " ";
	f_res << cam_pose[3] << " ";
	f_res << cam_pose[4] << " ";
	f_res << cam_pose[5] << " ";
	f_res << "\n";
}

