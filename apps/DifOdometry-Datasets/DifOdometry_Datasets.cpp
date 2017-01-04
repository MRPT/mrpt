/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "DifOdometry_Datasets.h"
#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/stock_objects.h>
#include "legend.xpm"

using namespace Eigen;
using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;

using namespace mrpt::gui;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;



void CDifodoDatasets::loadConfiguration(const utils::CConfigFileBase &ini )
{	
	fovh = M_PIf*62.5f/180.0f;	//Larger FOV because depth is registered with color
	fovv = M_PIf*48.5f/180.0f;
	cam_mode = 1;
	fast_pyramid = false;
	downsample = ini.read_int("DIFODO_CONFIG", "downsample", 2, true);
	rows = ini.read_int("DIFODO_CONFIG", "rows", 240, true);
	cols = ini.read_int("DIFODO_CONFIG", "cols", 320, true);
	ctf_levels = ini.read_int("DIFODO_CONFIG", "ctf_levels", 5, true);
	string filename = ini.read_string("DIFODO_CONFIG", "filename", "no file", true);

	//						Open Rawlog File
	//==================================================================
	if (!dataset.loadFromRawLogFile(filename))
		throw std::runtime_error("\nCouldn't open rawlog dataset file for input...");

	rawlog_count = 0;

	// Set external images directory:
	const string imgsPath = CRawlog::detectImagesDirectory(filename);
	CImage::IMAGES_PATH_BASE = imgsPath;

	//					Load ground-truth
	//=========================================================
	filename = system::extractFileDirectory(filename);
	filename.append("/groundtruth.txt");
	f_gt.open(filename.c_str());

	if (f_gt.fail())
		throw std::runtime_error("\nError finding the groundtruth file: it should be contained in the same folder than the rawlog file");

	char aux[100];
	f_gt.getline(aux, 100);
	f_gt.getline(aux, 100);
	f_gt.getline(aux, 100);
	f_gt >> last_groundtruth;
	f_gt >> last_gt_data[0]; f_gt >> last_gt_data[1]; f_gt >> last_gt_data[2];
	f_gt >> last_gt_data[3]; f_gt >> last_gt_data[4]; f_gt >> last_gt_data[5]; f_gt >> last_gt_data[6];
	last_groundtruth_ok = 1;

	//			Resize matrices and adjust parameters
	//=========================================================
	width = 640/(cam_mode*downsample);
	height = 480/(cam_mode*downsample);
	repr_level = utils::round(log(float(width/cols))/log(2.f));

	//Resize pyramid
    const unsigned int pyr_levels = round(log(float(width/cols))/log(2.f)) + ctf_levels;
    depth.resize(pyr_levels);
    depth_old.resize(pyr_levels);
    depth_inter.resize(pyr_levels);
	depth_warped.resize(pyr_levels);
    xx.resize(pyr_levels);
    xx_inter.resize(pyr_levels);
    xx_old.resize(pyr_levels);
	xx_warped.resize(pyr_levels);
    yy.resize(pyr_levels);
    yy_inter.resize(pyr_levels);
    yy_old.resize(pyr_levels);
	yy_warped.resize(pyr_levels);
	transformations.resize(pyr_levels);

	for (unsigned int i = 0; i<pyr_levels; i++)
    {
        unsigned int s = pow(2.f,int(i));
        cols_i = width/s; rows_i = height/s;
        depth[i].resize(rows_i, cols_i);
        depth_inter[i].resize(rows_i, cols_i);
        depth_old[i].resize(rows_i, cols_i);
        depth[i].assign(0.0f);
        depth_old[i].assign(0.0f);
        xx[i].resize(rows_i, cols_i);
        xx_inter[i].resize(rows_i, cols_i);
        xx_old[i].resize(rows_i, cols_i);
        xx[i].assign(0.0f);
        xx_old[i].assign(0.0f);
        yy[i].resize(rows_i, cols_i);
        yy_inter[i].resize(rows_i, cols_i);
        yy_old[i].resize(rows_i, cols_i);
        yy[i].assign(0.0f);
        yy_old[i].assign(0.0f);
		transformations[i].resize(4,4);

		if (cols_i <= cols)
		{
			depth_warped[i].resize(rows_i,cols_i);
			xx_warped[i].resize(rows_i,cols_i);
			yy_warped[i].resize(rows_i,cols_i);
		}
    }

	//Resize matrix that store the original depth image
	depth_wf.setSize(height,width);
}

void CDifodoDatasets::CreateResultsFile()
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
		printf(" Saving results to file: %s \n", aux);
	}
	catch (...)
	{
		printf("Exception found trying to create the 'results file' !!\n");
	}

}

void CDifodoDatasets::initializeScene()
{
	CPose3D rel_lenspose(0,-0.022,0,0,0,0);
	
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
	//CSetOfObjectsPtr reference = stock_objects::CornerXYZ();
	//scene->insert( reference );

	//					Cameras and points
	//------------------------------------------------------

	//DifOdo camera
	CBoxPtr camera_odo = CBox::Create(math::TPoint3D(-0.02,-0.1,-0.01),math::TPoint3D(0.02,0.1,0.01));
	camera_odo->setPose(cam_pose + rel_lenspose);
	camera_odo->setColor(0,1,0);
	scene->insert( camera_odo );

	//Groundtruth camera
	CBoxPtr camera_gt = CBox::Create(math::TPoint3D(-0.02,-0.1,-0.01),math::TPoint3D(0.02,0.1,0.01));
	camera_gt->setPose(gt_pose + rel_lenspose);
	camera_gt->setColor(1,0,0);
	scene->insert( camera_gt );

	//Frustum
	opengl::CFrustumPtr FOV = opengl::CFrustum::Create(0.3f, 2, 57.3f*fovh, 57.3f*fovv, 1.f, true, false);
	FOV->setColor(0.7,0.7,0.7);
	FOV->setPose(gt_pose);
	scene->insert( FOV );

	//Reference gt
	CSetOfObjectsPtr reference_gt = stock_objects::CornerXYZ();
	reference_gt->setScale(0.2f);
	reference_gt->setPose(gt_pose);
	scene->insert( reference_gt );

	//Camera points
	CPointCloudColouredPtr cam_points = CPointCloudColoured::Create();
	cam_points->setColor(1,0,0);
	cam_points->setPointSize(2);
	cam_points->enablePointSmooth(1);
	cam_points->setPose(cam_pose);
	scene->insert( cam_points );


	//					Trajectories and covariance
	//-------------------------------------------------------------

	//Dif Odometry
	CSetOfLinesPtr traj_lines_odo = CSetOfLines::Create();
	traj_lines_odo->setLocation(0,0,0);
	traj_lines_odo->setColor(0,0.6,0);
	traj_lines_odo->setLineWidth(6);
	scene->insert( traj_lines_odo );
	CPointCloudPtr traj_points_odo = CPointCloud::Create();
	traj_points_odo->setColor(0,0.6,0);
	traj_points_odo->setPointSize(4);
	traj_points_odo->enablePointSmooth(1);
	scene->insert( traj_points_odo );

	//Groundtruth
	CSetOfLinesPtr traj_lines_gt = CSetOfLines::Create();
	traj_lines_gt->setLocation(0,0,0);
	traj_lines_gt->setColor(0.6,0,0);
	traj_lines_gt->setLineWidth(6);
	scene->insert( traj_lines_gt );
	CPointCloudPtr traj_points_gt = CPointCloud::Create();
	traj_points_gt->setColor(0.6,0,0);
	traj_points_gt->setPointSize(4);
	traj_points_gt->enablePointSmooth(1);
	scene->insert( traj_points_gt );

	//Ellipsoid showing covariance
	math::CMatrixFloat33 cov3d = 20.f*est_cov.topLeftCorner(3,3);
	CEllipsoidPtr ellip = CEllipsoid::Create();
	ellip->setCovMatrix(cov3d);
	ellip->setQuantiles(2.0);
	ellip->setColor(1.0, 1.0, 1.0, 0.5);
	ellip->enableDrawSolid3D(true);
	ellip->setPose(cam_pose + rel_lenspose);
	scene->insert( ellip );

	//User-interface information
	utils::CImage img_legend;
	img_legend.loadFromXPM(legend_xpm);
	COpenGLViewportPtr legend = scene->createViewport("legend");
	legend->setViewportPosition(20, 20, 332, 164);
	legend->setImageView(img_legend);

	window.unlockAccess3DScene();
	window.repaint();
}

void CDifodoDatasets::updateScene()
{
	CPose3D rel_lenspose(0,-0.022,0,0,0,0);
	
	scene = window.get3DSceneAndLock();

	//Reference gt
	CSetOfObjectsPtr reference_gt = scene->getByClass<CSetOfObjects>(0);
	reference_gt->setPose(gt_pose);

	//Camera points
	CPointCloudColouredPtr cam_points = scene->getByClass<CPointCloudColoured>(0);
	cam_points->clear();
	cam_points->setPose(gt_pose);
	for (unsigned int y=0; y<cols; y++)
		for (unsigned int z=0; z<rows; z++)
			cam_points->push_back(depth[repr_level](z,y), xx[repr_level](z,y), yy[repr_level](z,y),
									1.f-sqrt(weights(z,y)), sqrt(weights(z,y)), 0);

	//DifOdo camera
	CBoxPtr camera_odo = scene->getByClass<CBox>(0);
	camera_odo->setPose(cam_pose + rel_lenspose);

	//Groundtruth camera
	CBoxPtr camera_gt = scene->getByClass<CBox>(1);
	camera_gt->setPose(gt_pose + rel_lenspose);

	//Frustum
	CFrustumPtr FOV = scene->getByClass<CFrustum>(0);
	FOV->setPose(gt_pose);

	if ((first_pose == true)&&((gt_pose-gt_oldpose).norm() < 0.5))
	{
		//Difodo traj lines
		CSetOfLinesPtr traj_lines_odo = scene->getByClass<CSetOfLines>(0);
		traj_lines_odo->appendLine(cam_oldpose.x(), cam_oldpose.y(), cam_oldpose.z(), cam_pose.x(), cam_pose.y(), cam_pose.z());

		//Difodo traj points
		CPointCloudPtr traj_points_odo = scene->getByClass<CPointCloud>(0);
		traj_points_odo->insertPoint(cam_pose.x(), cam_pose.y(), cam_pose.z());

		//Groundtruth traj lines
		CSetOfLinesPtr traj_lines_gt = scene->getByClass<CSetOfLines>(1);
		traj_lines_gt->appendLine(gt_oldpose.x(), gt_oldpose.y(), gt_oldpose.z(), gt_pose.x(), gt_pose.y(), gt_pose.z());

		//Groundtruth traj points
		CPointCloudPtr traj_points_gt = scene->getByClass<CPointCloud>(1);
		traj_points_gt->insertPoint(gt_pose.x(), gt_pose.y(), gt_pose.z());
	}

	//Ellipsoid showing covariance
	math::CMatrixFloat33 cov3d = 20.f*est_cov.topLeftCorner(3,3);
	CEllipsoidPtr ellip = scene->getByClass<CEllipsoid>(0);
	ellip->setCovMatrix(cov3d);
	ellip->setPose(cam_pose + rel_lenspose);

	window.unlockAccess3DScene();
	window.repaint();
}

void CDifodoDatasets::loadFrame()
{
	CObservationPtr alfa = dataset.getAsObservation(rawlog_count);

	while (!IS_CLASS(alfa, CObservation3DRangeScan))
	{
		rawlog_count++;
		if (dataset.size() <= rawlog_count)
		{
			dataset_finished = true;
			return;
		}
		alfa = dataset.getAsObservation(rawlog_count);
	}

	CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(alfa);
	obs3D->load();
	const CMatrix range = obs3D->rangeImage;
	const unsigned int height = range.getRowCount();
	const unsigned int width = range.getColCount();

	for (unsigned int j = 0; j<cols; j++)
		for (unsigned int i = 0; i<rows; i++)
		{
			const float z = range(height-downsample*i-1, width-downsample*j-1);
			if (z < 4.5f)	depth_wf(i,j) = z;
			else			depth_wf(i, j) = 0.f;
		}


	double timestamp_gt;
	timestamp_obs = mrpt::system::timestampTotime_t(obs3D->timestamp);

	//Exit if there is no groundtruth at this time
	if (last_groundtruth > timestamp_obs)
	{
		groundtruth_ok = 0;
		obs3D->unload();
		rawlog_count++;
		return;
	}

	//Search the corresponding groundtruth data and interpolate
	bool new_data = 0;
	last_groundtruth_ok = groundtruth_ok;
	while (last_groundtruth < timestamp_obs - 0.01)
	{
		f_gt.ignore(100,'\n');
		f_gt >> timestamp_gt;
		last_groundtruth = timestamp_gt;
		new_data = 1;

		if (f_gt.eof())
		{
			dataset_finished = true;
			return;
		}
	}

	//Read the inmediatly previous groundtruth
	double x0,y0,z0,qx0,qy0,qz0,w0,t0;
	if (new_data == 1)
	{
		f_gt >> x0; f_gt >> y0; f_gt >> z0;
		f_gt >> qx0; f_gt >> qy0; f_gt >> qz0; f_gt >> w0;
	}
	else
	{
		x0 = last_gt_data[0]; y0 = last_gt_data[1]; z0 = last_gt_data[2];
		qx0 = last_gt_data[3]; qy0 = last_gt_data[4]; qz0 = last_gt_data[5]; w0 = last_gt_data[6];
	}

	t0 = last_groundtruth;

	//Read the inmediatly posterior groundtruth
	f_gt.ignore(10,'\n');
	f_gt >> timestamp_gt;
	if (f_gt.eof())
	{
		dataset_finished = true;
		return;
	}
	last_groundtruth = timestamp_gt;

	//last_gt_data = [x y z qx qy qz w]
	f_gt >> last_gt_data[0]; f_gt >> last_gt_data[1]; f_gt >> last_gt_data[2];
	f_gt >> last_gt_data[3]; f_gt >> last_gt_data[4]; f_gt >> last_gt_data[5]; f_gt >> last_gt_data[6];

	if (last_groundtruth - timestamp_obs > 0.01)
		groundtruth_ok = 0;

	else
	{
		gt_oldpose = gt_pose;

		//							Update pose
		//-----------------------------------------------------------------
		const float incr_t0 = timestamp_obs - t0;
		const float incr_t1 = last_groundtruth - timestamp_obs;
		const float incr_t = incr_t0 + incr_t1;

		if (incr_t == 0.f) //Deal with defects in the groundtruth files
		{
			groundtruth_ok = 0;
			return;
		}

		//Sometimes the quaternion sign changes in the groundtruth
		if (abs(qx0 + last_gt_data[3]) + abs(qy0 + last_gt_data[4]) + abs(qz0 + last_gt_data[5]) + abs(w0 + last_gt_data[6]) < 0.05)
		{
			qx0 = -qx0; qy0 = -qy0; qz0 = -qz0; w0 = -w0;
		}

		double x,y,z,qx,qy,qz,w;
		x = (incr_t0*last_gt_data[0] + incr_t1*x0)/(incr_t);
		y = (incr_t0*last_gt_data[1] + incr_t1*y0)/(incr_t);
		z = (incr_t0*last_gt_data[2] + incr_t1*z0)/(incr_t);
		qx = (incr_t0*last_gt_data[3] + incr_t1*qx0)/(incr_t);
		qy = (incr_t0*last_gt_data[4] + incr_t1*qy0)/(incr_t);
		qz = (incr_t0*last_gt_data[5] + incr_t1*qz0)/(incr_t);
		w = (incr_t0*last_gt_data[6] + incr_t1*w0)/(incr_t);

		CMatrixDouble33 mat;
		mat(0,0) = 1- 2*qy*qy - 2*qz*qz;
		mat(0,1) = 2*(qx*qy - w*qz);
		mat(0,2) = 2*(qx*qz + w*qy);
		mat(1,0) = 2*(qx*qy + w*qz);
		mat(1,1) = 1 - 2*qx*qx - 2*qz*qz;
		mat(1,2) = 2*(qy*qz - w*qx);
		mat(2,0) = 2*(qx*qz - w*qy);
		mat(2,1) = 2*(qy*qz + w*qx);
		mat(2,2) = 1 - 2*qx*qx - 2*qy*qy;

		CPose3D gt, transf;
		gt.setFromValues(x,y,z,0,0,0);
		gt.setRotationMatrix(mat);
		transf.setFromValues(0,0,0,0.5*M_PI, -0.5*M_PI, 0);

		//Alternative - directly quaternions
		//vector<float> quat;
		//quat[0] = x, quat[1] = y; quat[2] = z;
		//quat[3] = w, quat[4] = qx; quat[5] = qy; quat[6] = qz;
		//gt.setFromXYZQ(quat);

		//Set the initial pose (if appropiate)
		if (first_pose == false)
		{
			cam_pose = gt + transf;
			first_pose = true;
		}

		gt_pose = gt + transf;
		groundtruth_ok = 1;
	}

	obs3D->unload();
	rawlog_count++;

	if (dataset.size() <= rawlog_count)
		dataset_finished = true;
}

void CDifodoDatasets::reset()
{
	loadFrame();
	if (fast_pyramid)	buildCoordinatesPyramidFast();
	else				buildCoordinatesPyramid();

	cam_oldpose = cam_pose;
	gt_oldpose = gt_pose;
}

void CDifodoDatasets::writeTrajectoryFile()
{	
	//Don't take into account those iterations with consecutive equal depth images
	if (abs(dt.sumAll()) > 0)
	{		
		mrpt::math::CQuaternionDouble quat;
		CPose3D auxpose, transf;
		transf.setFromValues(0,0,0,0.5*M_PI, -0.5*M_PI, 0);

		auxpose = cam_pose - transf;
		auxpose.getAsQuaternion(quat);
	
		char aux[24];
		sprintf(aux,"%.04f", timestamp_obs);
		f_res << aux << " ";
		f_res << cam_pose[0] << " ";
		f_res << cam_pose[1] << " ";
		f_res << cam_pose[2] << " ";
		f_res << quat(2) << " ";
		f_res << quat(3) << " ";
		f_res << -quat(1) << " ";
		f_res << -quat(0) << endl;
	}
}


