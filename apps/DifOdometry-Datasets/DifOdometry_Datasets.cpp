/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "DifOdometry_Datasets.h"
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
#include "legend.xpm"

using namespace Eigen;
using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::slam;


void CDifodoDatasets::loadConfiguration(const utils::CConfigFileBase &ini )
{
	downsample = ini.read_int("DIFODO_CONFIG", "downsample", 4, true);
	rows = ini.read_int("DIFODO_CONFIG", "rows", 50, true);
	cols = ini.read_int("DIFODO_CONFIG", "cols", 60, true);

	string filename = ini.read_string("DIFODO_CONFIG", "filename", "no file", true);


	//						Open Rawlog File
	//==================================================================

	if (!dataset.loadFromRawLogFile(filename))
		throw std::runtime_error("\nCouldn't open rawlog dataset file for input...");

	rawlog_count = 0;

	//filename.replace(filename.find(".rawlog"),7,"_Images\\");
	const string imgsPath = CRawlog::detectImagesDirectory(filename);

	// Set external images directory:
	CImage::IMAGES_PATH_BASE = imgsPath;

	//					Load ground_truth
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
	weights.setSize(rows,cols);
	weights.assign(0);
	est_cov.assign(0);

	x_incr = 2.0*f_dist*(floor(float(resh)/float(cols))*cols/float(resh))*tan(0.5*fovh)/(cols-1);	//In meters
	y_incr = 2.0*f_dist*(floor(float(resv)/float(rows))*rows/float(resv))*tan(0.5*fovv)/(rows-1);	//In meters																				//In Hz

	//Depth thresholds
	const int dz = floor(float(resv)/float(rows));
	const int dy = floor(float(resh)/float(cols));

	duv_threshold = 0.001*(dz + dy)*(cam_mode*downsample);
	dt_threshold = 0.2*fps;
	dif_threshold = 0.001*(dz + dy)*(cam_mode*downsample);
	difuv_surroundings = 0.005*(dz + dy)*(cam_mode*downsample);
	dift_surroundings = 0.01*fps*(dz + dy)*(cam_mode*downsample);

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

	//Groundtruth camera
	CBoxPtr camera_gt = CBox::Create(math::TPoint3D(-0.02,-0.1,-0.01),math::TPoint3D(0.02,0.1,0.01));
	camera_gt->setPose(gt_pose);
	camera_gt->setColor(1,0,0);
	scene->insert( camera_gt );

	//Frustum
	opengl::CFrustumPtr FOV = opengl::CFrustum::Create(0.3, 5, 57.3*fovh, 57.3*fovv, 1.5f, true, false);
	FOV->setPose(gt_pose);
	scene->insert( FOV );

	//Reference gt
	CSetOfObjectsPtr reference_gt = stock_objects::CornerXYZ();
	reference_gt->setScale(0.2);
	reference_gt->setPose(gt_pose);
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

	//Groundtruth
	CSetOfLinesPtr traj_lines_gt = CSetOfLines::Create();
	traj_lines_gt->setLocation(0,0,0);
	traj_lines_gt->setColor(0,0,0);
	traj_lines_gt->setLineWidth(3);
	scene->insert( traj_lines_gt );
	CPointCloudPtr traj_points_gt = CPointCloud::Create();
	traj_points_gt->setColor(0.5,0,0);
	traj_points_gt->setPointSize(4);
	traj_points_gt->enablePointSmooth(1);
	scene->insert( traj_points_gt );

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

void CDifodoDatasets::updateScene()
{
	scene = window.get3DSceneAndLock();

	//Reference gt
	CSetOfObjectsPtr reference_gt = scene->getByClass<CSetOfObjects>(1);
	reference_gt->setPose(gt_pose);

	//Camera points
	CPointCloudPtr cam_points = scene->getByClass<CPointCloud>(0);
	cam_points->clear();
	cam_points->setPose(gt_pose);
	for (unsigned int y=0; y<cols; y++)
		for (unsigned int z=0; z<rows; z++)
		{
			cam_points->insertPoint(depth_inter(z,y), xx_inter(z,y), yy_inter(z,y));
		}

	//Border points
	CPointCloudPtr border_points = scene->getByClass<CPointCloud>(1);
	border_points->clear();
	border_points->setPose(gt_pose);
	for (unsigned int y=0; y<cols; y++)
		for (unsigned int z=0; z<rows; z++)
			if (border(z,y) == 1)
				border_points->insertPoint(depth_inter(z,y), xx_inter(z,y), yy_inter(z,y));

	//DifOdo camera
	CBoxPtr camera_odo = scene->getByClass<CBox>(0);
	camera_odo->setPose(cam_pose);

	//Groundtruth camera
	CBoxPtr camera_gt = scene->getByClass<CBox>(1);
	camera_gt->setPose(gt_pose);

	//Frustum
	CFrustumPtr FOV = scene->getByClass<CFrustum>(0);
	FOV->setPose(gt_pose);

	if ((first_pose == true)&&((gt_pose-gt_oldpose).norm() < 0.5))
	{
		//Difodo traj lines
		CSetOfLinesPtr traj_lines_odo = scene->getByClass<CSetOfLines>(0);
		traj_lines_odo->appendLine(cam_oldpose.x(), cam_oldpose.y(), cam_oldpose.z(), cam_pose.x(), cam_pose.y(), cam_pose.z());

		//Difodo traj points
		CPointCloudPtr traj_points_odo = scene->getByClass<CPointCloud>(2);
		traj_points_odo->insertPoint(cam_pose.x(), cam_pose.y(), cam_pose.z());

		//Groundtruth traj lines
		CSetOfLinesPtr traj_lines_gt = scene->getByClass<CSetOfLines>(1);
		traj_lines_gt->appendLine(gt_oldpose.x(), gt_oldpose.y(), gt_oldpose.z(), gt_pose.x(), gt_pose.y(), gt_pose.z());

		//Groundtruth traj points
		CPointCloudPtr traj_points_gt = scene->getByClass<CPointCloud>(3);
		traj_points_gt->insertPoint(gt_pose.x(), gt_pose.y(), gt_pose.z());
	}

	//Ellipsoid showing covariance
	math::CMatrixFloat33 cov3d(est_cov.topLeftCorner(3,3));
	CEllipsoidPtr ellip = scene->getByClass<CEllipsoid>(0);
	ellip->setCovMatrix(cov3d);
	ellip->setPose(cam_pose);

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
			return;
		alfa = dataset.getAsObservation(rawlog_count);
	}

	CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(alfa);
	obs3D->load();

	const unsigned int height = obs3D->rangeImage.getRowCount();
	const unsigned int width = obs3D->rangeImage.getColCount();
	const unsigned int index_incr = downsample;

	//Load depth image
	for (unsigned int i=0; i<height; i+=index_incr)
		for (unsigned int j=0; j<width; j+=index_incr)
			depth_wf(i/index_incr,j/index_incr) = obs3D->rangeImage(height-i-1, width-j-1);


	double timestamp_gt;
	double timestamp_obs = mrpt::system::timestampTotime_t(obs3D->timestamp);

	//Exit if there is no ground truth at this time
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
	last_groundtruth = timestamp_gt;

	//last_gt_data = [x y z qx qy qz w]
	f_gt >> last_gt_data[0]; f_gt >> last_gt_data[1]; f_gt >> last_gt_data[2];
	f_gt >> last_gt_data[3]; f_gt >> last_gt_data[4]; f_gt >> last_gt_data[5]; f_gt >> last_gt_data[6];

	if (last_groundtruth - timestamp_obs > 0.01)
	{
		groundtruth_ok = 0;
	}
	else
	{
		gt_oldpose = gt_pose;

		//Update pose
		const float incr_t0 = timestamp_obs - t0;
		const float incr_t1 = last_groundtruth - timestamp_obs;
		const float incr_t = incr_t0 + incr_t1;

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

		//Alternative - directly quaternions
		//vector<float> quat;
		//quat[0] = x, quat[1] = y; quat[2] = z;
		//quat[3] = w, quat[4] = qx; quat[5] = qy; quat[6] = qz;
		//gt.setFromXYZQ(quat);

		gt.setFromValues(x,y,z,0,0,0);
		gt.setRotationMatrix(mat);
		transf.setFromValues(0,0,0,0.5*M_PI, -0.5*M_PI, 0);

		//Set the initial pose (if appropiate)
		if (first_pose == false)
		{
			cam_pose = gt + transf;
			first_pose = true;
		}

		gt_pose = gt + transf;
		groundtruth_ok = 1;

	//printf("\n (q0,q0,q0,t0) = (%f, %f, %f, %.04f)", qx0, qy0, qz0, t0);
	//printf("\n (q1,q1,q1,t1) = (%f, %f, %f, %.04f)", qx1, qy1, qz1, t1);
	//printf("\n (x,y,z,t) = (%f, %f, %f, %.04f) \n", x, y, z, timestampTotime_t(obs3D->timestamp));

	}

	obs3D->unload();
	rawlog_count++;
}


void CDifodoDatasets::reset()
{
	loadFrame();
	filterAndDownsample();
	calculateCoord();
	calculateDepthDerivatives();
	findNullPoints();
	findBorders();
	findValidPoints();

	cam_oldpose = cam_pose;
	gt_oldpose = gt_pose;
}


void CDifodoDatasets::filterSpeedAndPoseUpdate()
{
	//-------------------------------------------------------------------------
	//								Filter speed
	//-------------------------------------------------------------------------

	utils::CTicTac clock;
	clock.Tic();

	//		Calculate Eigenvalues and Eigenvectors
	//----------------------------------------------------------
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(est_cov);
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

	//==================================================================================
	//									Statistics
	//==================================================================================

	if ((groundtruth_ok)&&(last_groundtruth_ok))
	{
		CPose3D gt_pose_incr = gt_pose - gt_oldpose;
		CPose3D cam_pose_incr = cam_pose - cam_oldpose;
		CPose3D abs_pose_error = gt_pose - cam_pose;

		//cout << endl << "GT: " << gt_pose_incr;
		//cout << endl << "OD: " << cam_pose_incr;
		//cout << endl << "AE: " << abs_pose_error;

		//-------------------------------------------------------------------------
		//									DIF ODO
		//-------------------------------------------------------------------------

		//Relative errors in displacement
		for (unsigned int i=0; i<6; i++)
			rel_error[i] = cam_pose_incr[i] - gt_pose_incr[i];

		//Absolute error
		abs_error_tras = sqrt(abs_pose_error[0]*abs_pose_error[0] + abs_pose_error[1]*abs_pose_error[1] + abs_pose_error[2]*abs_pose_error[2]);
		abs_error_rot = sqrt(abs_pose_error[3]*abs_pose_error[3] + abs_pose_error[4]*abs_pose_error[4] + abs_pose_error[5]*abs_pose_error[5]);

		acu_rel_error_tras += sqrt(rel_error[0]*rel_error[0] + rel_error[1]*rel_error[1] + rel_error[2]*rel_error[2]);
		acu_rel_error_rot += sqrt(rel_error[3]*rel_error[3] + rel_error[4]*rel_error[4] + rel_error[5]*rel_error[5]);

		////Relative errors in velocity (m/s)^2
		//se_vel_dif = square(kai_loc_fil_d(0,0)-des_rel_true(0,0)*m_process.t_incr_inv)
		//			+ square(kai_loc_fil_d(1,0)-des_rel_true(1,0)*m_process.t_incr_inv)
		//			+ square(kai_loc_fil_d(2,0)-des_rel_true(2,0)*m_process.t_incr_inv);


		sum_exec_time += execution_time;

		//Don't take into account those iterations with the same depth images
		if (dt.sumAll() == 0)
			dtzero_before = 1;

		else if (dtzero_before == 1)
			dtzero_before = 0;

		else if (save_results == 1)
			writeToLogFile();

	}

	num_iter++;
}


void CDifodoDatasets::writeToLogFile()
{
	char aux[24];
	sprintf(aux,"%.04f", last_groundtruth);
	f_res << aux << " ";

	f_res << rel_error[0] << " ";
	f_res << rel_error[1] << " ";
	f_res << rel_error[2] << " ";
	f_res << rel_error[3] << " ";
	f_res << rel_error[4] << " ";
	f_res << rel_error[5] << " ";

	f_res << abs_error_tras << " ";
	f_res << abs_error_rot << " ";
	f_res << execution_time << " ";
	f_res << num_valid_points << " ";
	f_res << "\n";
}


void CDifodoDatasets::showStatistics()
{
	printf("\n==============================================================================");
	printf("\n Average execution time (ms): %f", sum_exec_time/num_iter);
	printf("\n Average frame to frame traslational error (m): %f", acu_rel_error_tras/num_iter);
	printf("\n Average frame to frame rotational error (deg): %f", 57.3*acu_rel_error_rot/num_iter);
	printf("\n Absolute traslational error (m): %f", abs_error_tras);
	printf("\n Absolute rotational error (deg): %f", 57.3*abs_error_rot);
	printf("\n==============================================================================\n");
}


