/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/vision/CDifodo.h>
#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CTicTac.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace std;
using namespace Eigen;

CDifodo::CDifodo()
{
	rows = 60;
	cols = 80;
	fovh = M_PI*57.5/180.0;
	fovv = M_PI*45.0/180.0;
	lens_disp = 0.022;
	cam_mode = 1;			// (1 - 640 x 480, 2 - 320 x 240, 4 - 160 x 120)
	downsample = 4;
	gaussian_mask_size = 7;
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

	f_dist = 1.0/525.0;																				//In meters
	x_incr = 2.0*f_dist*(floor(float(resh)/float(cols))*cols/float(resh))*tan(0.5*fovh)/(cols-1);	//In meters
	y_incr = 2.0*f_dist*(floor(float(resv)/float(rows))*rows/float(resv))*tan(0.5*fovv)/(rows-1);	//In meters
	fps = 30.0;																						//In Hz

	//Depth thresholds
	const int dy = floor(float(resv)/float(rows));
	const int dx = floor(float(resh)/float(cols));

	duv_threshold = 0.001*(dx + dy)*(cam_mode*downsample);
	dt_threshold = 0.2*fps;
	dif_threshold = 0.001*(dx + dy)*(cam_mode*downsample);
	difuv_surroundings = 0.005*(dx + dy)*(cam_mode*downsample);
	dift_surroundings = 0.01*fps*(dx + dy)*(cam_mode*downsample);

	previous_speed_const_weight = 0.2;
	previous_speed_eig_weight = 300.0;

	num_valid_points = 0;
}

void CDifodo::calculateCoord()
{
	for (unsigned int x = 0; x < cols; x++)
		for (unsigned int y = 0; y < rows; y++)
		{
			if ((depth(y,x)) == 0 || (depth_old(y,x) == 0))
			{
				depth_inter(y,x) = 0;
				xx_inter(y,x) = 0;
				yy_inter(y,x) = 0;
			}
			else
			{
				depth_inter(y,x) = 0.5*(depth(y,x) + depth_old(y,x));
				xx_inter(y,x) = 0.5*(xx(y,x) + xx_old(y,x));
				yy_inter(y,x) = 0.5*(yy(y,x) + yy_old(y,x));
			}
		}
}


void CDifodo::calculateDepthDerivatives()
{
	for (unsigned int x = 1; x < cols-1; x++)
		for (unsigned int y = 1; y < rows-1; y++)
		{
			du(y,x) = 0.5*(depth_inter(y,x+1) - depth_inter(y,x-1));
			dv(y,x) = 0.5*(depth_inter(y+1,x) - depth_inter(y-1,x));
			dt(y,x) = fps*(depth(y,x) - depth_old(y,x));
		}
}


void CDifodo::filterAndDownsample()
{
	//CTicTac clock;
	//clock.Tic();

	//Push the frames back
	depth_old = depth;
	xx_old = xx;
	yy_old = yy;

	//					Create the kernel
	//==========================================================
	Eigen::MatrixXf kernel(gaussian_mask_size,1);

    const float sigma = 0.2f*gaussian_mask_size;
    float r, s = 2.0f * sigma * sigma;
    float ksum = 0.0f;

    // Generate kernel
	if ((gaussian_mask_size%2 == 0)||(gaussian_mask_size<3))
	{
		cout << endl << "Mask size must be odd and bigger than 2";
		depth_ft = depth_wf;
		return;
	}

	const int lim_mask = (gaussian_mask_size-1)/2;
	for (int x = -lim_mask; x <= lim_mask; x++)
	{
		r = std::sqrt(float(x*x));
		kernel(x + lim_mask, 0) = (exp(-(r*r)/s))/(M_PI * s);
		ksum += kernel(x + lim_mask, 0);
	}

	// normalize the Kernel
	for (int x = -lim_mask; x <= lim_mask; x++)
	{
			kernel(x + lim_mask, 0)/=ksum;
			//cout << kernel(x + lim_mask, 1) << "  ";
	}

	const int width = depth_wf.getColCount();
	const int height = depth_wf.getRowCount();
	MatrixXf depth_if;
	depth_if.setSize(height, width);

	//Apply gaussian filter (separately)
	//rows
	for ( int i=0; i<height; i++)
		for ( int j=0; j<width; j++)
		{
			if ((j>=lim_mask)&&(j<width-lim_mask))
			{
				float sum = 0.0f;
				float ponder = 1.0f;
				for (int k=-lim_mask; k<=lim_mask; k++)
				{

					if (depth_wf(i,j+k) == 0)
						ponder -= kernel(k+lim_mask,0);
					else
						sum += kernel(k+lim_mask,0)*depth_wf(i,j+k);
				}
				if (ponder == 1.0f)
					depth_if(i,j) = sum;
				else if (ponder > 0.0001f)
					depth_if(i,j) = sum/ponder;
				else
					depth_if(i,j) = 0;
			}
			else
				depth_if(i,j) = depth_wf(i,j);

		}

	//cols
	for ( int i=0; i<height; i++)
		for ( int j=0; j<width; j++)
		{
			if ((i>=lim_mask)&&(i<height-lim_mask))
			{
				float sum = 0.0f;
				float ponder = 1.0f;

				for (int k=-lim_mask; k<=lim_mask; k++)
				{

					if (depth_if(i+k,j) == 0)
						ponder -= kernel(k+lim_mask,0);
					else
						sum += kernel(k+lim_mask,0)*depth_if(i+k,j);
				}

				if (ponder == 1.0f)
					depth_wf(i,j) = sum;
				else if (ponder > 0.0001f)
					depth_wf(i,j) = sum/ponder;
				else
					depth_wf(i,j) = 0;
			}
			else
				depth_wf(i,j) = depth_if(i,j);

		}

	//Downsample the pointcloud
	const float inv_f = float(640/width)/525.0f;
	const float disp_x = 0.5*(width-1);
	const float disp_y = 0.5*(height-1);

	const int dy = floor(float(height)/float(rows));
	const int dx = floor(float(width)/float(cols));
	const unsigned int iniy = (height-dy*rows)/2;
	const unsigned int inix = (width-dx*cols)/2;

	for (unsigned int y = 0; y < rows; y++)
		for (unsigned int x = 0; x < cols; x++)
		{
			depth(y,x) = depth_wf(iniy+y*dy,inix+x*dx);
			xx(y,x) = (inix+x*dx - disp_x)*depth_wf(iniy+y*dy,inix+x*dx)*inv_f + lens_disp;
			yy(y,x) = (iniy+y*dy - disp_y)*depth_wf(iniy+y*dy,inix+x*dx)*inv_f;
		}

	//cout << endl << "Execution time - filter + downsample (ms): " << 1000*clock.Tac();
}

void CDifodo::findBorders()
{
	border.assign(0);

	//Detect borders
	for (unsigned int x = 1; x < cols-1; x++)
		for (unsigned int y = 1; y < rows-1; y++)
		{
			if (null(y,x) == 0)
			{
				const float aver_duv = du(y,x)*du(y,x) + dv(y,x)*dv(y,x);
				const float ini_dx = 0.5*(depth_old(y,x+1) - depth_old(y,x-1));
				const float ini_dy = 0.5*(depth_old(y+1,x) - depth_old(y-1,x));
				const float final_dx = 0.5*(depth(y,x+1) - depth(y,x-1));
				const float final_dy = 0.5*(depth(y+1,x) - depth(y-1,x));


				//Derivative too high (the average derivative)
				if (aver_duv > duv_threshold)
					border(y,x) = 1;

				else if (abs(dt(y,x)) > dt_threshold)
					border(y,x) = 1;

				//Big difference between initial and final derivatives
				else if (abs(final_dx-ini_dx) + abs(final_dy-ini_dy) > dif_threshold)
					border(y,x) = 1;

				//Difference between derivatives in the surroundings
				else
				{
					float sum_duv = 0;
					float sum_dift = 0;
					float sum_difdepth = 0;
					for (int k = -1; k<2; k++)
						for (int l = -1; l<2; l++)
						{
							sum_duv += abs(du(y,x)-du(y+k,x+l)) + abs(dv(y,x)-dv(y+k,x+l));
							sum_dift += abs(dt(y,x) - dt(y+k,x+l));
							sum_difdepth += abs(depth_inter(y,x) - depth_inter(y+k,x+l));
						}

					if (sum_dift > depth_inter(y,x)*dift_surroundings)
						border(y,x) = 1;

					else if (sum_duv > (4.0*sum_difdepth + depth_inter(y,x))*difuv_surroundings)
						border(y,x) = 1;

				}
			}
		}

	//Delete sparse points
	for (unsigned int x = 1; x < cols-1; x++)
		for (unsigned int y = 1; y < rows-1; y++)
		{
			if ((null(y,x) == 0)&&(border(y,x) == 0))
			{
				float sum_alone = 0;
				for (int k = -1; k<2; k++)
					for (int l = -1; l<2; l++)
					{
						sum_alone += (border(y+k,x+l)||null(y+k,x+l));
					}

				if (sum_alone > 6)
					border(y,x) = 1;

			}
		}
}


void CDifodo::findNullPoints()
{
	null.assign(0);
	for (unsigned int x = 0; x < cols; x++)
		for (unsigned int y = 0; y < rows; y++)
			if (depth_inter(y,x) == 0)
				null(y,x) = 1;

}


void CDifodo::findValidPoints()
{
	num_valid_points = 0;

	for (unsigned int y = 1; y < rows-1; y++)
		for (unsigned int x = 1; x < cols-1; x++)
			if ((border(y,x) == 0)&&(null(y,x) == 0))
				num_valid_points++;
}


void CDifodo::solveDepthSystem()
{
	using mrpt::utils::square;

	utils::CTicTac	clock;
	unsigned int cont = 0;
	MatrixXf A;
	MatrixXf Var;
	MatrixXf B;
	A.resize(num_valid_points,6);
	B.setSize(num_valid_points,1);
	Var.setSize(6,1);

	//clock.Tic();

	//Fill the matrix A and the vector B
	//The order of the variables will be (vz, vx, vy, wz, wx, wy)
	//The points order will be (1,1), (1,2)...(1,cols-1), (2,1), (2,2)...(row-1,cols-1). Points at the borders are not included

	const float f_inv_x = f_dist/x_incr;
	const float f_inv_y = f_dist/y_incr;
	const float kz2 = square(1.425e-5/gaussian_mask_size);

	//We need to express the last camera velocity respect to the current camera reference frame
	math::CMatrixFloat61 kai_abs;
	math::CMatrixDouble33 inv_trans;
	math::CMatrixFloat31 v_old, w_old;
	cam_pose.getRotationMatrix(inv_trans);
	v_old = inv_trans.inverse().cast<float>()*kai_abs.topRows(3);
	w_old = inv_trans.inverse().cast<float>()*kai_abs.bottomRows(3);

	//Create the weighted least squares system
	for (unsigned int y = 1; y < rows-1; y++)
		for (unsigned int x = 1; x < cols-1; x++)
			if ((border(y,x) == 0)&&(null(y,x) == 0))
			{
				//Precomputed expressions
				const float inv_d = 1.0f/depth_inter(y,x);
				const float dxcomp = du(y,x)*f_inv_x*inv_d;
				const float dycomp = dv(y,x)*f_inv_y*inv_d;
				const float z2 = square(depth_inter(y,x));
				const float z4 = z2*z2;

				//Weights calculation
				const float var11 = kz2*z4;
				const float var12 = kz2*xx_inter(y,x)*z2*depth_inter(y,x);
				const float var13 = kz2*yy_inter(y,x)*z2*depth_inter(y,x);
				const float var22 = kz2*square(xx_inter(y,x))*z2;
				const float var23 = kz2*xx_inter(y,x)*yy_inter(y,x)*z2;
				const float var33 = kz2*square(yy_inter(y,x))*z2;
				const float var44 = kz2*z4*fps*fps;
				const float var55 = kz2*z4*0.25;
				const float var66 = kz2*z4*0.25;

				const float j1 = -2.0*inv_d*inv_d*(xx_inter(y,x)*dxcomp + yy_inter(y,x)*dycomp)*(v_old[0] + yy_inter(y,x)*w_old[1] - xx_inter(y,x)*w_old[2])
								+ inv_d*dxcomp*(v_old[1] - yy_inter(y,x)*w_old[0]) + inv_d*dycomp*(v_old[2] + xx_inter(y,x)*w_old[0]);

				const float j2 = inv_d*dxcomp*(v_old[0] + yy_inter(y,x)*w_old[1] - 2.0*xx_inter(y,x)*w_old[2]) - dycomp*w_old[0];

				const float j3 = inv_d*dycomp*(v_old[0] + 2*yy_inter(y,x)*w_old[1] - xx_inter(y,x)*w_old[2]) + dxcomp*w_old[0];

				const float j4 = 1;

				const float j5 =  xx_inter(y,x)*inv_d*inv_d*f_inv_y*(v_old[0] + yy_inter(y,x)*w_old[1] - xx_inter(y,x)*w_old[2])
							   + inv_d*f_inv_x*(-v_old[1] - depth_inter(y,x)*w_old[2] + yy_inter(y,x)*w_old[0]);

				const float j6 = yy_inter(y,x)*inv_d*inv_d*f_inv_y*(v_old[0] + yy_inter(y,x)*w_old[1] - xx_inter(y,x)*w_old[2])
							   + inv_d*f_inv_y*(-v_old[2] + depth_inter(y,x)*w_old[1] - xx_inter(y,x)*w_old[0]);

				weights(y,x) = sqrt(1.0/(j1*(j1*var11+j2*var12+j3*var13) + j2*(j1*var12+j2*var22+j3*var23)
										+j3*(j1*var13+j2*var23+j3*var33) + j4*j4*var44 + j5*j5*var55 + j6*j6*var66));

				A(cont,0) = weights(y,x)*(1.0f + dxcomp*xx_inter(y,x)*inv_d + dycomp*yy_inter(y,x)*inv_d);
				A(cont,1) = weights(y,x)*(-dxcomp);
				A(cont,2) = weights(y,x)*(-dycomp);
				A(cont,3) = weights(y,x)*(dxcomp*yy_inter(y,x) - dycomp*xx_inter(y,x));
				A(cont,4) = weights(y,x)*(yy_inter(y,x) + dxcomp*inv_d*yy_inter(y,x)*xx_inter(y,x) + dycomp*(yy_inter(y,x)*yy_inter(y,x)*inv_d + depth_inter(y,x)));
				A(cont,5) = weights(y,x)*(-xx_inter(y,x) - dxcomp*(xx_inter(y,x)*xx_inter(y,x)*inv_d + depth_inter(y,x)) - dycomp*inv_d*yy_inter(y,x)*xx_inter(y,x));

				B(cont,0) = weights(y,x)*(-dt(y,x));

				cont++;
			}

	//Solve the linear system of equations using a minimum least squares method
	MatrixXf atrans = A.transpose();
	MatrixXf a_ls = atrans*A;
	Var = a_ls.ldlt().solve(atrans*B);
	kai_solver = Var;

	//Covariance matrix calculation
	MatrixXf residuals(num_valid_points,1);
	residuals = A*Var - B;
	est_cov = (1.0f/float(num_valid_points-6))*a_ls.inverse()*residuals.squaredNorm();

}


void CDifodo::OdometryCalculation()
{
	utils::CTicTac clock;
	clock.Tic();
	filterAndDownsample();
	calculateCoord();
	calculateDepthDerivatives();
	findNullPoints();
	findBorders();
	findValidPoints();

	if (num_valid_points > 6)	{solveDepthSystem();}
	else						{kai_solver.assign(0);}

	execution_time = 1000*clock.Tac();
}


void CDifodo::filterSpeedAndPoseUpdate()
{
	//-------------------------------------------------------------------------
	//								Filter speed
	//-------------------------------------------------------------------------

	utils::CTicTac clock;
	clock.Tic();

	//				Calculate Eigenvalues and Eigenvectors
	//----------------------------------------------------------------------------
	SelfAdjointEigenSolver<MatrixXf> eigensolver(est_cov);
	if (eigensolver.info() != Success)
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
	MatrixXf kai_b_fil;
	kai_b_fil.setSize(6,1);
	for (unsigned int i=0; i<6; i++)
	{
		kai_b_fil(i,0) = (kai_b(i,0) + (previous_speed_eig_weight*eigensolver.eigenvalues()(i,0) + previous_speed_const_weight)*kai_b_old(i,0))/(1.0 + previous_speed_eig_weight*eigensolver.eigenvalues()(i,0) + previous_speed_const_weight);
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

	double yaw,pitch,roll;
	math::CMatrixDouble31 w_euler_d;

	cam_pose.getYawPitchRoll(yaw,pitch,roll);
	w_euler_d(0,0) = kai_loc_fil(4,0)*sin(roll)/cos(pitch) + kai_loc_fil(5,0)*cos(roll)/cos(pitch);
	w_euler_d(1,0) = kai_loc_fil(4,0)*cos(roll) - kai_loc_fil(5,0)*sin(roll);
	w_euler_d(2,0) = kai_loc_fil(3,0) + kai_loc_fil(4,0)*sin(roll)*tan(pitch) + kai_loc_fil(5,0)*cos(roll)*tan(pitch);

	//Update pose
	cam_pose.x_incr(v_abs_fil(0,0)/fps);
	cam_pose.y_incr(v_abs_fil(1,0)/fps);
	cam_pose.y_incr(v_abs_fil(2,0)/fps);
	cam_pose.setYawPitchRoll(yaw + w_euler_d(0,0)/fps, pitch + w_euler_d(1,0)/fps, roll + w_euler_d(2,0)/fps);

	execution_time += 1000*clock.Tac();
}


void CDifodo::setCameraFocalLenght(float new_f)
{
	const unsigned int resh = 640/(cam_mode*downsample);
	const unsigned int resv = 480/(cam_mode*downsample);

	f_dist = new_f;
	x_incr = 2.0*f_dist*(floor(float(resh)/float(cols))*cols/float(resh))*tan(0.5*fovh)/(cols-1);
	y_incr = 2.0*f_dist*(floor(float(resv)/float(rows))*rows/float(resv))*tan(0.5*fovv)/(rows-1);
}


void CDifodo::setNumberOfRowsAndCols(unsigned int num_rows, unsigned int num_cols)
{
	rows = num_rows;
	cols = num_cols;

	const unsigned int resh = 640/(cam_mode*downsample);
	const unsigned int resv = 480/(cam_mode*downsample);

	depth.setSize(rows,cols);
	depth_old.setSize(rows,cols);
	depth_inter.setSize(rows,cols);

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

	x_incr = 2.0*f_dist*(floor(float(resh)/float(cols))*cols/float(resh))*tan(0.5*fovh)/(cols-1);	//In meters
	y_incr = 2.0*f_dist*(floor(float(resv)/float(rows))*rows/float(resv))*tan(0.5*fovv)/(rows-1);	//In meters																				//In Hz

	//Depth thresholds
	const int dy = floor(float(resv)/float(rows));
	const int dx = floor(float(resh)/float(cols));

	duv_threshold = 0.001*(dx + dy)*(cam_mode*downsample);
	dt_threshold = 0.2*fps;
	dif_threshold = 0.001*(dx + dy)*(cam_mode*downsample);
	difuv_surroundings = 0.005*(dx + dy)*(cam_mode*downsample);
	dift_surroundings = 0.01*fps*(dx + dy)*(cam_mode*downsample);
}


void CDifodo::setFOV(float new_fovh, float new_fovv)
{
	fovh = M_PI*new_fovh/180.0;
	fovv = M_PI*new_fovv/180.0;
	const unsigned int resh = 640/(cam_mode*downsample);
	const unsigned int resv = 480/(cam_mode*downsample);

	x_incr = 2.0*f_dist*(floor(float(resh)/float(cols))*cols/float(resh))*tan(0.5*fovh)/(cols-1);	//In meters
	y_incr = 2.0*f_dist*(floor(float(resv)/float(rows))*rows/float(resv))*tan(0.5*fovv)/(rows-1);	//In meters
}


void CDifodo::getPointsCoord(MatrixXf &x, MatrixXf &y, MatrixXf &z)
{
	x.resize(rows,cols);
	y.resize(rows,cols);
	z.resize(rows,cols);

	z = depth_inter;
	x = xx_inter;
	y = yy_inter;
}


void CDifodo::getDepthDerivatives(MatrixXf &cur_du, MatrixXf &cur_dv, MatrixXf &cur_dt)
{
	cur_du.resize(rows,cols);
	cur_dv.resize(rows,cols);
	cur_dt.resize(rows,cols);

	cur_du = du;
	cur_dv = dv;
	cur_dt = dt;
}


void CDifodo::getWeights(MatrixXf &w)
{
	w.resize(rows,cols);
	w = weights;
}


void CDifodo::bordersThresholdToDefault()
{
	const int dy = floor(float(480/(cam_mode*downsample))/float(rows));
	const int dx = floor(float(640/(cam_mode*downsample))/float(cols));

	duv_threshold = 0.001*(dx + dy)*(cam_mode*downsample);
	dt_threshold = 0.2*fps;
	dif_threshold = 0.001*(dx + dy)*(cam_mode*downsample);
	difuv_surroundings = 0.005*(dx + dy)*(cam_mode*downsample);
	dift_surroundings = 0.01*fps*(dx + dy)*(cam_mode*downsample);
}

