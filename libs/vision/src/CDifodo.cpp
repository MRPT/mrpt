/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/vision/CDifodo.h>
#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/round.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::math;
using namespace std;
using namespace Eigen;
using mrpt::utils::round;
using mrpt::utils::square;

CDifodo::CDifodo()
{
	rows = 60;
	cols = 80;
	fovh = M_PIf*58.6f/180.0f;
	fovv = M_PIf*45.6f/180.0f;
	cam_mode = 1;			// (1 - 640 x 480, 2 - 320 x 240, 4 - 160 x 120)
	downsample = 1;
	ctf_levels = 1;
	width = 640/(cam_mode*downsample);
	height = 480/(cam_mode*downsample);
	fast_pyramid = true;

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

	depth_wf.setSize(height,width);

	fps = 30.f;		//In Hz

	previous_speed_const_weight = 0.05f;
	previous_speed_eig_weight = 0.5f;
	kai_loc_old.assign(0.f);
	num_valid_points = 0;

	//Compute gaussian mask
	VectorXf v_mask(4);
	v_mask(0) = 1.f; v_mask(1) = 2.f; v_mask(2) = 2.f; v_mask(3) = 1.f;
	for (unsigned int i = 0; i<4; i++)
		for (unsigned int j = 0; j<4; j++)
			f_mask(i,j) = v_mask(i)*v_mask(j)/36.f;

	//Compute gaussian mask
	float v_mask2[5] = {1,4,6,4,1};
	for (unsigned int i = 0; i<5; i++)
		for (unsigned int j = 0; j<5; j++)
			g_mask[i][j] = v_mask2[i]*v_mask2[j]/256.f;
}

void CDifodo::buildCoordinatesPyramid()
{
	const float max_depth_dif = 0.1f;

	//Push coordinates back
	depth_old.swap(depth);
	xx_old.swap(xx);
	yy_old.swap(yy);

	//The number of levels of the pyramid does not match the number of levels used
	//in the odometry computation (because we might want to finish with lower resolutions)

	unsigned int pyr_levels = round(log(float(width/cols))/log(2.f)) + ctf_levels;

	//Generate levels
	for (unsigned int i = 0; i<pyr_levels; i++)
	{
		unsigned int s = pow(2.f,int(i));
		cols_i = width/s;
		rows_i = height/s;
		const int rows_i2 = 2*rows_i;
		const int cols_i2 = 2*cols_i;
		const int i_1 = i-1;

		if (i == 0)
			depth[i].swap(depth_wf);

		//                              Downsampling
		//-----------------------------------------------------------------------------
		else
		{
			for (unsigned int u = 0; u < cols_i; u++)
			for (unsigned int v = 0; v < rows_i; v++)
			{
				const int u2 = 2*u;
				const int v2 = 2*v;
				const float dcenter = depth[i_1](v2,u2);

				//Inner pixels
				if ((v>0)&&(v<rows_i-1)&&(u>0)&&(u<cols_i-1))
				{
					if (dcenter > 0.f)
					{
						float sum = 0.f;
						float weight = 0.f;

						for (int l = -2; l<3; l++)
						for (int k = -2; k<3; k++)
						{
							const float abs_dif = abs(depth[i_1](v2+k,u2+l)-dcenter);
							if (abs_dif < max_depth_dif)
							{
								const float aux_w = g_mask[2+k][2+l]*(max_depth_dif - abs_dif);
								weight += aux_w;
								sum += aux_w*depth[i_1](v2+k,u2+l);
							}
						}
						depth[i](v,u) = sum/weight;
					}
					else
					{
						float min_depth = 10.f;
						for (int l = -2; l<3; l++)
						for (int k = -2; k<3; k++)
						{
							const float d = depth[i_1](v2+k,u2+l);
							if ((d > 0.f)&&(d < min_depth))
								min_depth = d;
						}

						if (min_depth < 10.f)
							depth[i](v,u) = min_depth;
						else
							depth[i](v,u) = 0.f;
					}
				}

				//Boundary
				else
				{
					if (dcenter > 0.f)
					{
						float sum = 0.f;
						float weight = 0.f;

						for (int l = -2; l<3; l++)
						for (int k = -2; k<3; k++)
						{
							const int indv = v2+k,indu = u2+l;
							if ((indv>=0)&&(indv<rows_i2)&&(indu>=0)&&(indu<cols_i2))
							{
								const float abs_dif = abs(depth[i_1](indv,indu)-dcenter);
								if (abs_dif < max_depth_dif)
								{
									const float aux_w = g_mask[2+k][2+l]*(max_depth_dif - abs_dif);
									weight += aux_w;
									sum += aux_w*depth[i_1](indv,indu);
								}
							}
						}
						depth[i](v,u) = sum/weight;
					}
					else
					{
						float min_depth = 10.f;
						for (int l = -2; l<3; l++)
						for (int k = -2; k<3; k++)
						{
							const int indv = v2+k,indu = u2+l;
							if ((indv>=0)&&(indv<rows_i2)&&(indu>=0)&&(indu<cols_i2))
							{
								const float d = depth[i_1](indv,indu);
								if ((d > 0.f)&&(d < min_depth))
									min_depth = d;
							}
						}

						if (min_depth < 10.f)
							depth[i](v,u) = min_depth;
						else
							depth[i](v,u) = 0.f;
					}
				}
			}
		}

		//Calculate coordinates "xy" of the points
		const float inv_f_i = 2.f*tan(0.5f*fovh)/float(cols_i);
		const float disp_u_i = 0.5f*(cols_i-1);
		const float disp_v_i = 0.5f*(rows_i-1);

		for (unsigned int u = 0; u < cols_i; u++)
		for (unsigned int v = 0; v < rows_i; v++)
		if (depth[i](v,u) > 0.f)
		{
			xx[i](v,u) = (u - disp_u_i)*depth[i](v,u)*inv_f_i;
			yy[i](v,u) = (v - disp_v_i)*depth[i](v,u)*inv_f_i;
		}
		else
		{
			xx[i](v,u) = 0.f;
			yy[i](v,u) = 0.f;
		}
	}
}

void CDifodo::buildCoordinatesPyramidFast()
{
	const float max_depth_dif = 0.1f;
	
	//Push coordinates back
	depth_old.swap(depth);
	xx_old.swap(xx);
	yy_old.swap(yy);

	//The number of levels of the pyramid does not match the number of levels used
	//in the odometry computation (because we might want to finish with lower resolutions)

	unsigned int pyr_levels = round(log(float(width/cols))/log(2.f)) + ctf_levels;

	//Generate levels
	for (unsigned int i = 0; i<pyr_levels; i++)
	{
		unsigned int s = pow(2.f,int(i));
		cols_i = width/s;
		rows_i = height/s;
		//const int rows_i2 = 2*rows_i;
		//const int cols_i2 = 2*cols_i;
		const int i_1 = i-1;

		if (i == 0)
			depth[i].swap(depth_wf);

		//                              Downsampling
		//-----------------------------------------------------------------------------
		else
		{
			for (unsigned int u = 0; u < cols_i; u++)
				for (unsigned int v = 0; v < rows_i; v++)
				{
					const int u2 = 2*u;
					const int v2 = 2*v;
					
					//Inner pixels
					if ((v>0)&&(v<rows_i-1)&&(u>0)&&(u<cols_i-1))
					{
						const Matrix4f d_block = depth[i_1].block<4,4>(v2-1,u2-1);
						float depths[4] = {d_block(5),d_block(6),d_block(9),d_block(10)};
						float dcenter;

						//Sort the array (try to find a good/representative value)
						for (signed char k = 2; k>=0; k--)
						if (depths[k+1] < depths[k])
							std::swap(depths[k+1],depths[k]);
						for (unsigned char k = 1; k<3; k++)
						if (depths[k] > depths[k+1])
							std::swap(depths[k+1],depths[k]);
						if (depths[2] < depths[1])
							dcenter = depths[1];
						else
							dcenter = depths[2];
						
						if (dcenter > 0.f)
						{	
							float sum = 0.f;
							float weight = 0.f;

							for (unsigned char k = 0; k<16; k++)
							{
								const float abs_dif = abs(d_block(k) - dcenter);
								if (abs_dif < max_depth_dif)
								{
									const float aux_w = f_mask(k)*(max_depth_dif - abs_dif);
									weight += aux_w;
									sum += aux_w*d_block(k);
								}
							}
							depth[i](v,u) = sum/weight;
						}
						else
							depth[i](v,u) = 0.f;

                    }

                    //Boundary
					else
					{
						const Matrix2f d_block = depth[i_1].block<2,2>(v2,u2);
						const float new_d = 0.25f*d_block.sumAll();
						if (new_d < 0.4f)
							depth[i](v,u) = 0.f;
						else
							depth[i](v,u) = new_d;
					}
				}
        }

        //Calculate coordinates "xy" of the points
		const float inv_f_i = 2.f*tan(0.5f*fovh)/float(cols_i);
        const float disp_u_i = 0.5f*(cols_i-1);
        const float disp_v_i = 0.5f*(rows_i-1);

        for (unsigned int u = 0; u < cols_i; u++) 
			for (unsigned int v = 0; v < rows_i; v++)
                if (depth[i](v,u) > 0.f)
				{
					xx[i](v,u) = (u - disp_u_i)*depth[i](v,u)*inv_f_i;
					yy[i](v,u) = (v - disp_v_i)*depth[i](v,u)*inv_f_i;
				}
				else
				{
					xx[i](v,u) = 0.f;
					yy[i](v,u) = 0.f;
				}
    }
}

void CDifodo::performWarping()
{
	//Camera parameters (which also depend on the level resolution)
	const float f = float(cols_i)/(2.f*tan(0.5f*fovh));
	const float disp_u_i = 0.5f*float(cols_i-1);
    const float disp_v_i = 0.5f*float(rows_i-1);

	//Rigid transformation estimated up to the present level
	Matrix4f acu_trans; 
	acu_trans.setIdentity();
	for (unsigned int i=1; i<=level; i++)
		acu_trans = transformations[i-1]*acu_trans;

	MatrixXf wacu(rows_i,cols_i);
	wacu.assign(0.f);
	depth_warped[image_level].assign(0.f);

	const float cols_lim = float(cols_i-1);
	const float rows_lim = float(rows_i-1);

	//						Warping loop
	//---------------------------------------------------------
	for (unsigned int j = 0; j<cols_i; j++)
		for (unsigned int i = 0; i<rows_i; i++)
		{		
			const float z = depth[image_level](i,j);
			
			if (z > 0.f)
			{
				//Transform point to the warped reference frame
				const float depth_w = acu_trans(0,0)*z + acu_trans(0,1)*xx[image_level](i,j) + acu_trans(0,2)*yy[image_level](i,j) + acu_trans(0,3);
				const float x_w = acu_trans(1,0)*z + acu_trans(1,1)*xx[image_level](i,j) + acu_trans(1,2)*yy[image_level](i,j) + acu_trans(1,3);
				const float y_w = acu_trans(2,0)*z + acu_trans(2,1)*xx[image_level](i,j) + acu_trans(2,2)*yy[image_level](i,j) + acu_trans(2,3);

				//Calculate warping
				const float uwarp = f*x_w/depth_w + disp_u_i;
				const float vwarp = f*y_w/depth_w + disp_v_i;

				//The warped pixel (which is not integer in general) contributes to all the surrounding ones
				if (( uwarp >= 0.f)&&( uwarp < cols_lim)&&( vwarp >= 0.f)&&( vwarp < rows_lim))
				{
					const int uwarp_l = uwarp;
					const int uwarp_r = uwarp_l + 1;
					const int vwarp_d = vwarp;
					const int vwarp_u = vwarp_d + 1;
					const float delta_r = float(uwarp_r) - uwarp;
					const float delta_l = uwarp - float(uwarp_l);
					const float delta_u = float(vwarp_u) - vwarp;
					const float delta_d = vwarp - float(vwarp_d);

					//Warped pixel very close to an integer value
					if (abs(round(uwarp) - uwarp) + abs(round(vwarp) - vwarp) < 0.05f)
					{
						depth_warped[image_level](round(vwarp), round(uwarp)) += depth_w;
						wacu(round(vwarp), round(uwarp)) += 1.f;
					}
					else
					{
						const float w_ur = square(delta_l) + square(delta_d);
						depth_warped[image_level](vwarp_u,uwarp_r) += w_ur*depth_w;
						wacu(vwarp_u,uwarp_r) += w_ur;

						const float w_ul = square(delta_r) + square(delta_d);
						depth_warped[image_level](vwarp_u,uwarp_l) += w_ul*depth_w;
						wacu(vwarp_u,uwarp_l) += w_ul;

						const float w_dr = square(delta_l) + square(delta_u);
						depth_warped[image_level](vwarp_d,uwarp_r) += w_dr*depth_w;
						wacu(vwarp_d,uwarp_r) += w_dr;

						const float w_dl = square(delta_r) + square(delta_u);
						depth_warped[image_level](vwarp_d,uwarp_l) += w_dl*depth_w;
						wacu(vwarp_d,uwarp_l) += w_dl;
					}
				}
			}
		}

	//Scale the averaged depth and compute spatial coordinates
    const float inv_f_i = 1.f/f;
	for (unsigned int u = 0; u<cols_i; u++)
		for (unsigned int v = 0; v<rows_i; v++)
		{	
			if (wacu(v,u) > 0.f)
			{
				depth_warped[image_level](v,u) /= wacu(v,u);
				xx_warped[image_level](v,u) = (u - disp_u_i)*depth_warped[image_level](v,u)*inv_f_i;
				yy_warped[image_level](v,u) = (v - disp_v_i)*depth_warped[image_level](v,u)*inv_f_i;
			}
			else
			{
				depth_warped[image_level](v,u) = 0.f;
				xx_warped[image_level](v,u) = 0.f;
				yy_warped[image_level](v,u) = 0.f;
			}
		}
}

void CDifodo::calculateCoord()
{	
	null.resize(rows_i, cols_i);
	null.assign(false);
	num_valid_points = 0;
	
	for (unsigned int u = 0; u < cols_i; u++)
		for (unsigned int v = 0; v < rows_i; v++)
		{
			if ((depth_old[image_level](v,u)) == 0.f || (depth_warped[image_level](v,u) == 0.f))
			{
				depth_inter[image_level](v,u) = 0.f;
				xx_inter[image_level](v,u) = 0.f;
				yy_inter[image_level](v,u) = 0.f;
				null(v, u) = true;
			}
			else
			{
				depth_inter[image_level](v,u) = 0.5f*(depth_old[image_level](v,u) + depth_warped[image_level](v,u));
				xx_inter[image_level](v,u) = 0.5f*(xx_old[image_level](v,u) + xx_warped[image_level](v,u));
				yy_inter[image_level](v,u) = 0.5f*(yy_old[image_level](v,u) + yy_warped[image_level](v,u));
				null(v, u) = false;
				if ((u>0)&&(v>0)&&(u<cols_i-1)&&(v<rows_i-1))
					num_valid_points++;
			}
		}
}

void CDifodo::calculateDepthDerivatives()
{
	dt.resize(rows_i,cols_i); dt.assign(0.f);
	du.resize(rows_i,cols_i); du.assign(0.f);
	dv.resize(rows_i,cols_i); dv.assign(0.f);

    //Compute connectivity
	MatrixXf rx_ninv(rows_i,cols_i);
	MatrixXf ry_ninv(rows_i,cols_i);
    rx_ninv.assign(1.f); ry_ninv.assign(1.f);

	for (unsigned int u = 0; u < cols_i-1; u++)
        for (unsigned int v = 0; v < rows_i; v++)
			if (null(v,u) == false)
			{
				rx_ninv(v,u) = sqrtf(square(xx_inter[image_level](v,u+1) - xx_inter[image_level](v,u))
									+ square(depth_inter[image_level](v,u+1) - depth_inter[image_level](v,u)));
			}

	for (unsigned int u = 0; u < cols_i; u++)
        for (unsigned int v = 0; v < rows_i-1; v++)
			if (null(v,u) == false)
			{
				ry_ninv(v,u) = sqrtf(square(yy_inter[image_level](v+1,u) - yy_inter[image_level](v,u))
									+ square(depth_inter[image_level](v+1,u) - depth_inter[image_level](v,u)));
			}


    //Spatial derivatives
    for (unsigned int v = 0; v < rows_i; v++)
    {
        for (unsigned int u = 1; u < cols_i-1; u++)
			if (null(v,u) == false)
				du(v,u) = (rx_ninv(v,u-1)*(depth_inter[image_level](v,u+1)-depth_inter[image_level](v,u)) + rx_ninv(v,u)*(depth_inter[image_level](v,u) - depth_inter[image_level](v,u-1)))/(rx_ninv(v,u)+rx_ninv(v,u-1));

		du(v,0) = du(v,1);
		du(v,cols_i-1) = du(v,cols_i-2);
    }

    for (unsigned int u = 0; u < cols_i; u++)
    {
        for (unsigned int v = 1; v < rows_i-1; v++)
			if (null(v,u) == false)
				dv(v,u) = (ry_ninv(v-1,u)*(depth_inter[image_level](v+1,u)-depth_inter[image_level](v,u)) + ry_ninv(v,u)*(depth_inter[image_level](v,u) - depth_inter[image_level](v-1,u)))/(ry_ninv(v,u)+ry_ninv(v-1,u));

		dv(0,u) = dv(1,u);
		dv(rows_i-1,u) = dv(rows_i-2,u);
    }

	//Temporal derivative
	for (unsigned int u = 0; u < cols_i; u++)
		for (unsigned int v = 0; v < rows_i; v++)
			if (null(v,u) == false)
				dt(v,u) = fps*(depth_warped[image_level](v,u) - depth_old[image_level](v,u));
}

void CDifodo::computeWeights()
{
	weights.resize(rows_i, cols_i);
	weights.assign(0.f);

	//Obtain the velocity associated to the rigid transformation estimated up to the present level
	Matrix<float,6,1> kai_level = kai_loc_old;

	Matrix4f acu_trans;
	acu_trans.setIdentity();
	for (unsigned int i=0; i<level; i++)
		acu_trans = transformations[i]*acu_trans;

	//Alternative way to compute the log
	CMatrixDouble44 mat_aux = acu_trans.cast<double>();
	poses::CPose3D aux(mat_aux);
	CArrayDouble<6> kai_level_acu = aux.ln()*fps;
	kai_level -= kai_level_acu.cast<float>();

	//Parameters for the measurement error
	const float f_inv = float(cols_i)/(2.f*tan(0.5f*fovh));
	const float kz2 = 8.122e-12f;  //square(1.425e-5) / 25
	
	//Parameters for linearization error
	const float kduv = 20e-5f;
	const float kdt = kduv/square(fps);
	const float k2dt = 5e-6f;
	const float k2duv = 5e-6f;
	
	for (unsigned int u = 1; u < cols_i-1; u++)
		for (unsigned int v = 1; v < rows_i-1; v++)
			if (null(v,u) == false)
			{
				//					Compute measurment error (simplified)
				//-----------------------------------------------------------------------
				const float z = depth_inter[image_level](v,u);
				const float inv_d = 1.f/z;
				//const float dycomp = du2(v,u)*f_inv_y*inv_d;
				//const float dzcomp = dv2(v,u)*f_inv_z*inv_d;
				const float z2 = z*z;
				const float z4 = z2*z2;

				//const float var11 = kz2*z4;
				//const float var12 = kz2*xx_inter[image_level](v,u)*z2*depth_inter[image_level](v,u);
				//const float var13 = kz2*yy_inter[image_level](v,u)*z2*depth_inter[image_level](v,u);
				//const float var22 = kz2*square(xx_inter[image_level](v,u))*z2;
				//const float var23 = kz2*xx_inter[image_level](v,u)*yy_inter[image_level](v,u)*z2;
				//const float var33 = kz2*square(yy_inter[image_level](v,u))*z2;
				const float var44 = kz2*z4*square(fps);
				const float var55 = kz2*z4*0.25f;
				const float var66 = var55;

				//const float j1 = -2.f*inv_d*inv_d*(xx_inter[image_level](v,u)*dycomp + yy_inter[image_level](v,u)*dzcomp)*(kai_level[0] + yy_inter[image_level](v,u)*kai_level[4] - xx_inter[image_level](v,u)*kai_level[5])
				//				+ inv_d*dycomp*(kai_level[1] - yy_inter[image_level](v,u)*kai_level[3]) + inv_d*dzcomp*(kai_level[2] + xx_inter[image_level](v,u)*kai_level[3]);
				//const float j2 = inv_d*dycomp*(kai_level[0] + yy_inter[image_level](v,u)*kai_level[4] - 2.f*xx_inter[image_level](v,u)*kai_level[5]) - dzcomp*kai_level[3];
				//const float j3 = inv_d*dzcomp*(kai_level[0] + 2.f*yy_inter[image_level](v,u)*kai_level[4] - xx_inter[image_level](v,u)*kai_level[5]) + dycomp*kai_level[3];

				const float j4 = 1.f;
				const float j5 =  xx_inter[image_level](v,u)*inv_d*inv_d*f_inv*(kai_level[0] + yy_inter[image_level](v,u)*kai_level[4] - xx_inter[image_level](v,u)*kai_level[5]) 
							   + inv_d*f_inv*(-kai_level[1] - z*kai_level[5] + yy_inter[image_level](v,u)*kai_level[3]);
				const float j6 = yy_inter[image_level](v,u)*inv_d*inv_d*f_inv*(kai_level[0] + yy_inter[image_level](v,u)*kai_level[4] - xx_inter[image_level](v,u)*kai_level[5])
							   + inv_d*f_inv*(-kai_level[2] + z*kai_level[4] - xx_inter[image_level](v,u)*kai_level[3]);

				//error_measurement(v,u) = j1*(j1*var11+j2*var12+j3*var13) + j2*(j1*var12+j2*var22+j3*var23)
				//						+j3*(j1*var13+j2*var23+j3*var33) + j4*j4*var44 + j5*j5*var55 + j6*j6*var66;

				const float error_m = j4*j4*var44 + j5*j5*var55 + j6*j6*var66;

				
				//					Compute linearization error
				//-----------------------------------------------------------------------
				const float ini_du = depth_old[image_level](v,u+1) - depth_old[image_level](v,u-1);
				const float ini_dv = depth_old[image_level](v+1,u) - depth_old[image_level](v-1,u);
				const float final_du = depth_warped[image_level](v,u+1) - depth_warped[image_level](v,u-1);
				const float final_dv = depth_warped[image_level](v+1,u) - depth_warped[image_level](v-1,u);

				const float dut = ini_du - final_du;
				const float dvt = ini_dv - final_dv;
				const float duu = du(v,u+1) - du(v,u-1);
				const float dvv = dv(v+1,u) - dv(v-1,u);
				const float dvu = dv(v,u+1) - dv(v,u-1); //Completely equivalent to compute duv

				const float error_l = kdt*square(dt(v,u)) + kduv*(square(du(v,u)) + square(dv(v,u))) + k2dt*(square(dut) + square(dvt))
											+ k2duv*(square(duu) + square(dvv) + square(dvu));

				//Weight
				weights(v,u) = sqrt(1.f/(error_m + error_l));
			}

	//Normalize weights in the range [0,1]
	const float inv_max = 1.f/weights.maximum();
	weights = inv_max*weights;
}

void CDifodo::solveOneLevel()
{
	MatrixXf A(num_valid_points,6);
	MatrixXf B(num_valid_points,1);
	unsigned int cont = 0;

	//Fill the matrix A and the vector B
	//The order of the unknowns is (vz, vx, vy, wz, wx, wy)
	//The points order will be (1,1), (1,2)...(1,cols-1), (2,1), (2,2)...(row-1,cols-1).

	const float f_inv = float(cols_i)/(2.f*tan(0.5f*fovh));

	for (unsigned int u = 1; u < cols_i-1; u++)
		for (unsigned int v = 1; v < rows_i-1; v++)
			if (null(v,u) == false)
			{
				// Precomputed expressions
				const float d = depth_inter[image_level](v,u);
				const float inv_d = 1.f/d;
				const float x = xx_inter[image_level](v,u);
				const float y = yy_inter[image_level](v,u);
				const float dycomp = du(v,u)*f_inv*inv_d;
				const float dzcomp = dv(v,u)*f_inv*inv_d;
				const float tw = weights(v,u);

				//Fill the matrix A
				A(cont, 0) = tw*(1.f + dycomp*x*inv_d + dzcomp*y*inv_d);
				A(cont, 1) = tw*(-dycomp);
				A(cont, 2) = tw*(-dzcomp);
				A(cont, 3) = tw*(dycomp*y - dzcomp*x);
				A(cont, 4) = tw*(y + dycomp*inv_d*y*x + dzcomp*(y*y*inv_d + d));
				A(cont, 5) = tw*(-x - dycomp*(x*x*inv_d + d) - dzcomp*inv_d*y*x);
				B(cont,0) = tw*(-dt(v,u));

				cont++;
			}
	
	//Solve the linear system of equations using weighted least squares
	MatrixXf AtA, AtB;
	AtA.multiply_AtA(A);
	AtB.multiply_AtB(A,B);
	MatrixXf Var = AtA.ldlt().solve(AtB);

	//Covariance matrix calculation 
	MatrixXf res = -B;
	for (unsigned int k = 0; k<6; k++)
		res += Var(k)*A.col(k);

	est_cov = (1.f/float(num_valid_points-6))*AtA.inverse()*res.squaredNorm();

	//Update last velocity in local coordinates
	kai_loc_level = Var;
}

void CDifodo::odometryCalculation()
{
	//Clock to measure the runtime
	utils::CTicTac clock;
	clock.Tic();

	//Build the gaussian pyramid
	if (fast_pyramid)	buildCoordinatesPyramidFast();
	else				buildCoordinatesPyramid();

	//Coarse-to-fines scheme
    for (unsigned int i=0; i<ctf_levels; i++)
    {
		//Previous computations
		transformations[i].setIdentity();

		level = i;
		unsigned int s = pow(2.f,int(ctf_levels-(i+1)));
        cols_i = cols/s; rows_i = rows/s;
        image_level = ctf_levels - i + round(log(float(width/cols))/log(2.f)) - 1;

		//1. Perform warping
		if (i == 0)
		{
			depth_warped[image_level] = depth[image_level];
			xx_warped[image_level] = xx[image_level];
			yy_warped[image_level] = yy[image_level];
		}
		else
			performWarping();

		//2. Calculate inter coords and find null measurements
		calculateCoord();

		//3. Compute derivatives
		calculateDepthDerivatives();

		//4. Compute weights
		computeWeights();

		//5. Solve odometry
		if (num_valid_points > 6)
			solveOneLevel();

		//6. Filter solution
		filterLevelSolution();
	}

	//Update poses
	poseUpdate();

	//Save runtime
	execution_time = 1000.f*clock.Tac();   
}

void CDifodo::filterLevelSolution()
{
	//		Calculate Eigenvalues and Eigenvectors
	//----------------------------------------------------------
	SelfAdjointEigenSolver<MatrixXf> eigensolver(est_cov);
	if (eigensolver.info() != Success) 
	{ 
		printf("\n Eigensolver couldn't find a solution. Pose is not updated");
		return;
	}
	
	//First, we have to describe both the new linear and angular velocities in the "eigenvector" basis
	//-------------------------------------------------------------------------------------------------
	Matrix<float,6,6> Bii;
	Matrix<float,6,1> kai_b;
	Bii = eigensolver.eigenvectors();
	kai_b = Bii.colPivHouseholderQr().solve(kai_loc_level);

	//Second, we have to describe both the old linear and angular velocities in the "eigenvector" basis
	//-------------------------------------------------------------------------------------------------
	Matrix<float,6,1> kai_loc_sub = kai_loc_old;

	//Important: we have to substract the previous levels' solutions from the old velocity.
	Matrix4f acu_trans;
	acu_trans.setIdentity();
	for (unsigned int i=0; i<level; i++)
		acu_trans = transformations[i]*acu_trans;

	CMatrixDouble44 mat_aux = acu_trans.cast<double>();
	poses::CPose3D aux(mat_aux);
	CArrayDouble<6> kai_level_acu = aux.ln()*fps;
	kai_loc_sub -= kai_level_acu.cast<float>();

	//Matrix<float, 4, 4> log_trans = fps*acu_trans.log();
	//kai_loc_sub(0) -= log_trans(0,3); kai_loc_sub(1) -= log_trans(1,3); kai_loc_sub(2) -= log_trans(2,3);
	//kai_loc_sub(3) += log_trans(1,2); kai_loc_sub(4) -= log_trans(0,2); kai_loc_sub(5) += log_trans(0,1);

	//Transform that local representation to the "eigenvector" basis
	Matrix<float,6,1> kai_b_old;
	kai_b_old = Bii.colPivHouseholderQr().solve(kai_loc_sub);

	//									Filter velocity
	//--------------------------------------------------------------------------------
	const float cf = previous_speed_eig_weight*expf(-int(level)), df = previous_speed_const_weight*expf(-int(level));
	Matrix<float,6,1> kai_b_fil;
	for (unsigned int i=0; i<6; i++)
		kai_b_fil(i) = (kai_b(i) + (cf*eigensolver.eigenvalues()(i,0) + df)*kai_b_old(i))/(1.f + cf*eigensolver.eigenvalues()(i) + df);

	//Transform filtered velocity to the local reference frame 
	Matrix<float, 6, 1> kai_loc_fil = Bii.inverse().colPivHouseholderQr().solve(kai_b_fil);

	//Compute the rigid transformation
	mrpt::math::CArrayDouble<6> aux_vel = kai_loc_fil.cast<double>()/fps;
	poses::CPose3D aux1, aux2; 
	CMatrixDouble44 trans;
	aux2 = aux1.exp(aux_vel);
	aux2.getHomogeneousMatrix(trans);
	transformations[level] = trans.cast<float>();
}

void CDifodo::poseUpdate()
{
	//First, compute the overall transformation
	//---------------------------------------------------
	Matrix4f acu_trans;
	acu_trans.setIdentity();
	for (unsigned int i=1; i<=ctf_levels; i++)
		acu_trans = transformations[i-1]*acu_trans;


	//Compute the new estimates in the local and absolutes reference frames
	//---------------------------------------------------------------------
	CMatrixDouble44 mat_aux = acu_trans.cast<double>();
	poses::CPose3D aux(mat_aux);
	CArrayDouble<6> kai_level_acu = aux.ln()*fps;
	kai_loc = kai_level_acu.cast<float>();

	//---------------------------------------------------------------------------------------------
	//Directly from Eigen:
	//- Eigen 3.1.0 needed for Matrix::log()
	//- The line "#include <unsupported/Eigen/MatrixFunctions>" should be uncommented (CDifodo.h)
	//
	//Matrix<float, 4, 4> log_trans = fps*acu_trans.log();
	//kai_loc(0) = log_trans(0,3); kai_loc(1) = log_trans(1,3); kai_loc(2) = log_trans(2,3);
	//kai_loc(3) = -log_trans(1,2); kai_loc(4) = log_trans(0,2); kai_loc(5) = -log_trans(0,1);
	//---------------------------------------------------------------------------------------------

	CMatrixDouble33 inv_trans;
	CMatrixFloat31 v_abs, w_abs;

	cam_pose.getRotationMatrix(inv_trans);
	v_abs = inv_trans.cast<float>()*kai_loc.topRows(3);
	w_abs = inv_trans.cast<float>()*kai_loc.bottomRows(3);
	kai_abs.topRows<3>() = v_abs;
	kai_abs.bottomRows<3>() = w_abs;	


	//						Update poses
	//-------------------------------------------------------	
	cam_oldpose = cam_pose;
	CMatrixDouble44 aux_acu = acu_trans;
	poses::CPose3D pose_aux(aux_acu);
	cam_pose = cam_pose + pose_aux;


	//Compute the velocity estimate in the new ref frame (to be used by the filter in the next iteration)
	//---------------------------------------------------------------------------------------------------
	cam_pose.getRotationMatrix(inv_trans);
	kai_loc_old.topRows<3>() = inv_trans.inverse().cast<float>()*kai_abs.topRows(3);
	kai_loc_old.bottomRows<3>() = inv_trans.inverse().cast<float>()*kai_abs.bottomRows(3);
}

void CDifodo::setFOV(float new_fovh, float new_fovv)
{
	fovh = M_PI*new_fovh/180.0;
	fovv = M_PI*new_fovv/180.0;
}

void CDifodo::getPointsCoord(MatrixXf &x, MatrixXf &y, MatrixXf &z)
{
	x.resize(rows,cols);
	y.resize(rows,cols);
	z.resize(rows,cols);

	z = depth_inter[0];
	x = xx_inter[0];
	y = yy_inter[0];
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


