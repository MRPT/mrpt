/*********************************************************************
SO3-PnP
Algorithm to estimate Position and Orientation from image data
using SO3-PnP
**********************************************************************
Copyright(C)  <December 2015>  <Chandra P. Mangipudi, Perry Y. Li>
This program is free software : you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.
*********************************************************************/

#include <iostream>
#include <Eigen/Dense>

#include "so3.h"

#define epsilon 0.00001

mrpt::vision::so3::so3(const Eigen::MatrixXd& obj_pts_, const Eigen::MatrixXd& img_pts_, const Eigen::MatrixXd& cam_intrinsic_, int n0)
{
    obj_pts = obj_pts_;
    img_pts = img_pts_;
    
    cam_intrinsic = cam_intrinsic_;
    
    beta1 = Eigen::MatrixXd::Zero(2*n,3);
    Pr1 = Eigen::MatrixXd::Zero(3, 2*n);
    Pr = Eigen::MatrixXd::Zero(2*n, 2*n);
    
    Ra    = Eigen::MatrixXd::Identity(3,3);
    Rc    = Eigen::MatrixXd::Identity(3,3);
    R_ret = Eigen::MatrixXd::Identity(3,3); 
    R     = Eigen::MatrixXd::Identity(3,3); 
    step  = Eigen::MatrixXd::Zero(3,3);
    N     = Eigen::MatrixXd::Zero(3,3);
    
    rgm       = Eigen::VectorXd::Zero(3);
    Veca      = Eigen::VectorXd::Zero(3);
    Vecc      = Eigen::VectorXd::Zero(3);
    dummyrgm  = Eigen::VectorXd::Zero(3);
    ax        = Eigen::VectorXd::Zero(3);
    gam       = Eigen::VectorXd::Zero(2*n);
    err       = Eigen::VectorXd::Zero(2*n);
    
    k=0;
    k1=0;
    k2=0;
    k3=0;
    
    n=n0;
    
    for(k=0;k<n;k++)
		beta1.block(2*k, 0, 2, 2) = -cam_intrinsic.block(0,0,2,2);
        
    std::cout<<"Init Complete" << std::endl << std::endl;
    
}


//<---------------------- Function quatrot to get rotation matrix------------------------->//
Eigen::Matrix3d  mrpt::vision::so3::quatrot(Eigen::Vector3d  Vec)
{
	ax=Vec;
	ang = Vec.norm();
	ax = ax / ang;
	c_ang = cos(ang), s_ang = sin(ang);

	R_ret << c_ang + ax(0)*ax(0)*(1 - c_ang), ax(0)*ax(1)*(1 - c_ang) - ax(2)*s_ang, ax(0)*ax(2)*(1 - c_ang)+ax(1)*s_ang,
	         ax(0)*ax(1)*(1 - c_ang) + ax(2)*s_ang, c_ang + ax(1)*ax(1)*(1 - c_ang), ax(1)*ax(2)*(1 - c_ang)-ax(0)*s_ang,
		 ax(0)*ax(2)*(1 - c_ang) - ax(1)*s_ang, ax(1)*ax(2)*(1 - c_ang) + ax(0)*s_ang, c_ang + ax(2) *ax(2) * (1 - c_ang);

	return R_ret;
	
}

//<---------------------- Function to calcualte rotation error------------------------->//
void mrpt::vision::so3::err_calc(Eigen::Matrix3d & R, int flag, Eigen::VectorXd & err, Eigen::Vector3d & Vec, Eigen::Vector3d & rgm)
{
	gam=Eigen::VectorXd ::Zero(2*n);
	
	N = Eigen::MatrixXd ::Zero(3,3);

	for (k = 0; k < n; k++)
	{
		gam.segment(2 * k, 2) = beta1.block(2 * k, 0, 2, 3)*R*(obj_pts.row(k)).transpose();
	}
	if (flag)
		rgm = Pr1*(- gam);
	
	err = Pr*(- gam);

	for (k = 0; k < n; k++)
	{
		N = N + (obj_pts.row(k)).transpose()*(err.block(2 * k, 0, 2, 1)).transpose()*beta1.block(2 * k, 0, 2, 3);
	}

	step = R.transpose()*N.transpose() - N*R;

	Vec << step(2, 1), step(0, 2), step(1, 0);
	
	Vec /= Vec.norm();

}

//<---------------------- Function to calcualte optimal rotation matrix------------------------->//
void mrpt::vision::so3::findPosSO3(Eigen::Matrix3d & R_guess)
{
	R = R_guess;

	for (int k = 0; k < n; k++)
	{
		beta1(2*k,2)  = img_pts(k,0)-cam_intrinsic(0,2)/2;
		beta1(2*k+1,2)= img_pts(k,1)-cam_intrinsic(1,2)/2;
	}

	Pr1 = (beta1.transpose()*beta1).inverse()*beta1.transpose();

	Pr = Eigen::MatrixXd ::Identity(2 * n, 2 * n) - beta1*Pr1;
	
	Ra = R;

	err_calc(Ra, 0, err, Veca, dummyrgm);

	error_a = err.norm();

	error_c = error_a;
	Rc = Ra;
	Vecc = Veca;

	stepsize=M_PI/72;

	while (stepsize > M_PI / 1440)
	{
		k2++;
		
		while (Veca.transpose()*Vecc>0)
		{
			error_a = error_c;
			Ra = Rc;
			Veca = Vecc;
			Rc = Ra*quatrot(stepsize*Veca);
			err_calc(Rc, 0, err, Vecc, dummyrgm);
			error_c = err.norm();
			k3++;
            
		}

		while (Veca.transpose()*Vecc < 0)
		{
			stepsize *= 0.98 / (1 - Veca.transpose()*Vecc);
			Rc = Ra*quatrot(stepsize*Veca);
			err_calc(Rc, 0, err, Vecc, dummyrgm);
			error_c = err.norm();
			k1++;
            
		}
        
       
	}

	R = Rc;
	err_calc(R, 1, err, dummyrgm, rgm);
	
	//<--------------------- Display and Log Data -------------------------->
	//std::cout<<"R_Cam=\n"<<R<<std::endl<<std::endl;
	//std::cout<<"r_cam=\n"<<rgm<<std::endl<<std::endl;

}

bool mrpt::vision::so3::compute_pose(Eigen::Matrix3d& R_, Eigen::Vector3d& t_)
{
    if(R_(2,2)==1)
        findPosSO3(R_);
    else 
        findPosSO3(Ra);
    R_ = R;
    
    if(rgm(2)<0)
        t_ = -rgm;
    else 
        t_ = rgm;
    
    return true;
}

//<---------------------- Function to convert from DCM to Quaternion------------------------->//
Eigen::Vector4d  mrpt::vision::so3::dcm2quat(Eigen::Matrix3d  R)
{
	Eigen::VectorXd  q(4);
	q(0) = sqrt(abs((1 + R(0, 0) + R(1, 1) + R(2, 2)) / 4));

	if (abs(q(0))>epsilon)
	{
		q(1) = (R(2, 1) - R(1, 2)) / 4 / q(0);
		q(2) = (R(0, 2) - R(2, 0)) / 4 / q(0);
		q(3) = (R(1, 0) - R(0, 1)) / 4 / q(0);
	}
	else
	{
		if (R(0, 0) >= R(1, 1) && R(0, 0) >= R(2, 2))
		{
			q(1) = sqrt((1 + R(0, 0)) / 2);
			q(2) = R(0, 1) / sqrt(2 * (1 + R(0, 0)));
			q(3) = R(0, 2) / sqrt(2 * (1 + R(0, 0)));
		}
		else if (R(1, 1) >= R(0, 0) && R(1, 1) >= R(2, 2))
		{
			q(2) = sqrt((1 + R(1, 1)) / 2);
			q(1) = R(1, 0) / sqrt(2 * (1 + R(1, 1)));
			q(3) = R(1, 2) / sqrt(2 * (1 + R(1, 1)));
		}
		else if (R(2, 2) >= R(0, 0) && R(2, 2) >= R(0, 0))
		{
			q(3) = sqrt((1 + R(2, 2)) / 2);
			q(1) = R(2, 0) / sqrt(2 * (1 + R(2, 2)));
			q(2) = R(2, 1) / sqrt(2 * (1 + R(2, 2)));
		}
	}

	return q;
}


