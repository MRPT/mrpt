/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers
#include <iostream>
#include <mrpt/utils/types_math.h> // Eigen must be included first via MRPT to enable the plugin system
#include <Eigen/Dense>
#include <Eigen/SVD>

#include "posit.h"


mrpt::vision::pnp::posit::posit(Eigen::MatrixXd obj_pts_, Eigen::MatrixXd img_pts_, Eigen::MatrixXd camera_intrinsic_, int n0)
{
		obj_pts=obj_pts_;
		img_pts=img_pts_.block(0,0,n0,2);
		cam_intrinsic=camera_intrinsic_;
		R=Eigen::MatrixXd::Identity(3,3);
		t=Eigen::VectorXd::Zero(3);
		f=(cam_intrinsic(0,0)+cam_intrinsic(1,1))/2;
		
		obj_matrix=(obj_pts.transpose() * obj_pts).inverse() * obj_pts.transpose();
	
		n=n0;
		
		obj_vecs=Eigen::MatrixXd::Zero(n0,3);
		
		for(int i=0;i<n;i++)
			obj_vecs.row(i)=obj_pts.row(i)-obj_pts.row(0);
		
		img_vecs = Eigen::MatrixXd::Zero(n0,2);
		img_vecs_old=img_vecs;
		
		epsilons=Eigen::VectorXd::Zero(n);
}

void mrpt::vision::pnp::posit::POS()
{
	Eigen::Vector3d I0, J0 , r1, r2, r3;
	double I0_norm, J0_norm;
	
	int i;
	double scale;
	
	for(i=0;i<3;i++)
	{
		I0(i)=obj_matrix.row(i).dot(img_vecs.col(0));
		J0(i)=obj_matrix.row(i).dot(img_vecs.col(1));
	}
	
    
	I0_norm=I0.norm();
	J0_norm=J0.norm();
	
	scale=(I0_norm + J0_norm)/2;
	
	/*Computing TRANSLATION */
	t(0)=img_pts(0,0)/scale;
	t(1)=img_pts(0,1)/scale;
	t(2)=f/scale;
	
	/* Computing ROTATION */
	r1=I0/I0_norm;
	r2=J0/J0_norm;
	r3=r1.cross(r2);
	
	R.row(0)=r1;
	R.row(1)=r2;
	R.row(2)=r3;
}

/** 
Iterate over results obtained by the POS function;
see paper "Model-Based Object Pose in 25 Lines of Code", IJCV 15, pp. 123-141, 1995.
*/
bool mrpt::vision::pnp::posit::compute_pose(Eigen::Ref<Eigen::Matrix3d> R_, Eigen::Ref<Eigen::Vector3d> t_)
{
	Eigen::FullPivLU<Eigen::MatrixXd> lu(obj_pts);
	if(lu.rank()<3)
		return false;
		
	int i, iCount;
	long imageDiff=1000;
	
	for(iCount=0; iCount<LOOP_MAX_COUNT;iCount++)
	{
		if(iCount==0)
		{
			for(i=0;i<img_vecs.rows();i++)
				img_vecs.row(i)=img_pts.row(i)-img_pts.row(0);
		}
		
		else
		{
			// Compute new image vectors
			epsilons.setZero();
			for(i=0; i<n; i++)
			{
				epsilons(i)+=obj_vecs.row(i).dot(R.row(2));
			}
			epsilons/=t(2);
			
			// Corrected image vectors 	
			for(i=0; i<n; i++)
			{
				img_vecs.row(i)= img_pts.row(i) * (1+epsilons(i)) -img_pts.row(0);
			}
			
			imageDiff=this->get_img_diff();
			
		}
		
		img_vecs_old=img_vecs;
		
		this->POS();
		
		if(iCount>0 && imageDiff==0)
			break;
			
		if(iCount==LOOP_MAX_COUNT)
		{
			std::cout<<"Solution Not converged"<<std::endl<<std::endl;
			break;
		}
		
	}
	R_=R;
	t_=t;

	return true;
}

long mrpt::vision::pnp::posit::get_img_diff()
{
	int i, j;
	long sumOfDiffs = 0;
	
	for (i=0;i<n;i++){
		for (j=0;j<2;j++){
			sumOfDiffs += std::abs(floor(0.5+img_vecs(i,j))-floor(0.5+img_vecs_old(i,j)));
		}
	}
	return sumOfDiffs;
	
}





