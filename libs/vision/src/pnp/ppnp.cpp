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

#include "ppnp.h"

mrpt::vision::pnp::ppnp::ppnp(const Eigen::MatrixXd& obj_pts, const Eigen::MatrixXd& img_pts, const Eigen::MatrixXd& cam_intrinsic)
{
	P = img_pts;
	S = obj_pts;
	C = cam_intrinsic;
}

bool mrpt::vision::pnp::ppnp::compute_pose(Eigen::Matrix3d& R, Eigen::Vector3d& t, int n)
{
	double tol=0.00001;
	
	Eigen::MatrixXd I=Eigen::MatrixXd::Identity(n, n), A(n,n), Y(n,3), E(n,3), E_old(n,3), U,V, I3 =Eigen::MatrixXd::Identity(3, 3), PR, Z=Eigen::MatrixXd::Zero(n, n);
	Eigen::VectorXd e(n), II(n), c(3), Zmindiag(n);
	
	e.fill(1);
	II.fill(1.0/((double)n));
	
	A=I-e*e.transpose()/n;
	
	double err=std::numeric_limits<double>::infinity();
	
	E_old.fill(1000);
	
    int cnt =0;

	while(err>tol)
	{
		
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(P.transpose()*Z*A*S, Eigen::ComputeThinU | Eigen::ComputeThinV);
		U=svd.matrixU();
		V=svd.matrixV();
		
		I3(2,2) = (U*V.transpose()).determinant();
		R=U*I3*V.transpose();
		PR = P*R;
		
		c=(S-Z*PR).transpose()*II;

		Y=S-e*c.transpose();

		Zmindiag=((PR*Y.transpose()).diagonal()).array() / ((P.array()*P.array()).rowwise().sum()).array();
		
		for (int i = 0; i < n; i++)
			if (Zmindiag(i) < 0)
				Zmindiag(i) = 0;


		Z=Zmindiag.asDiagonal();
		
		E=Y-Z*PR;

		err=(E-E_old).norm();

		E_old=E;
        
        cnt ++;
    
	}

	t=-R*c;
	
	return 1;
	
	
}
