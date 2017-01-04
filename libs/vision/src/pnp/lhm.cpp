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
#include <Eigen/StdVector>

// Opencv 2.3 had a broken <opencv/eigen.h> in Ubuntu 14.04 Trusty => Disable PNP classes
#include <mrpt/config.h>
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM<0x240
#	undef MRPT_HAS_OPENCV
#	define MRPT_HAS_OPENCV 0
#endif


#include "lhm.h"
using namespace mrpt::vision::pnp;

lhm::lhm(Eigen::MatrixXd obj_pts_, Eigen::MatrixXd img_pts_, Eigen::MatrixXd cam_, int n0) : F(n0)
{
	obj_pts = obj_pts_;
	img_pts = img_pts_;
	cam_intrinsic = cam_;
	n = n0;

	// Store obj_pts as 3XN and img_projections as 2XN matrices 
	P = obj_pts.transpose();
	Q = Eigen::MatrixXd::Ones(3, n);

	Q = img_pts.transpose();

	t.setZero();
}

void lhm::estimate_t()
{
	Eigen::Vector3d sum_;
	sum_.setZero();
	for (int i = 0; i < n; i++)
		sum_ += F[i] * R*P.col(i);
	t = G*sum_;
}

void lhm::xform()
{
	for (int i = 0; i < n; i++)
		Q.col(i) = R*P.col(i) + t;
}

Eigen::Matrix4d lhm::qMatQ(Eigen::VectorXd q)
{
	Eigen::Matrix4d Q_(4, 4);

	Q_ << q(0), -q(1), -q(2), -q(3),
		q(1), q(0), -q(3), q(2),
		q(2), q(3), q(0), -q(1),
		q(3), -q(2), q(1), q(0);

	return Q_;
}

Eigen::Matrix4d lhm::qMatW(Eigen::VectorXd q)
{
	Eigen::Matrix4d Q_(4, 4);

	Q_ << q(0), -q(1), -q(2), -q(3),
		q(1), q(0), q(3), -q(2),
		q(2), -q(3), q(0), q(1),
		q(3), q(2), -q(1), q(0);

	return Q_;
}

void lhm::absKernel()
{
	for (int i = 0; i < n; i++)
		Q.col(i) = F[i] * Q.col(i);

	Eigen::Vector3d P_bar, Q_bar;
	P_bar = P.rowwise().mean();
	Q_bar = Q.rowwise().mean();

	for (int i = 0; i < n; i++)
	{
		P.col(i) = P.col(i) - P_bar;
		Q.col(i) = Q.col(i) - Q_bar;
	}

	//<------------------- Use SVD Solution ------------------->//
	/*
	Eigen::Matrix3d M;
	M.setZero();

	for (i = 0; i < n; i++)
		M += P.col(i)*Q.col(i).transpose();

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);

	R = svd.matrixV()*svd.matrixU().transpose();

	Eigen::Matrix3d dummy;
	dummy.col(0) = -svd.matrixV().col(0);
	dummy.col(1) = -svd.matrixV().col(1);
	dummy.col(2) = svd.matrixV().col(2);

	if (R.determinant() == 1)
	{
		estimate_t();
		if (t(2) < 0)
		{
			R = dummy*svd.matrixU().transpose();
			estimate_t();
		}
	}
	else
	{
		R = -dummy*svd.matrixU().transpose();
		estimate_t();
		if (t(2) < 0)
		{
			R = -svd.matrixV()*svd.matrixU().transpose();
			estimate_t();
		}
	}

	err2 = 0;
	xform();

	Eigen::Vector3d vec;
	Eigen::Matrix3d I3 = Eigen::MatrixXd::Identity(3, 3);

	for (i = 0; i < n; i++)
	{
		vec = (I3 - F[i])*Q.col(i);
		err2 += vec.squaredNorm();
	}
    */
	//<------------------- Use QTN Solution ------------------->//

	Eigen::Matrix4d A;
	A.setZero();

	for (int i = 0; i < n; i++)
	{
		Eigen::Vector4d q1, q2;
		q1 << 1, Q.col(i);
		q2 << 1, P.col(i);
		A += qMatQ(q1).transpose()*qMatW(q2);
	}

	Eigen::EigenSolver<Eigen::Matrix4d> es(A);

	const Eigen::Matrix4d Ae = es.pseudoEigenvalueMatrix();
	Eigen::Vector4d D; // Ae.diagonal(); for some reason this leads to an internal compiler error in MSVC11... (sigh)
	for (int i=0;i<4;i++) D[i] = Ae(i,i);

	Eigen::Matrix4d V_mat = es.pseudoEigenvectors();

	Eigen::Vector4d::Index max_index;

	D.maxCoeff(&max_index);

	Eigen::Vector4d V;

	V = V_mat.col(max_index);

	Eigen::Quaterniond q(V(0), V(1), V(2), V(3));

	R = q.toRotationMatrix();

	estimate_t();
	
	err2 = 0;
	xform();

	Eigen::Vector3d vec;
	Eigen::Matrix3d I3 = Eigen::MatrixXd::Identity(3, 3);

	for (int i = 0; i < n; i++)
	{
		vec = (I3 - F[i])*Q.col(i);
		err2 += vec.squaredNorm();
	}

}

bool lhm::compute_pose(Eigen::Ref<Eigen::Matrix3d> R_, Eigen::Ref<Eigen::Vector3d> t_)
{
	int i, j = 0;

	Eigen::VectorXd p_bar;
	Eigen::Matrix3d sum_F, I3;
	I3 = Eigen::MatrixXd::Identity(3, 3);
	sum_F.setZero();

	p_bar = P.rowwise().mean();

	for (i = 0; i<n; i++)
	{
		P.col(i) -= p_bar;
		F[i] = Q.col(i)*Q.col(i).transpose() / Q.col(i).squaredNorm();
		sum_F = sum_F + F[i];
	}

	G = (I3 - sum_F / n).inverse() / n;

	err = 0;
	err2 = 1000;
	absKernel();

	while (std::abs(err2 - err) > TOL_LHM && err2 > EPSILON_LHM)
	{
		err = err2;

		absKernel();
		
		j += 1;
		if (j > 100)
			break;
	}

	R_ = R;
	t_ = t - R*p_bar;

	return 1;
}

