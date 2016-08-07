#include <iostream>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/StdVector>

#include "lhm.h"
using namespace mrpt::vision;

lhm::lhm(Eigen::MatrixXd obj_pts_, Eigen::MatrixXd img_pts_, Eigen::MatrixXd cam_, int n0) : F(n0)
{
	obj_pts = obj_pts_;
	img_pts = img_pts_;
	cam_intrinsic = cam_;
	n = n0;

	//std::cout << "obj_pts_=" << std::endl << obj_pts_ << std::endl << std::endl;
	//std::cout << "img_pts_=" << std::endl << img_pts_ << std::endl << std::endl;
	//std::cout << "cam_=" << std::endl << cam_ << std::endl << std::endl;


	// Store obj_pts as 3XN and img_projections as 2XN matrices 
	P = obj_pts.transpose();
	Q = Eigen::MatrixXd::Ones(3, n);

	Q = img_pts.transpose();

	//std::cout << "P=" << std::endl << P << std::endl << std::endl;
	//std::cout << "Q=" << std::endl << Q << std::endl << std::endl;

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

	//std::cout << "Q_xform =" << std::endl << Q << std::endl << std::endl;
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
	int i;

	for (i = 0; i < n; i++)
		Q.col(i) = F[i] * Q.col(i);

	Eigen::Vector3d P_bar, Q_bar;
	P_bar = P.rowwise().mean();
	Q_bar = Q.rowwise().mean();

	//std::cout<<"P_bar="<<std::endl<<P_bar<<std::endl<<std::endl;
	//std::cout<<"Q_bar="<<std::endl<<Q_bar<<std::endl<<std::endl;

	for (i = 0; i < n; i++)
	{
		P.col(i) = P.col(i) - P_bar;
		Q.col(i) = Q.col(i) - Q_bar;
	}

	//std::cout<<"P="<<std::endl<<P<<std::endl<<std::endl;
	//std::cout<<"Q="<<std::endl<<Q<<std::endl<<std::endl;

	//<------------------- Use SVD Solution ------------------->//
	/*
	Eigen::Matrix3d M;
	M.setZero();

	for (i = 0; i < n; i++)
		M += P.col(i)*Q.col(i).transpose();

	cout<<"M="<<endl<<M<<endl<<endl;

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

	cout << "R=" << endl << R << endl << endl;
	cout << "t=" << endl << t << endl << endl;

	err2 = 0;
	xform();

	Eigen::Vector3d vec;
	Eigen::Matrix3d I3 = Eigen::MatrixXd::Identity(3, 3);

	cout << "Q_out=" << endl << Q << endl << endl;

	for (i = 0; i < n; i++)
	{
		vec = (I3 - F[i])*Q.col(i);
		err2 += vec.squaredNorm();
	}

	cout << "err2=" << err2 << endl << endl;
	*/
	//<------------------- Use QTN Solution ------------------->//

	Eigen::Matrix4d A;
	A.setZero();

	for (i = 0; i < n; i++)
	{
		Eigen::Vector4d q1, q2;
		q1 << 1, Q.col(i);
		q2 << 1, P.col(i);
		A += qMatQ(q1).transpose()*qMatW(q2);
	}

	Eigen::EigenSolver<Eigen::Matrix4d> es(A);

	Eigen::Vector4d D = es.pseudoEigenvalueMatrix().diagonal();

	Eigen::Matrix4d V_mat = es.pseudoEigenvectors();

	Eigen::Vector4d::Index max_index;

	D.maxCoeff(&max_index);

	Eigen::Vector4d V;

	V = V_mat.col(max_index);

	//std::cout << "D=" << std::endl << D << std::endl << std::endl;
	//std::cout << "V=" << std::endl << V << std::endl << std::endl;

	Eigen::Quaterniond q(V(0), V(1), V(2), V(3));

	R = q.toRotationMatrix();

	//std::cout << "R=" << std::endl << R << std::endl << std::endl;

	estimate_t();
	//std::cout << "t=" << std::endl << t << std::endl << std::endl;


	err2 = 0;
	xform();

	Eigen::Vector3d vec;
	Eigen::Matrix3d I3 = Eigen::MatrixXd::Identity(3, 3);

	//std::cout << "Q_out=" << std::endl << Q << std::endl << std::endl;

	for (i = 0; i < n; i++)
	{
		vec = (I3 - F[i])*Q.col(i);
		err2 += vec.squaredNorm();
	}

	//std::cout << "err2=" << err2 << std::endl << std::endl;

}

bool lhm::compute_pose(Eigen::Ref<Eigen::Matrix3d> R_, Eigen::Ref<Eigen::Vector3d> t_)
{
	int i, j = 0;

	Eigen::VectorXd p_bar;
	Eigen::Matrix3d sum_F, I3;
	I3 = Eigen::MatrixXd::Identity(3, 3);
	sum_F.setZero();

	p_bar = P.rowwise().mean();

	//std::cout<<"p_bar="<< std::endl << p_bar<<std::endl<<std::endl;

	for (i = 0; i<n; i++)
	{
		P.col(i) -= p_bar;
		F[i] = Q.col(i)*Q.col(i).transpose() / Q.col(i).squaredNorm();
		sum_F = sum_F + F[i];

		//cout << "i= " << i << endl;
		//cout << F[i] << endl << endl;
	}

	//std::cout << "sum_F=" << std::endl << sum_F << std::endl << std::endl;

	G = (I3 - sum_F / n).inverse() / n;

	//std::cout << "G=" << std::endl << G << std::endl << std::endl;

	err = 0;
	err2 = 1000;
	absKernel();

	while (abs(err2 - err) > TOL_LHM && err2 > EPSILON_LHM)
	{
		err = err2;

		absKernel();
		
		//cout << "j=" << j << endl;
		//cout << abs(err2 - err) << endl << err2 << endl << endl;

		j += 1;
		if (j > 100)
			break;
	}

	R_ = R;
	t_ = t - R*p_bar;
    
    //std::cout<< "R=" << std::endl << R_ << std::endl << std::endl;
    //std::cout<< "t=" << std::endl << t_ << std::endl << std::endl;
    

	return 1;


}



