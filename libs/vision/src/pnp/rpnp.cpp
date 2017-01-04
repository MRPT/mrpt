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
#include <vector>
#include <cmath>

#include <mrpt/utils/types_math.h> // Eigen must be included first via MRPT to enable the plugin system
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/StdVector>
#include <unsupported/Eigen/Polynomials>

#include "rpnp.h"


mrpt::vision::pnp::rpnp::rpnp(Eigen::MatrixXd obj_pts_, Eigen::MatrixXd img_pts_, Eigen::MatrixXd cam_, int n0)
{
	obj_pts = obj_pts_;
	img_pts = img_pts_;
	cam_intrinsic = cam_;
	n = n0;

	// Store obj_pts as 3XN and img_projections as 2XN matrices 
	P = obj_pts.transpose();
	Q = img_pts.transpose();

	for (int i = 0; i < n; i++)
		Q.col(i) = Q.col(i) / Q.col(i).norm();

    R.setZero();
	t.setZero();
}

bool mrpt::vision::pnp::rpnp::compute_pose(Eigen::Ref<Eigen::Matrix3d> R_, Eigen::Ref<Eigen::Vector3d> t_)
{
	// selecting an edge $P_{ i1 }P_{ i2 }$ by n random sampling
	int i1 = 0, i2 = 1;
	double lmin = Q(0, i1)*Q(0, i2) + Q(1, i1)*Q(1, i2) + Q(2, i1)*Q(2, i2);

	Eigen::MatrixXi rij (n,2);
    
    R_=Eigen::MatrixXd::Identity(3,3);
    t_=Eigen::Vector3d::Zero();
    
	for (int i = 0; i < n; i++)
		for (int j = 0; j < 2; j++)
			rij(i, j) = rand() % n;

	for (int ii = 0; ii < n; ii++)
	{
		int i = rij(ii, 0), j = rij(ii,1);

		if (i == j)
			continue;

		double l = Q(0, i)*Q(0, j) + Q(1, i)*Q(1, j) + Q(2, i)*Q(2, j);

		if (l < lmin)
		{
			i1 = i;
			i2 = j;
			lmin = l;
		}
	}

	// calculating the rotation matrix of $O_aX_aY_aZ_a$.
	Eigen::Vector3d p1, p2, p0, x, y, z, dum_vec;

	p1 = P.col(i1);
	p2 = P.col(i2);
	p0 = (p1 + p2) / 2;

	x = p2 - p0; x /= x.norm();

	if (std::abs(x(1)) < std::abs(x(2)) )
	{
		dum_vec << 0, 1, 0;
		z = x.cross(dum_vec); z /= z.norm();
		y = z.cross(x); y /= y.norm();
	}
	else
	{
		dum_vec << 0, 0, 1;
		y = dum_vec.cross(x); y /= y.norm();
		z = x.cross(y); x /= x.norm();
	}

	Eigen::Matrix3d R0;

	R0.col(0) = x; R0.col(1) =y; R0.col(2) = z;

	for (int i = 0; i < n; i++)
		P.col(i) = R0.transpose() * (P.col(i) - p0);

	// Dividing the n - point set into(n - 2) 3 - point subsets
	// and setting up the P3P equations

	Eigen::Vector3d v1 = Q.col(i1), v2 = Q.col(i2);
	double cg1 = v1.dot(v2);
	double sg1 = sqrt(1 - cg1*cg1);
	double D1 = (P.col(i1) - P.col(i2)).norm();
	Eigen::MatrixXd D4(n - 2, 5);
    
	int j = 0;
    Eigen::Vector3d vi;
    Eigen::VectorXd rowvec(5);
	for (int i = 0; i < n; i++)
	{
		if (i == i1 || i == i2)
			continue;

		vi = Q.col(i);
		double cg2 = v1.dot(vi); 
		double cg3 = v2.dot(vi);
		double sg2 = sqrt(1 - cg2*cg2);
		double D2 = (P.col(i1) - P.col(i)).norm();
		double D3 = (P.col(i) - P.col(i2)).norm();

		// get the coefficients of the P3P equation from each subset.
		
		rowvec = getp3p(cg1, cg2, cg3, sg1, sg2, D1, D2, D3);
		D4.row(j) = rowvec;
		j += 1;
      
        if(j>n-3)
            break;
	}

	Eigen::VectorXd D7(8), dumvec(8), dumvec1(5);
	D7.setZero();
    
	for (int i = 0; i < n-2; i++)
	{
        dumvec1 = D4.row(i);
		dumvec= getpoly7(dumvec1);
		D7 += dumvec;
	}
    
	Eigen::PolynomialSolver<double, 7> psolve(D7.reverse());
	Eigen::VectorXcd comp_roots = psolve.roots().transpose();
	Eigen::VectorXd real_comp, imag_comp;
	real_comp = comp_roots.real();
	imag_comp = comp_roots.imag();

	Eigen::VectorXd::Index max_index;

	double max_real= real_comp.cwiseAbs().maxCoeff(&max_index);

    std::vector<double> act_roots_;

	int cnt=0;
    
	for (int i=0; i<imag_comp.size(); i++ )
	{
		if(std::abs(imag_comp(i))/max_real<0.001)
		{
				act_roots_.push_back(real_comp(i));
                cnt++;
		}
	}
    
    double* ptr = &act_roots_[0];
    Eigen::Map<Eigen::VectorXd> act_roots(ptr, cnt); 
    
	if (cnt==0)
    {
		return false;
    }
    
    Eigen::VectorXd act_roots1(cnt);
    act_roots1 << act_roots.segment(0,cnt);

	std::vector<Eigen::Matrix3d> R_cum(cnt);
	std::vector<Eigen::Vector3d> t_cum(cnt);
	std::vector<double> err_cum(cnt);
    
	for(int i=0; i<cnt; i++)
	{
		double root = act_roots(i);

		// Compute the rotation matrix

		double d2 = cg1 + root;

		Eigen::Vector3d unitx, unity, unitz;
		unitx << 1,0,0;
		unity << 0,1,0;
		unitz << 0,0,1;
		x = v2*d2 -v1;
		x/=x.norm();
		if (std::abs(unity.dot(x)) < std::abs(unitz.dot(x)))
		{
			z = x.cross(unity);z/=z.norm();
			y=z.cross(x); y/y.norm();
		}
		else
		{
			y=unitz.cross(x); y/=y.norm();
			z = x.cross(y); z/=z.norm();
		}
		R.col(0)=x;
		R.col(1)=y;
		R.col(2)=z;

		//calculating c, s, tx, ty, tz

		Eigen::MatrixXd D(2 * n, 6);
		D.setZero();

		R0 = R.transpose();
		Eigen::VectorXd r(Eigen::Map<Eigen::VectorXd>(R0.data(), R0.cols()*R0.rows()));
        
		for (int j = 0; j<n; j++)
		{
			double ui = img_pts(j, 0), vi = img_pts(j, 1), xi = P(0, j), yi = P(1, j), zi = P(2, j);
			D.row(2 * j) << -r(1)*yi + ui*(r(7)*yi + r(8)*zi) - r(2)*zi,
				-r(2)*yi + ui*(r(8)*yi - r(7)*zi) + r(1)*zi,
				-1,
				0,
				ui,
				ui*r(6)*xi - r(0)*xi;
			
			D.row(2 * j + 1) << -r(4)*yi + vi*(r(7)*yi + r(8)*zi) - r(5)*zi,
				-r(5)*yi + vi*(r(8)*yi - r(7)*zi) + r(4)*zi,
				0,
				-1,
				vi,
				vi*r(6)*xi - r(3)*xi;
		}
        
		Eigen::MatrixXd DTD = D.transpose()*D;

		Eigen::EigenSolver<Eigen::MatrixXd> es(DTD);

		Eigen::VectorXd Diag = es.pseudoEigenvalueMatrix().diagonal();

		Eigen::MatrixXd V_mat = es.pseudoEigenvectors();

		Eigen::MatrixXd::Index min_index;

		Diag.minCoeff(&min_index);

		Eigen::VectorXd V = V_mat.col(min_index);

		V /= V(5);

		double c = V(0), s = V(1);
		t << V(2), V(3), V(4);

		//calculating the camera pose by 3d alignment
		Eigen::VectorXd xi, yi, zi;
		xi = P.row(0); 
		yi = P.row(1);
		zi = P.row(2);

		Eigen::MatrixXd XXcs(3, n), XXc(3,n);
		XXc.setZero();

		XXcs.row(0) = r(0)*xi + (r(1)*c + r(2)*s)*yi + (-r(1)*s + r(2)*c)*zi + t(0)*Eigen::VectorXd::Ones(n);
		XXcs.row(1) = r(3)*xi + (r(4)*c + r(5)*s)*yi + (-r(4)*s + r(5)*c)*zi + t(1)*Eigen::VectorXd::Ones(n);
		XXcs.row(2) = r(6)*xi + (r(7)*c + r(8)*s)*yi + (-r(7)*s + r(8)*c)*zi + t(2)*Eigen::VectorXd::Ones(n);

		for (int ii = 0; ii < n; ii++)
			XXc.col(ii) = Q.col(ii)*XXcs.col(ii).norm();

		Eigen::Matrix3d R2;
		Eigen::Vector3d t2;

		Eigen::MatrixXd XXw = obj_pts.transpose();

		calcampose(XXc, XXw, R2, t2);

		R_cum[i] = R2;
		t_cum[i] = t2;

		for (int k = 0; k < n; k++)
			XXc.col(k) = R2 * XXw.col(k) + t2;

		Eigen::MatrixXd xxc(2, n);
		
		xxc.row(0) = XXc.row(0).array() / XXc.row(2).array();
		xxc.row(1) = XXc.row(1).array() / XXc.row(2).array();

		double res = ((xxc.row(0) - img_pts.col(0).transpose()).norm() + (xxc.row(1) - img_pts.col(1).transpose()).norm()) / 2;

		err_cum[i] = res;
	
	}
    
	int pos_cum = std::min_element(err_cum.begin(), err_cum.end()) - err_cum.begin();

	R_ = R_cum[pos_cum];
	t_ = t_cum[pos_cum];
    
	return true;
}

void mrpt::vision::pnp::rpnp::calcampose(Eigen::MatrixXd& XXc, Eigen::MatrixXd& XXw, Eigen::Matrix3d& R2, Eigen::Vector3d& t2)
{
	Eigen::MatrixXd X = XXc;
	Eigen::MatrixXd Y = XXw;
	Eigen::MatrixXd K = Eigen::MatrixXd::Identity(n, n) - Eigen::MatrixXd::Ones(n, n) * 1 / n;
	Eigen::VectorXd ux, uy;
	uy = X.rowwise().mean();
	ux = Y.rowwise().mean();

	// Need to verify sigmax2
	double sigmax2 = (((X*K).array() * (X*K).array()).colwise().sum()).mean();

	Eigen::MatrixXd SXY = Y*K*(X.transpose()) / n;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(SXY, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::Matrix3d S = Eigen::MatrixXd::Identity(3, 3);
	if (SXY.determinant() <0)
		S(2, 2) = -1;

	R2 = svd.matrixV()*S*svd.matrixU().transpose();

	double c2 = (svd.singularValues().asDiagonal()*S).trace() / sigmax2;
	t2 = uy - c2*R2*ux;
	 
	Eigen::Vector3d x, y, z;
	x = R2.col(0);
	y = R2.col(1);
	z = R2.col(2);

	if ((x.cross(y) - z).norm()>0.02)
		R2.col(2) = -R2.col(2);
}

Eigen::VectorXd mrpt::vision::pnp::rpnp::getpoly7(const Eigen::VectorXd& vin)
{
	Eigen::VectorXd vout(8);
	vout << 4 * pow(vin(0), 2),
		7 * vin(1) * vin(0),
		6 * vin(2)*vin(0) + 3 * pow(vin(1), 2),
		5 * vin(3)*vin(0) + 5 * vin(2)*vin(1),
		4 * vin(4)*vin(0) + 4 * vin(3)*vin(1) + 2 * pow(vin(2), 2),
		3 * vin(4)*vin(1) + 3 * vin(3)*vin(2),
		2 * vin(4)*vin(2) + pow(vin(3), 2),
		vin(4)*vin(3);
	return vout;
}

Eigen::VectorXd mrpt::vision::pnp::rpnp::getp3p(double l1, double l2, double A5, double C1, double C2, double D1, double D2, double D3)
{
	double A1 = (D2 / D1)*(D2 / D1);
	double A2 = A1*pow(C1, 2) - pow(C2, 2);
	double A3 = l2*A5 - l1;
	double A4 = l1*A5 - l2;
	double A6 = (pow(D3, 2) - pow(D1, 2) - pow(D2, 2)) / (2 * pow(D1, 2));
	double A7 = 1 - pow(l1, 2) - pow(l2, 2) + l1*l2*A5 + A6*pow(C1, 2);

    Eigen::VectorXd vec(5);

	vec << pow(A6, 2) - A1*pow(A5, 2),
		2 * (A3*A6 - A1*A4*A5),
		pow(A3, 2) + 2 * A6*A7 - A1*pow(A4, 2) - A2*pow(A5, 2),
		2 * (A3*A7 - A2*A4*A5),
		pow(A7, 2) - A2*pow(A4, 2);
        
    return vec;
}
