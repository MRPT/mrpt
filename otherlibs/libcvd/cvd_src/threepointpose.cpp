#include <TooN/gaussian_elimination.h>
#include <TooN/se3.h>
#include <vector>
#include "cvd_src/quartic.h"
#include <cvd/geometry/threepointpose.h>
using namespace TooN;
using namespace std;

namespace CVD {

static inline double square(double x) { return x*x; }

static SE3<> three_point_absolute_orientation(const array<Vector<3>,3>& x, const array<Vector<3>,3>& y)
{
    Matrix<3> D, D1;

    D[0] = x[1] - x[0];
    D[1] = x[2] - x[0];
    D[2] = D[1] ^ D[0];

    D1[0] = y[1] - y[0];
    D1[1] = y[2] - y[0];
    D1[2] = D1[1] ^ D1[0];


    SO3<> so3(gaussian_elimination(D, D1).T());

    Vector<3> T = y[0] - so3 * x[0];

    return SE3<>(so3, T);
}

int three_point_pose(const array<Vector<3>, 3>& xi, const array<Vector<2>,3>& zi, vector<SE3<> >& poses)
{
    double ab_sq = norm_sq(xi[1] - xi[0]);
    double ac_sq = norm_sq(xi[2] - xi[0]);
    double bc_sq = norm_sq(xi[2] - xi[1]);

    Vector<3> za = unit(unproject(zi[0])), zb = unit(unproject(zi[1])), zc = unit(unproject(zi[2]));

    double cos_ab = za*zb;
    double cos_ac = za*zc;
    double cos_bc = zb*zc;

    double K1 = bc_sq / ac_sq;
    double K2 = bc_sq / ab_sq;

    double p[5];
    {

	double K12 = K1*K2;
	double K12_1_2 = K12 - K1 - K2;
	double K2_1_K1_cab = K2*(1-K1)*cos_ab;


	p[4] = (square(K12_1_2)
		- 4*K12*cos_bc*cos_bc);
	p[3] = (4*(K12_1_2)*K2_1_K1_cab
		+ 4*K1*cos_bc*((K12+K2-K1)*cos_ac
			       + 2*K2*cos_ab*cos_bc));
	p[2] = (square(2*K2_1_K1_cab) +
		2*(K12 + K1 - K2)*K12_1_2
		+ 4*K1*((K1-K2)*cos_bc*cos_bc
			+ (1-K2)*K1*cos_ac*cos_ac
			- 2*K2*(1 + K1) *cos_ab*cos_ac*cos_bc));
	p[1] = (4*(K12 + K1 - K2)*K2_1_K1_cab
		+ 4*K1*((K12 - K1 + K2)*cos_ac*cos_bc
			+ 2*K12*cos_ab*cos_ac*cos_ac));
	p[0] = square(K12 + K1 - K2) - 4*K12*K1*cos_ac*cos_ac;
    }
    array<Vector<3>,3> xi1;

    double roots[4];
    double inv_p4 = 1.0 / p[4];
    for (int i=0; i<4; ++i)
	p[i] *= inv_p4;
    int nr = find_quartic_real_roots(p[3], p[2], p[1], p[0], roots);

    int count = 0;
    for (int i=0; i<nr; ++i) {
	double x = roots[i];
	if (x <= 0)
	    continue;
	for (int j=0; j<3; ++j)
	    x = newton_quartic(p[3], p[2], p[1], p[0], x);
	double xx = x*x;

	double a_den = xx - 2*x*cos_ab + 1;
	double a = sqrt(ab_sq / a_den);
	double b = a*x;

	double M = 1 - K1;
	double P = 2*(K1*cos_ac - x*cos_bc);
	double Q = xx - K1;

	double P1 = -2*x*cos_bc;
	double Q1 = xx - K2*a_den;

	double den = M*Q1 - Q;
	if (den == 0) {
	    cerr << "skipped" << endl;
	    continue;
	}

	double y = (P1*Q - P*Q1) / den;
	double c = a * y;

	xi1[0] = a*za;
	xi1[1] = b*zb;
	xi1[2] = c*zc;

	poses.push_back(three_point_absolute_orientation(xi, xi1));
	++count;
    }
    return count;
}

int three_point_pose(const array<Vector<3>, 3>& xi, const array<Vector<3>,3>& rays, vector<SE3<> >& poses)
{
    double ab_sq = norm_sq(xi[1] - xi[0]);
    double ac_sq = norm_sq(xi[2] - xi[0]);
    double bc_sq = norm_sq(xi[2] - xi[1]);

    Vector<3> za = unit(rays[0]), zb = unit(rays[1]), zc = unit(rays[2]);

    double cos_ab = za*zb;
    double cos_ac = za*zc;
    double cos_bc = zb*zc;

    double K1 = bc_sq / ac_sq;
    double K2 = bc_sq / ab_sq;

    double p[5];
    {

	double K12 = K1*K2;
	double K12_1_2 = K12 - K1 - K2;
	double K2_1_K1_cab = K2*(1-K1)*cos_ab;


	p[4] = (square(K12_1_2)
		- 4*K12*cos_bc*cos_bc);
	p[3] = (4*(K12_1_2)*K2_1_K1_cab
		+ 4*K1*cos_bc*((K12+K2-K1)*cos_ac
			       + 2*K2*cos_ab*cos_bc));
	p[2] = (square(2*K2_1_K1_cab) +
		2*(K12 + K1 - K2)*K12_1_2
		+ 4*K1*((K1-K2)*cos_bc*cos_bc
			+ (1-K2)*K1*cos_ac*cos_ac
			- 2*K2*(1 + K1) *cos_ab*cos_ac*cos_bc));
	p[1] = (4*(K12 + K1 - K2)*K2_1_K1_cab
		+ 4*K1*((K12 - K1 + K2)*cos_ac*cos_bc
			+ 2*K12*cos_ab*cos_ac*cos_ac));
	p[0] = square(K12 + K1 - K2) - 4*K12*K1*cos_ac*cos_ac;
    }
    array<Vector<3>,3> xi1;

    double roots[4];
    double inv_p4 = 1.0 / p[4];
    for (int i=0; i<4; ++i)
	p[i] *= inv_p4;
    int nr = find_quartic_real_roots(p[3], p[2], p[1], p[0], roots);

    int count = 0;
    for (int i=0; i<nr; ++i) {
	double x = roots[i];
	if (x <= 0)
	    continue;
	for (int j=0; j<3; ++j)
	    x = newton_quartic(p[3], p[2], p[1], p[0], x);
	double xx = x*x;

	double a_den = xx - 2*x*cos_ab + 1;
	double a = sqrt(ab_sq / a_den);
	double b = a*x;

	double M = 1 - K1;
	double P = 2*(K1*cos_ac - x*cos_bc);
	double Q = xx - K1;

	double P1 = -2*x*cos_bc;
	double Q1 = xx - K2*a_den;

	double den = M*Q1 - Q;
	if (den == 0) {
	    cerr << "skipped" << endl;
	    continue;
	}

	double y = (P1*Q - P*Q1) / den;
	double c = a * y;

	xi1[0] = a*za;
	xi1[1] = b*zb;
	xi1[2] = c*zc;

	poses.push_back(three_point_absolute_orientation(xi, xi1));
	++count;
    }
    return count;
}

}
