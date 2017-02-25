/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h"   // Precompiled headers

#include <mrpt/nav/planners/nav_plan_geometry_utils.h>
#include <mrpt/math/poly_roots.h>
#include <mrpt/math/wrap2pi.h>

using namespace mrpt;
using namespace mrpt::math;

bool mrpt::nav::collision_free_dist_segment_circ_robot(
	const mrpt::math::TPoint2D &p0, const mrpt::math::TPoint2D &p1,
	const double R,
	const mrpt::math::TPoint2D &o,
	double &out_col_dist)
{
	using mrpt::utils::square;

	out_col_dist = -1.0;

	// Unit vector from start -> end:
	mrpt::math::TPoint2D u = (p1 - p0);
	const double L = u.norm();
	ASSERT_ABOVE_(L, 1e-10);
	u *= 1.0 / L;

	/*
	syms x y d ux uy o.x o.y R real
	f=(x+d*ux-o.x)^2+(y+d*uy-o.y)^2-R^2
	coeffs -> 
	[ (o.x - x)^2 + (o.y - y)^2 - R^2, - 2*ux*(o.x - x) - 2*uy*(o.y - y), ux^2 + uy^2]
	*/
	
	// quadratic eq: a*d^2 + b*d+c=0
	const double a = square(u.x) + square(u.y);
	const double b = -2 * u.x*(o.x - p0.x) - 2 * u.y*(o.y - p0.y);
	const double c = square(o.x - p0.x) + square(o.y - p0.y) - square(R);

	double r1, r2;
	const int nsols = mrpt::math::solve_poly2(a, b, c, r1, r2);

	if (nsols <= 0) return false;
	double r_min;
	if (nsols == 1) r_min = r1; else {
		if (r1 < 0 && r2 < 0) return false;
		if (r1 < 0) {
			r_min = r2;
		}
		else if (r2 < 0) {
			r_min = r1;
		}
		else {
			r_min = std::min(r1, r2);
		}
	}

	if (r_min > L) return false;

	// A real, valid collision:
	out_col_dist = r_min;
	return true;
}

bool mrpt::nav::collision_free_dist_arc_circ_robot(
	const double arc_radius, const double R,
	const mrpt::math::TPoint2D &o, double &out_col_dist)
{
	ASSERT_ABOVE_(std::abs(arc_radius), 1e-10);
	out_col_dist = -1.0;

	const mrpt::math::TPoint2D ptArcCenter(.0, arc_radius);
	const double center2obs_dist = (ptArcCenter - o).norm();
	if (std::abs(center2obs_dist - std::abs(arc_radius))>R)
		return false;
	
	// x:
	const double r = arc_radius;
	const double discr = (R*r*2.0 - o.y*r*2.0 - R*R + o.x*o.x + o.y*o.y)*(R*r*2.0 + o.y*r*2.0 + R*R - o.x*o.x - o.y*o.y);
	if (discr < 0)
		return false;
	const double sol_x0 = ((R*R)*(-1.0 / 2.0) + (o.x*o.x)*(1.0 / 2.0) + (o.y*o.y)*(1.0 / 2.0) - (o.y*(-(R*R)*o.y + (R*R)*r + (o.x*o.x)*o.y + (o.x*o.x)*r - (o.y*o.y)*r + o.y*o.y*o.y + o.x*sqrt(discr))*(1.0 / 2.0)) / (o.y*r*-2.0 + o.x*o.x + o.y*o.y + r*r) + (r*(-(R*R)*o.y + (R*R)*r + (o.x*o.x)*o.y + (o.x*o.x)*r - (o.y*o.y)*r + o.y*o.y*o.y + o.x*sqrt(discr))*(1.0 / 2.0)) / (o.y*r*-2.0 + o.x*o.x + o.y*o.y + r*r)) / o.x;
	const double sol_x1 = ((R*R)*(-1.0 / 2.0) + (o.x*o.x)*(1.0 / 2.0) + (o.y*o.y)*(1.0 / 2.0) - (o.y*(-(R*R)*o.y + (R*R)*r + (o.x*o.x)*o.y + (o.x*o.x)*r - (o.y*o.y)*r + o.y*o.y*o.y - o.x*sqrt(discr))*(1.0 / 2.0)) / (o.y*r*-2.0 + o.x*o.x + o.y*o.y + r*r) + (r*(-(R*R)*o.y + (R*R)*r + (o.x*o.x)*o.y + (o.x*o.x)*r - (o.y*o.y)*r + o.y*o.y*o.y - o.x*sqrt(discr))*(1.0 / 2.0)) / (o.y*r*-2.0 + o.x*o.x + o.y*o.y + r*r)) / o.x;

	// y: 
	const double sol_y0 = ((R*R)*o.y*(-1.0 / 2.0) + (R*R)*r*(1.0 / 2.0) + (o.x*o.x)*o.y*(1.0 / 2.0) + (o.x*o.x)*r*(1.0 / 2.0) - (o.y*o.y)*r*(1.0 / 2.0) + (o.y*o.y*o.y)*(1.0 / 2.0) + o.x*sqrt(discr)*(1.0 / 2.0)) / (o.y*r*-2.0 + o.x*o.x + o.y*o.y + r*r);
	const double sol_y1 = ((R*R)*o.y*(-1.0 / 2.0) + (R*R)*r*(1.0 / 2.0) + (o.x*o.x)*o.y*(1.0 / 2.0) + (o.x*o.x)*r*(1.0 / 2.0) - (o.y*o.y)*r*(1.0 / 2.0) + (o.y*o.y*o.y)*(1.0 / 2.0) - o.x*sqrt(discr)*(1.0 / 2.0)) / (o.y*r*-2.0 + o.x*o.x + o.y*o.y + r*r);

	const mrpt::math::TPoint2D sol0(sol_x0, sol_y0), sol1(sol_x1, sol_y1);

	double th0 = atan2(sol0.x - ptArcCenter.x, -( sol0.y - ptArcCenter.y));  // (x,y) order is intentionally like this!
	double th1 = atan2(sol1.x - ptArcCenter.x, -(sol1.y - ptArcCenter.y));

	if (r > 0)
	{
		th0 = mrpt::math::wrapTo2Pi(th0);
		th1 = mrpt::math::wrapTo2Pi(th1);
	}
	else
	{
		th0 = mrpt::math::wrapTo2Pi(M_PI - th0);
		th1 = mrpt::math::wrapTo2Pi(M_PI - th1);
	}
	
	out_col_dist = std::abs(r)*std::min(th0, th1);
	return true;
}
