/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h"   // Precompiled headers

#include <mrpt/nav/planners/nav_plan_geometry_utils.h>
#include <mrpt/math/poly_roots.h>

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
	syms x y d ux uy ox oy R real
	f=(x+d*ux-ox)^2+(y+d*uy-oy)^2-R^2
	coeffs -> 
	[ (ox - x)^2 + (oy - y)^2 - R^2, - 2*ux*(ox - x) - 2*uy*(oy - y), ux^2 + uy^2]
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
