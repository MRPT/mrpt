/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/types_math.h>  // for dynamic_vector, CVector...
#include <mrpt/math/CSplineInterpolator1D.h>  // for CSplineInterpolator1D
#include <mrpt/serialization/stl_serialization.h>  // for operator<<, operator>>
#include <map>  // for _Rb_tree_const_iterator
#include <mrpt/math/interp_fit.hpp>  // for spline
#include <mrpt/serialization/CSerializable.h>  // for CSerializable, CSeriali...

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CSplineInterpolator1D, CSerializable, mrpt::math)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CSplineInterpolator1D::CSplineInterpolator1D(bool wrap2pi) : m_wrap2pi(wrap2pi)
{
}

/*---------------------------------------------------------------
						appendXY
  ---------------------------------------------------------------*/
void CSplineInterpolator1D::appendXY(double x, double y) { m_x2y[x] = y; }
/*---------------------------------------------------------------
						query
  ---------------------------------------------------------------*/
double& CSplineInterpolator1D::query(double x, double& y, bool& out_valid) const
{
	out_valid = false;
	y = 0;

	std::pair<double, double> p1, p2, p3, p4;

	auto it_ge1 = m_x2y.lower_bound(x);

	// Exact match?
	if (it_ge1 != m_x2y.end() && it_ge1->first == x)
	{
		y = it_ge1->second;
		out_valid = true;
		return y;
	}

	// Are we in the beginning or the end of the path?
	if (it_ge1 == m_x2y.end() || it_ge1 == m_x2y.begin())
	{
		return y;
	}

	p3 = *it_ge1;  // Third pair
	++it_ge1;
	if (it_ge1 == m_x2y.end())
	{
		return y;
	}
	p4 = *it_ge1;  // Fourth pair

	--it_ge1;
	--it_ge1;
	p2 = *it_ge1;  // Second pair

	if (it_ge1 == m_x2y.begin())
	{
		return y;
	}

	p1 = *(--it_ge1);  // First pair

	// ---------------------------------------
	//    SPLINE INTERPOLATION
	// ---------------------------------------
	CVectorDouble xs(4);
	xs[0] = p1.first;
	xs[1] = p2.first;
	xs[2] = p3.first;
	xs[3] = p4.first;

	CVectorDouble ys(4);
	ys[0] = p1.second;
	ys[1] = p2.second;
	ys[2] = p3.second;
	ys[3] = p4.second;

	out_valid = true;
	return y = math::spline(x, xs, ys, m_wrap2pi);
}

uint8_t CSplineInterpolator1D::serializeGetVersion() const { return 0; }
void CSplineInterpolator1D::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << m_x2y << m_wrap2pi;
}

void CSplineInterpolator1D::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:  // floats
		{
			in >> m_x2y >> m_wrap2pi;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}
