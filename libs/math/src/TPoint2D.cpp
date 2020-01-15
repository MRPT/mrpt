/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>

namespace mrpt::math
{
static_assert(std::is_trivial_v<TPoint2D_data<float>>);
static_assert(std::is_trivial_v<TPoint2D_data<double>>);
static_assert(std::is_trivially_copyable_v<TPoint2D>);
static_assert(std::is_trivially_copyable_v<TPoint2Df>);

template <typename T>
TPoint2D_<T>::TPoint2D_(const TPose2D& p)
	: TPoint2D_data<T>{static_cast<T>(p.x), static_cast<T>(p.y)}
{
}

template <typename T>
TPoint2D_<T>::TPoint2D_(const TPoint3D_<T>& p) : TPoint2D_data<T>{p.x, p.y}
{
}

template <typename T>
TPoint2D_<T>::TPoint2D_(const TPose3D& p)
	: TPoint2D_data<T>{static_cast<T>(p.x), static_cast<T>(p.y)}
{
}

template <typename T>
bool TPoint2D_<T>::operator<(const TPoint2D_<T>& p) const
{
	if (this->x < p.x)
		return true;
	else if (this->x > p.x)
		return false;
	else
		return this->y < p.y;
}

template <typename T>
void TPoint2D_<T>::fromString(const std::string& s)
{
	CMatrixDynamic<T> m;
	if (!m.fromMatlabStringFormat(s))
		THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(
		m.rows() == 1 && m.cols() == 2, "Wrong size of vector in ::fromString");
	this->x = m(0, 0);
	this->y = m(0, 1);
}

// Explicit instantiations:
template struct TPoint2D_<float>;
template struct TPoint2D_<double>;
}  // namespace mrpt::math
