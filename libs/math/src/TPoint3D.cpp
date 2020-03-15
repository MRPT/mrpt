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
#include <mrpt/serialization/CArchive.h>  // impl of << operator

namespace mrpt::math
{
static_assert(std::is_trivial_v<TPoint3D_data<float>>);
static_assert(std::is_trivial_v<TPoint3D_data<double>>);
static_assert(std::is_trivially_copyable_v<TPoint3D>);
static_assert(std::is_trivially_copyable_v<TPoint3Df>);

template <typename T>
TPoint3D_<T>::TPoint3D_(const TPoint2D_<T>& p) : TPoint3D_data<T>{p.x, p.y, 0}
{
}

template <typename T>
TPoint3D_<T>::TPoint3D_(const TPose2D& p)
	: TPoint3D_data<T>{static_cast<T>(p.x), static_cast<T>(p.y), 0}
{
}

template <typename T>
TPoint3D_<T>::TPoint3D_(const TPose3D& p)
	: TPoint3D_data<T>{static_cast<T>(p.x), static_cast<T>(p.y),
					   static_cast<T>(p.z)}
{
}

template <typename T>
bool TPoint3D_<T>::operator<(const TPoint3D_<T>& p) const
{
	if (this->x < p.x)
		return true;
	else if (this->x > p.x)
		return false;
	else if (this->y < p.y)
		return true;
	else if (this->y > p.y)
		return false;
	else
		return this->z < p.z;
}

template <typename T>
void TPoint3D_<T>::fromString(const std::string& s)
{
	mrpt::math::CMatrixDynamic<T> m;
	if (!m.fromMatlabStringFormat(s))
		THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(
		m.rows() == 1 && m.cols() == 3, "Wrong size of vector in ::fromString");
	this->x = m(0, 0);
	this->y = m(0, 1);
	this->z = m(0, 2);
}

// Explicit instantiations:
template struct TPoint3D_<float>;
template struct TPoint3D_<double>;

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TPointXYZfRGBu8& p)
{
	return in >> p.pt >> p.r >> p.g >> p.b;
}
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TPointXYZfRGBu8& p)
{
	return out << p.pt << p.r << p.g << p.b;
}

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TPointXYZfRGBAu8& p)
{
	return in >> p.pt >> p.r >> p.g >> p.b >> p.a;
}
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TPointXYZfRGBAu8& p)
{
	return out << p.pt << p.r << p.g << p.b << p.a;
}

}  // namespace mrpt::math
