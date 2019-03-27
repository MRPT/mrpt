/* +------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)            |
|                          https://www.mrpt.org/                         |
|                                                                        |
| Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
| See: https://www.mrpt.org/Authors - All rights reserved.               |
| Released under BSD License. See: https://www.mrpt.org/License          |
+------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CVectorDynamic.h>
#include <Eigen/Dense>

using namespace mrpt::math;

template <typename T>
CMatrixDynamic<float> CMatrixDynamic<T>::cast_float() const
{
	CMatrixDynamic<float> r(rows(), cols());
	r.asEigen() = asEigen().template cast<float>();
	return r;
}
template <typename T>
CMatrixDynamic<double> CMatrixDynamic<T>::cast_double() const
{
	CMatrixDynamic<double> r(rows(), cols());
	r.asEigen() = asEigen().template cast<double>();
	return r;
}

template <typename T>
CVectorDynamic<T> CMatrixDynamic<T>::llt_solve(const CVectorDynamic<T>& b) const
{
	auto ret = CVectorDynamic<T>(asEigen().llt().solve(b.asEigen()));
	return ret;
}
template <typename T>
CVectorDynamic<T> CMatrixDynamic<T>::lu_solve(const CVectorDynamic<T>& b) const
{
	auto ret = CVectorDynamic<T>(asEigen().lu().solve(b.asEigen()));
	return ret;
}

// Template instantiation:
#define DO_MATDYN_INSTANTIATION(T_) \
	template class mrpt::math::CMatrixDynamic<T_>;

DO_MATDYN_INSTANTIATION(float)
DO_MATDYN_INSTANTIATION(double)
