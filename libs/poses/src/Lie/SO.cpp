/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/config.h>  // for HAVE_SINCOS
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/otherlibs/sophus/so3.hpp>
#include <cmath>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::poses::Lie;

// See .h for documentation

// ====== SO(3) ===========
SO<3>::type SO<3>::exp(const SO<3>::tangent_vector& x)
{
	auto R = Sophus::SO3<double>::exp(x);
	return R.matrix();
}

SO<3>::tang2mat_jacob SO<3>::jacob_dexpe_de(const SO<3>::tangent_vector& x)
{
	/*
	   0   0   0
	   0   0   1
	   0  -1   0
	   0   0  -1
	   0   0   0
	   1   0   0
	   0   1   0
	  -1   0   0
	   0   0   0
	 */
	tang2mat_jacob J = tang2mat_jacob::Zero();
	J(1, 2) = J(5, 0) = J(6, 1) = +1.0;
	J(2, 1) = J(3, 2) = J(7, 0) = -1.0;
	return J;
}

SO<3>::tangent_vector SO<3>::log(const SO<3>::type& R)
{
	Sophus::SO3<double> Rot(R);
	const auto& r = Rot.log();
	tangent_vector ret;
	for (int i = 0; i < 3; i++) ret[i] = r[i];
	return ret;
}

SO<3>::type SO<3>::fromYPR(
    const double yaw, const double pitch, const double roll)
{
#ifdef HAVE_SINCOS
	double cy, sy;
	::sincos(yaw, &sy, &cy);
	double cp, sp;
	::sincos(pitch, &sp, &cp);
	double cr, sr;
	::sincos(roll, &sr, &cr);
#else
	const double cy = cos(yaw);
	const double sy = sin(yaw);
	const double cp = cos(pitch);
	const double sp = sin(pitch);
	const double cr = cos(roll);
	const double sr = sin(roll);
#endif

	// clang-format off
	alignas(MRPT_MAX_ALIGN_BYTES) const double rot_vals[] = {
	    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
	    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
	    -sp, cp * sr, cp * cr
	};
	// clang-format on
	SO<3>::type R(mrpt::math::UNINITIALIZED_MATRIX);
	R.loadFromArray(rot_vals);
	return R;
}

template <typename VEC3, typename MAT3x3, typename MAT3x9>
inline void M3x9(const VEC3& a, const MAT3x3& B, MAT3x9& RES)
{
	// clang-format off
	alignas(MRPT_MAX_ALIGN_BYTES) const double vals[] = {
	    a[0], -B(0, 2), B(0, 1), B(0, 2), a[0], -B(0, 0), -B(0, 1), B(0, 0), a[0],
	    a[1], -B(1, 2), B(1, 1), B(1, 2), a[1], -B(1, 0), -B(1, 1), B(1, 0), a[1],
	    a[2], -B(2, 2), B(2, 1), B(2, 2), a[2], -B(2, 0), -B(2, 1), B(2, 0), a[2]
	};
	// clang-format on
	RES.loadFromArray(vals);
}

SO<3>::mat2tang_jacob SO<3>::jacob_dlogv_dv(const SO<3>::type& R)
{
	using namespace mrpt::math;

	const double d = 0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1);
	CArrayDouble<3> a;
	CMatrixDouble33 B(UNINITIALIZED_MATRIX);
	if (d > 0.99999)
	{
		a[0] = a[1] = a[2] = 0;
		B.unit(3, -0.5);
	}
	else
	{
		const double theta = acos(d);
		const double d2 = square(d);
		const double sq = std::sqrt(1 - d2);
		a = SO<3>::vee_RmRt(R);
		a *= (d * theta - sq) / (4 * (sq * sq * sq));
		B.unit(3, -theta / (2 * sq));
	}
	CMatrixDouble39 M(UNINITIALIZED_MATRIX);
	M3x9(a, B, M);
	return M;
}

SO<3>::tangent_vector SO<3>::vee_RmRt(const SO<3>::type& R)
{
	SO<3>::tangent_vector v;
	v[0] = R(2, 1) - R(1, 2);
	v[1] = R(0, 2) - R(2, 0);
	v[2] = R(1, 0) - R(0, 1);
	return v;
}

// ====== SO(2) ===========
SO<2>::type SO<2>::exp(const SO<2>::tangent_vector& x)
{
	return mrpt::math::wrapToPi(x[0]);
}

SO<2>::tang2mat_jacob SO<2>::jacob_dexpe_de(const SO<2>::tangent_vector& x)
{
	tang2mat_jacob J;
	J(0, 0) = 1.0;
	return J;
}

SO<2>::tangent_vector SO<2>::log(const SO<2>::type& R)
{
	tangent_vector v;
	v[0] = mrpt::math::wrapToPi(R);
	return v;
}

SO<2>::mat2tang_jacob SO<2>::jacob_dlogv_dv(const SO<2>::type& R)
{
	mat2tang_jacob J;
	J(0, 0) = 1.0;
	return J;
}
