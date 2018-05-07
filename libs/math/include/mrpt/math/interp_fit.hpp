/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */
#pragma once

#include <mrpt/math/interp_fit.h>

namespace mrpt::math
{
template <class T, class VECTOR>
T interpolate(const T& x, const VECTOR& ys, const T& x0, const T& x1)
{
	MRPT_START
	ASSERT_(x1 > x0);
	ASSERT_(!ys.empty());
	const size_t N = ys.size();
	if (x <= x0) return ys[0];
	if (x >= x1) return ys[N - 1];
	const T Ax = (x1 - x0) / T(N);
	const size_t i = int((x - x0) / Ax);
	if (i >= N - 1) return ys[N - 1];
	const T Ay = ys[i + 1] - ys[i];
	return ys[i] + (x - (x0 + i * Ax)) * Ay / Ax;
	MRPT_END
}

template <typename NUMTYPE, class VECTORLIKE>
NUMTYPE spline(
	const NUMTYPE t, const VECTORLIKE& x, const VECTORLIKE& y, bool wrap2pi)
{
	// Check input data
	ASSERT_(x.size() == 4 && y.size() == 4);
	ASSERT_(x[0] <= x[1] && x[1] <= x[2] && x[2] <= x[3]);
	ASSERT_(t > x[0] && t < x[3]);

	NUMTYPE h[3];
	for (unsigned int i = 0; i < 3; i++) h[i] = x[i + 1] - x[i];

	double k = 1 / (4 * h[0] * h[1] + 4 * h[0] * h[2] + 3 * h[1] * h[1] +
					4 * h[1] * h[2]);
	double a11 = 2 * (h[1] + h[2]) * k;
	double a12 = -h[1] * k;
	double a22 = 2 * (h[0] + h[1]) * k;

	double y0, y1, y2, y3;

	if (!wrap2pi)
	{
		y0 = y[0];
		y1 = y[1];
		y2 = y[2];
		y3 = y[3];
	}
	else
	{
		// Assure the function is linear without jumps in the interval:
		y0 = mrpt::math::wrapToPi(y[0]);
		y1 = mrpt::math::wrapToPi(y[1]);
		y2 = mrpt::math::wrapToPi(y[2]);
		y3 = mrpt::math::wrapToPi(y[3]);

		double Ay;

		Ay = y1 - y0;
		if (Ay > M_PI)
			y1 -= M_2PI;
		else if (Ay < -M_PI)
			y1 += M_2PI;

		Ay = y2 - y1;
		if (Ay > M_PI)
			y2 -= M_2PI;
		else if (Ay < -M_PI)
			y2 += M_2PI;

		Ay = y3 - y2;
		if (Ay > M_PI)
			y3 -= M_2PI;
		else if (Ay < -M_PI)
			y3 += M_2PI;
	}

	double b1 = (y2 - y1) / h[1] - (y1 - y0) / h[0];
	double b2 = (y3 - y2) / h[2] - (y2 - y1) / h[1];

	double z0 = 0;
	double z1 = 6 * (a11 * b1 + a12 * b2);
	double z2 = 6 * (a12 * b1 + a22 * b2);
	double z3 = 0;

	double res = 0;
	if (t < x[1])
		res = (z1 * pow((t - x[0]), 3) + z0 * pow((x[1] - t), 3)) / (6 * h[0]) +
			  (y1 / h[0] - h[0] / 6 * z1) * (t - x[0]) +
			  (y0 / h[0] - h[0] / 6 * z0) * (x[1] - t);
	else
	{
		if (t < x[2])
			res = (z2 * pow((t - x[1]), 3) + z1 * pow((x[2] - t), 3)) /
					  (6 * h[1]) +
				  (y2 / h[1] - h[1] / 6 * z2) * (t - x[1]) +
				  (y1 / h[1] - h[1] / 6 * z1) * (x[2] - t);
		else if (t < x[3])
			res = (z3 * pow((t - x[2]), 3) + z2 * pow((x[3] - t), 3)) /
					  (6 * h[2]) +
				  (y3 / h[2] - h[2] / 6 * z3) * (t - x[2]) +
				  (y2 / h[2] - h[2] / 6 * z2) * (x[3] - t);
	}
	return wrap2pi ? mrpt::math::wrapToPi(res) : res;
}

template <typename NUMTYPE, class VECTORLIKE, int NUM_POINTS>
NUMTYPE leastSquareLinearFit(
	const NUMTYPE t, const VECTORLIKE& x, const VECTORLIKE& y, bool wrap2pi)
{
	MRPT_START

	// http://en.wikipedia.org/wiki/Linear_least_squares
	ASSERT_(x.size() == y.size());
	ASSERT_(x.size() > 1);

	const size_t N = x.size();

	// X= [1 columns of ones, x' ]
	const NUMTYPE x_min = x.minimum();
	Eigen::Matrix<NUMTYPE, 2, NUM_POINTS> Xt;
	Xt.resize(2, N);
	for (size_t i = 0; i < N; i++)
	{
		Xt.set_unsafe(0, i, 1);
		Xt.set_unsafe(1, i, x[i] - x_min);
	}

	const auto B = ((Xt * Xt.transpose()).inv().eval() * Xt * y).eval();
	ASSERT_(B.size() == 2);

	NUMTYPE ret = B[0] + B[1] * (t - x_min);

	// wrap?
	if (!wrap2pi)
		return ret;
	else
		return mrpt::math::wrapToPi(ret);

	MRPT_END
}

template <
	class VECTORLIKE1, class VECTORLIKE2, class VECTORLIKE3, int NUM_POINTS>
void leastSquareLinearFit(
	const VECTORLIKE1& ts, VECTORLIKE2& outs, const VECTORLIKE3& x,
	const VECTORLIKE3& y, bool wrap2pi)
{
	MRPT_START

	// http://en.wikipedia.org/wiki/Linear_least_squares
	ASSERT_(x.size() == y.size());
	ASSERT_(x.size() > 1);

	const size_t N = x.size();

	// X= [1 columns of ones, x' ]
	using NUM = typename VECTORLIKE3::value_type;
	const NUM x_min = x.minimum();
	Eigen::Matrix<NUM, 2, NUM_POINTS> Xt;
	Xt.resize(2, N);
	for (size_t i = 0; i < N; i++)
	{
		Xt.set_unsafe(0, i, 1);
		Xt.set_unsafe(1, i, x[i] - x_min);
	}

	const auto B = ((Xt * Xt.transpose()).inv().eval() * Xt * y).eval();
	ASSERT_(B.size() == 2);

	const size_t tsN = size_t(ts.size());
	outs.resize(tsN);
	if (!wrap2pi)
		for (size_t k = 0; k < tsN; k++)
			outs[k] = B[0] + B[1] * (ts[k] - x_min);
	else
		for (size_t k = 0; k < tsN; k++)
			outs[k] = mrpt::math::wrapToPi(B[0] + B[1] * (ts[k] - x_min));
	MRPT_END
}
}  // namespace mrpt


