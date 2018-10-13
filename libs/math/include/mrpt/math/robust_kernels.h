/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <cmath>  // std::sqrt()

namespace mrpt::math
{
/** \addtogroup mrpt_math_grp
 *  @{ */

/** The different types of kernels for usage within a robustified least-squares
 * estimator.
 * \sa Use these types as arguments of the template RobustKernel<>
 */
enum TRobustKernelType
{
	/** No robust kernel, use standard least squares: rho(r)= 1/2 * r^2 */
	rkLeastSquares = 0,
	/** Pseudo-huber robust kernel */
	rkPseudoHuber
};

// Generic declaration.
template <TRobustKernelType KERNEL_TYPE, typename T = double>
struct RobustKernel;

/** No robust kernel, use standard least squares: rho(r)   = r^2 */
template <typename T>
struct RobustKernel<rkLeastSquares, T>
{
	/** The kernel parameter (the "threshold") squared [Not used in this class,
	 * provided for consistency with the other classes] */
	T param_sq;

	/** Evaluates the kernel function for the squared error r2 and returns
	 * robustified squared error and derivatives of sqrt(2*rho(r)) at this
	 * point. */
	inline T eval(const T r2, T& out_1st_deriv, T& out_2nd_deriv)
	{
		out_1st_deriv = 1;
		out_2nd_deriv = 0;
		return r2;  // return: 2*cost;  cost: 0.5* |r|^2
	}
};

/** Pseudo-huber robust kernel: rho(r)   = 2 * delta^2 * ( -1+sqrt( 1+
 * r^2/delta^2 ) ) */
template <typename T>
struct RobustKernel<rkPseudoHuber, T>
{
	/** The kernel parameter (the "threshold") squared. */
	T param_sq;

	/** Evaluates the kernel function for the squared error r2 and returns
	 * robustified squared error and derivatives of sqrt(2*rho(r)) at this
	 * point. */
	inline T eval(const T r2, T& out_1st_deriv, T& out_2nd_deriv)
	{
		const T param_sq_inv = 1.0 / param_sq;
		const T a = 1 + r2 * param_sq_inv;
		const T b = std::sqrt(a);
		out_1st_deriv = 1. / b;
		out_2nd_deriv = -0.5 * param_sq_inv * out_1st_deriv / a;
		return 2 * param_sq * (b - 1);
		;  // return: 2*cost
	}
};

/** @} */  // end of grouping
}  // namespace mrpt::math
