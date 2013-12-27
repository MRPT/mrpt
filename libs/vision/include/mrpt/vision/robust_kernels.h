/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_vision_robust_kernels_H
#define mrpt_vision_robust_kernels_H

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/TMatchingPair.h>

#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
namespace vision
{
	/** \addtogroup mrpt_vision_grp
	  *  @{ */

	/** The different types of kernels for usage within a robustified least-squares estimator.
	  * \sa Use these types as arguments of the template RobustKernel<>
	  */
	enum TRobustKernelType
	{
		rkLeastSquares = 0,   //!< No robust kernel, use standard least squares: rho(r)= 1/2 * r^2
		rkPseudoHuber         //!< Pseudo-huber robust kernel
	};

	// Generic declaration.
	template <int TRobustKernelType, typename T=double> struct RobustKernel;

	/** No robust kernel, use standard least squares: rho(r)= 1/2 * r^2
	  * \note the "static_cast<>" in the declaration is to avoid a bug in MSVC 2008
	  */
	template <typename T>
	struct RobustKernel< static_cast<int>(rkLeastSquares), T >
	{
		/** Evaluates the kernel function at error norm = "delta" and returns 
		  *  robustified squared error and, optionally, the 1st derivative of sqrt(2*rho(r)) at this point.
		  */
		inline T eval(
			const T delta, 
			T * out_1st_deriv=NULL)
		{
			if (out_1st_deriv) *out_1st_deriv = 1;
			//if (out_2nd_deriv) *out_2nd_deriv = 0;
			// cost: 0.5* |delta|^2
			return delta*delta; // return: 2*cost   
		}			
	};

	/** Pseudo-huber robust kernel: rho(r) = b^2 * (-1+sqrt(1+(r\b)^2))
	  * \note the "static_cast<>" in the declaration is to avoid a bug in MSVC 2008
	  */
	template <typename T>
	struct RobustKernel< static_cast<int>(rkPseudoHuber), T >
	{
		T   b_sq; //!< The kernel parameter "b" squared (b^2).

		/** Evaluates the kernel function at error norm = "delta" and returns 
		  *  robustified squared error and, optionally, the 1st derivative of sqrt(2*rho(r)) at this point.
		  */
		inline T eval(
			const T delta, 
			T * out_1st_deriv=NULL)
		{
			const double r = delta*delta/b_sq;
			const double n = std::sqrt(1+r);
			const double cost = b_sq*(n-1);
			if (out_1st_deriv) *out_1st_deriv = ( 1.414213562373095 * delta)/(2* n * std::sqrt(b_sq*(n-1)) );
			//if (out_2nd_deriv) *out_2nd_deriv = ((-3*1.414213562373095*n)*delta*delta + 2*1.414213562373095*b_sq*pow(1+r,1.5) - 2*1.414213562373095*b_sq)/(4*pow(1+r,1.5)*pow(b_sq*n - b_sq,1.5));
			return 2*cost; // return: 2*cost
		}			
	};




	/** @} */ // end of grouping
}
}

#endif
