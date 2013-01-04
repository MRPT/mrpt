/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
