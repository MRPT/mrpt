/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef  CLevenbergMarquardt_H
#define  CLevenbergMarquardt_H

#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/utils.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
namespace math
{
	/** An implementation of the Levenberg-Marquardt algorithm for least-square minimization.
	 *
	 *  Refer to this <a href="http://www.mrpt.org/Levenberg%E2%80%93Marquardt_algorithm" >page</a> for more details on the algorithm and its usage.
	 *
	 * \tparam NUMTYPE The numeric type for all the operations (float, double, or long double)
	 * \tparam USERPARAM The type of the "y" input to the user supplied evaluation functor. Default type is a vector of NUMTYPE.
	 * \ingroup mrpt_base_grp
	 */
	template <typename VECTORTYPE = mrpt::vector_double, class USERPARAM = VECTORTYPE >
	class CLevenbergMarquardtTempl : public mrpt::utils::CDebugOutputCapable
	{
	public:
		typedef typename VECTORTYPE::value_type  NUMTYPE;


		/** The type of the function passed to execute. The user must supply a function which evaluates the error of a given point in the solution space.
		  *  \param x The state point under examination.
		  *  \param y The same object passed to "execute" as the parameter "userParam".
		  *  \param out The vector of (non-squared) errors, of the average square root error, for the given "x". The functor code must set the size of this vector.
		  */
		typedef void (*TFunctorEval)(
			const VECTORTYPE &x,
			const USERPARAM &y,
			VECTORTYPE &out);

		/** The type of an optional functor passed to \a execute to replace the Euclidean addition "x_new = x_old + x_incr" by any other operation.
		  */
		typedef void (*TFunctorIncrement)(
			VECTORTYPE &x_new,
			const VECTORTYPE &x_old,
			const VECTORTYPE &x_incr,
			const USERPARAM &user_param);

		struct TResultInfo
		{
			NUMTYPE		final_sqr_err;
			size_t		iterations_executed;
			VECTORTYPE	last_err_vector;		//!< The last error vector returned by the user-provided functor.
			CMatrixTemplateNumeric<NUMTYPE>	path;	//!< Each row is the optimized value at each iteration.

			/** This matrix can be used to obtain an estimate of the optimal parameters covariance matrix:
			  *  \f[ COV = H M H^\top \f]
			  *  With COV the covariance matrix of the optimal parameters, H this matrix, and M the covariance of the input (observations).
			  */
 			//CMatrixTemplateNumeric<NUMTYPE> H;
		};

		/** Executes the LM-method, with derivatives estimated from
		  *  \a functor is a user-provided function which takes as input two vectors, in this order:
		  *		- x: The parameters to be optimized.
		  *		- userParam: The vector passed to the LM algorithm, unmodified.
		  *	  and must return the "error vector", or the error (not squared) in each measured dimension, so the sum of the square of that output is the overall square error.
		  *
		  * \a x_increment_adder Is an optional functor which may replace the Euclidean "x_new = x + x_increment" at the core of the incremental optimizer by
		  *     any other operation. It can be used for example, in on-manifold optimizations.
		  */
		static void	execute(
			VECTORTYPE			&out_optimal_x,
			const VECTORTYPE	&x0,
			TFunctorEval		functor,
			const VECTORTYPE	&increments,
			const USERPARAM		&userParam,
			TResultInfo			&out_info,
			bool				verbose = false,
			const size_t		maxIter = 200,
			const NUMTYPE		tau = 1e-3,
			const NUMTYPE		e1 = 1e-8,
			const NUMTYPE		e2 = 1e-8,
			bool                returnPath =true,
			TFunctorIncrement	x_increment_adder = NULL
			)
		{
			using namespace mrpt;
			using namespace mrpt::utils;
			using namespace mrpt::math;
			using namespace std;

			MRPT_START

			VECTORTYPE &x=out_optimal_x; // Var rename

			// Asserts:
			ASSERT_( increments.size() == x0.size() );

			x=x0;									// Start with the starting point
			VECTORTYPE	f_x;					// The vector error from the user function
			CMatrixTemplateNumeric<NUMTYPE> AUX;
			CMatrixTemplateNumeric<NUMTYPE> J;		// The Jacobian of "f"
			CMatrixTemplateNumeric<NUMTYPE>	H;		// The Hessian of "f"
			VECTORTYPE	g;						// The gradient

			// Compute the jacobian and the Hessian:
			mrpt::math::estimateJacobian( x, functor, increments, userParam, J);
			H.multiply_AtA(J);

			const size_t  H_len = H.getColCount();

			// Compute the gradient:
			functor(x, userParam ,f_x);
			J.multiply_Atb(f_x, g);

			// Start iterations:
			bool	found = math::norm_inf(g)<=e1;
			if (verbose && found)	printf_debug("[LM] End condition: math::norm_inf(g)<=e1 :%f\n",math::norm_inf(g));

			NUMTYPE	lambda = tau * H.maximumDiagonal();
			size_t  iter = 0;
			NUMTYPE	v = 2;

			VECTORTYPE	h_lm;
			VECTORTYPE	xnew, f_xnew ;
			NUMTYPE			F_x  = pow( math::norm( f_x ), 2);

			const size_t	N = x.size();

			if (returnPath)	{
				out_info.path.setSize(maxIter,N+1);
				out_info.path.block(iter,0,1,N) = x.transpose();
			}	else out_info.path = Eigen::Matrix<NUMTYPE,Eigen::Dynamic,Eigen::Dynamic>(); // Empty matrix

			while (!found && ++iter<maxIter)
			{
				// H_lm = -( H + \lambda I ) ^-1 * g
				for (size_t k=0;k<H_len;k++)
					H(k,k)+= lambda;
					//H(k,k) *= 1+lambda;

				H.inv_fast(AUX);
				AUX.multiply_Ab(g,h_lm);
				h_lm *= NUMTYPE(-1.0);

				double h_lm_n2 = math::norm(h_lm);
				double x_n2 = math::norm(x);

				if (verbose) printf_debug( (format("[LM] Iter: %u x:",(unsigned)iter)+ sprintf_vector(" %f",x) + string("\n")).c_str() );

				if (h_lm_n2<e2*(x_n2+e2))
				{
					// Done:
					found = true;
					if (verbose) printf_debug("[LM] End condition: %e < %e\n", h_lm_n2, e2*(x_n2+e2) );
				}
				else
				{
					// Improvement: xnew = x + h_lm;
					if (!x_increment_adder)
							xnew = x + h_lm; // Normal Euclidean space addition.
					else 	x_increment_adder(xnew, x, h_lm, userParam);

					functor(xnew, userParam ,f_xnew );
					const double F_xnew = pow( math::norm(f_xnew), 2);

					// denom = h_lm^t * ( \lambda * h_lm - g )
					VECTORTYPE	tmp(h_lm);
					tmp *= lambda;
					tmp -= g;
					tmp.array() *=h_lm.array();
					double denom = tmp.sum();
					double l = (F_x - F_xnew) / denom;

					if (l>0) // There is an improvement:
					{
						// Accept new point:
						x   = xnew;
						f_x = f_xnew;
						F_x = F_xnew;

						math::estimateJacobian( x, functor, increments, userParam, J);
						H.multiply_AtA(J);
						J.multiply_Atb(f_x, g);

						found = math::norm_inf(g)<=e1;
						if (verbose && found)	printf_debug("[LM] End condition: math::norm_inf(g)<=e1 : %e\n", math::norm_inf(g) );

						lambda *= max(0.33, 1-pow(2*l-1,3) );
						v = 2;
					}
					else
					{
						// Nope...
						lambda *= v;
						v*= 2;
					}


					if (returnPath)	{
						out_info.path.block(iter,0,1,x.size()) = x.transpose();
						out_info.path(iter,x.size()) = F_x;
					}
				}
			} // end while

			// Output info:
			out_info.final_sqr_err = F_x;
			out_info.iterations_executed = iter;
			out_info.last_err_vector = f_x;
			if (returnPath) out_info.path.setSize(iter,N+1);

			MRPT_END
		}

	}; // End of class def.


	typedef CLevenbergMarquardtTempl<vector_double> CLevenbergMarquardt;  //!< The default name for the LM class is an instantiation for "double"

	} // End of namespace
} // End of namespace
#endif
