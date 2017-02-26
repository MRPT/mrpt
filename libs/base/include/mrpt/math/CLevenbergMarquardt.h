/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CLevenbergMarquardt_H
#define  CLevenbergMarquardt_H

#include <mrpt/utils/COutputLogger.h>
#include <mrpt/utils/types_math.h>
#include <mrpt/math/num_jacobian.h>
#include <mrpt/utils/printf_vector.h>
#include <mrpt/math/ops_containers.h>

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
	template <typename VECTORTYPE = Eigen::VectorXd, class USERPARAM = VECTORTYPE >
	class CLevenbergMarquardtTempl : public mrpt::utils::COutputLogger
	{
	public:
		typedef typename VECTORTYPE::Scalar  NUMTYPE;
		typedef Eigen::Matrix<NUMTYPE,Eigen::Dynamic,Eigen::Dynamic>  matrix_t;
		typedef VECTORTYPE vector_t;

		CLevenbergMarquardtTempl() : 
			mrpt::utils::COutputLogger("CLevenbergMarquardt")
		{}

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
			matrix_t	path;	//!< Each row is the optimized value at each iteration.

			/** This matrix can be used to obtain an estimate of the optimal parameters covariance matrix:
			  *  \f[ COV = H M H^\top \f]
			  *  With COV the covariance matrix of the optimal parameters, H this matrix, and M the covariance of the input (observations).
			  */
 			matrix_t H;
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
		void execute(
			VECTORTYPE			&out_optimal_x,
			const VECTORTYPE	&x0,
			TFunctorEval		functor,
			const VECTORTYPE	&increments,
			const USERPARAM		&userParam,
			TResultInfo			&out_info,
			mrpt::utils::VerbosityLevel verbosity = mrpt::utils::LVL_INFO,
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

			this->setMinLoggingLevel(verbosity);

			VECTORTYPE &x=out_optimal_x; // Var rename

			// Asserts:
			ASSERT_( increments.size() == x0.size() );

			x=x0;									// Start with the starting point
			VECTORTYPE	f_x;					// The vector error from the user function
			matrix_t AUX;
			matrix_t J;		// The Jacobian of "f"
			VECTORTYPE	g;						// The gradient

			// Compute the jacobian and the Hessian:
			mrpt::math::estimateJacobian( x, functor, increments, userParam, J);
			out_info.H.multiply_AtA(J);

			const size_t  H_len = out_info.H.getColCount();

			// Compute the gradient:
			functor(x, userParam ,f_x);
			J.multiply_Atb(f_x, g);

			// Start iterations:
			bool	found = math::norm_inf(g)<=e1;
			if (found) logFmt(mrpt::utils::LVL_INFO, "End condition: math::norm_inf(g)<=e1 :%f\n",math::norm_inf(g));

			NUMTYPE	lambda = tau * out_info.H.maximumDiagonal();
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
				matrix_t H = out_info.H;
				for (size_t k=0;k<H_len;k++)
					H(k,k)+= lambda;

				H.inv_fast(AUX);
				AUX.multiply_Ab(g,h_lm);
				h_lm *= NUMTYPE(-1.0);

				double h_lm_n2 = math::norm(h_lm);
				double x_n2 = math::norm(x);

				logFmt(mrpt::utils::LVL_DEBUG, "Iter:%u x=%s\n",(unsigned)iter,sprintf_vector(" %f",x).c_str() );

				if (h_lm_n2<e2*(x_n2+e2))
				{
					// Done:
					found = true;
					logFmt(mrpt::utils::LVL_INFO, "End condition: %e < %e\n", h_lm_n2, e2*(x_n2+e2) );
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
						out_info.H.multiply_AtA(J);
						J.multiply_Atb(f_x, g);

						found = math::norm_inf(g)<=e1;
						if (found) logFmt(mrpt::utils::LVL_INFO, "End condition: math::norm_inf(g)<=e1 : %e\n", math::norm_inf(g) );

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


	typedef CLevenbergMarquardtTempl<mrpt::math::CVectorDouble> CLevenbergMarquardt;  //!< The default name for the LM class is an instantiation for "double"

	} // End of namespace
} // End of namespace
#endif
