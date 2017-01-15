/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  transform_gaussian_H
#define  transform_gaussian_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/math/jacobians.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/random.h>

namespace mrpt
{
	namespace math
	{
		/** @addtogroup  gausspdf_transform_grp Gaussian PDF transformation functions
		  *  \ingroup mrpt_base_grp
		  * @{ */

		/** Scaled unscented transformation (SUT) for estimating the Gaussian distribution of a variable Y=f(X) for an arbitrary function f() provided by the user.
		  *  The user must supply the function in "functor" which takes points in the X space and returns the mapped point in Y, optionally using an extra, constant parameter ("fixed_param") which remains constant.
		  *
		  *  The parameters alpha, K and beta are the common names of the SUT method, and the default values are those recommended in most papers.
		  *
		  * \param elem_do_wrap2pi If !=NULL; it must point to an array of "bool" of size()==dimension of each element, stating if it's needed to do a wrap to [-pi,pi] to each dimension.
		  * \sa The example in MRPT/samples/unscented_transform_test
		  * \sa transform_gaussian_montecarlo, transform_gaussian_linear
		  */
		template <class VECTORLIKE1,class MATLIKE1, class USERPARAM,class VECTORLIKE2,class VECTORLIKE3,class MATLIKE2>
		void transform_gaussian_unscented(
			const VECTORLIKE1 &x_mean,
			const MATLIKE1    &x_cov,
			void  (*functor)(const VECTORLIKE1 &x,const USERPARAM &fixed_param, VECTORLIKE3 &y),
			const USERPARAM &fixed_param,
			VECTORLIKE2 &y_mean,
			MATLIKE2    &y_cov,
			const bool *elem_do_wrap2pi = NULL,
			const double alpha = 1e-3,
			const double K = 0,
			const double beta = 2.0
			)
		{
			MRPT_START
			const size_t Nx = x_mean.size(); // Dimensionality of inputs X
			const double lambda = alpha*alpha*(Nx+K)-Nx;
			const double c = Nx+lambda;

			// Generate weights:
			const double Wi = 0.5/c;
			std::vector<double>  W_mean(1+2*Nx,Wi),W_cov(1+2*Nx,Wi);
			W_mean[0] = lambda/c;
			W_cov[0]  = W_mean[0]+(1-alpha*alpha+beta);

			// Generate X_i samples:
			MATLIKE1 L;
			const bool valid = x_cov.chol(L);
			if (!valid) throw std::runtime_error("transform_gaussian_unscented: Singular covariance matrix in Cholesky.");
			L*= sqrt(c);

			// Propagate the samples X_i -> Y_i:
			// We don't need to store the X sigma points: just use one vector to compute all the Y sigma points:
			typename mrpt::aligned_containers<VECTORLIKE3>::vector_t Y(1+2*Nx); // 2Nx+1 sigma points
			VECTORLIKE1 X = x_mean;
			functor(X,fixed_param,Y[0]);
			VECTORLIKE1 delta; // i'th row of L:
			delta.resize(Nx);
			size_t row=1;
			for (size_t i=0;i<Nx;i++)
			{
				L.extractRowAsCol(i,delta);
				X=x_mean;X-=delta;
				functor(X,fixed_param,Y[row++]);
				X=x_mean;X+=delta;
				functor(X,fixed_param,Y[row++]);
			}

			// Estimate weighted cov & mean:
			mrpt::math::covariancesAndMeanWeighted(Y, y_cov,y_mean,&W_mean,&W_cov,elem_do_wrap2pi);
			MRPT_END
		}

		/** Simple Montecarlo-base estimation of the Gaussian distribution of a variable Y=f(X) for an arbitrary function f() provided by the user.
		  *  The user must supply the function in "functor" which takes points in the X space and returns the mapped point in Y, optionally using an extra, constant parameter ("fixed_param") which remains constant.
		  * \param out_samples_y If !=NULL, this vector will contain, upon return, the sequence of random samples generated and propagated through the functor().
		  * \sa The example in MRPT/samples/unscented_transform_test
		  * \sa transform_gaussian_unscented, transform_gaussian_linear
		  */
		template <class VECTORLIKE1,class MATLIKE1, class USERPARAM,class VECTORLIKE2,class VECTORLIKE3,class MATLIKE2>
		void transform_gaussian_montecarlo(
			const VECTORLIKE1 &x_mean,
			const MATLIKE1    &x_cov,
			void  (*functor)(const VECTORLIKE1 &x,const USERPARAM &fixed_param,VECTORLIKE3 &y),
			const USERPARAM &fixed_param,
			VECTORLIKE2 &y_mean,
			MATLIKE2    &y_cov,
			const size_t  num_samples = 1000,
			typename mrpt::aligned_containers<VECTORLIKE3>::vector_t   *out_samples_y = NULL
			)
		{
			MRPT_START
			typename mrpt::aligned_containers<VECTORLIKE1>::vector_t samples_x;
			mrpt::random::randomGenerator.drawGaussianMultivariateMany(samples_x,num_samples,x_cov,&x_mean);
			typename mrpt::aligned_containers<VECTORLIKE3>::vector_t samples_y(num_samples);
			for (size_t i=0;i<num_samples;i++)
				functor(samples_x[i],fixed_param,samples_y[i]);
			mrpt::math::covariancesAndMean(samples_y,y_cov,y_mean);
			if (out_samples_y) { out_samples_y->clear(); samples_y.swap(*out_samples_y); }
			MRPT_END
		}

		/** First order uncertainty propagation estimator of the Gaussian distribution of a variable Y=f(X) for an arbitrary function f() provided by the user.
		  *  The user must supply the function in "functor" which takes points in the X space and returns the mapped point in Y, optionally using an extra, constant parameter ("fixed_param") which remains constant.
		  *  The Jacobians are estimated numerically using the vector of small increments "x_increments".
		  * \sa The example in MRPT/samples/unscented_transform_test
		  * \sa transform_gaussian_unscented, transform_gaussian_montecarlo
		  */
		template <class VECTORLIKE1,class MATLIKE1, class USERPARAM,class VECTORLIKE2,class VECTORLIKE3,class MATLIKE2>
		void transform_gaussian_linear(
			const VECTORLIKE1 &x_mean,
			const MATLIKE1    &x_cov,
			void  (*functor)(const VECTORLIKE1 &x,const USERPARAM &fixed_param,VECTORLIKE3 &y),
			const USERPARAM &fixed_param,
			VECTORLIKE2 &y_mean,
			MATLIKE2    &y_cov,
			const VECTORLIKE1 &x_increments
			)
		{
			MRPT_START
			// Mean: simple propagation:
			functor(x_mean,fixed_param,y_mean);
			// Cov: COV = H C Ht
			Eigen::Matrix<double,VECTORLIKE3::RowsAtCompileTime,VECTORLIKE1::RowsAtCompileTime> H;
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,functor,x_increments,fixed_param,H);
			H.multiply_HCHt(x_cov, y_cov);
			MRPT_END
		}

		/** @} */

	} // End of MATH namespace

} // End of namespace


#endif
