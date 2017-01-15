/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_DATA_UTILS_MATH_H
#define  MRPT_DATA_UTILS_MATH_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/ops_matrices.h>

namespace mrpt
{
	/** This base provides a set of functions for maths stuff. \ingroup mrpt_base_grp
	 */
	namespace math
	{
/** \addtogroup stats_grp
		  * @{
		  */

		/** @name Probability density distributions (pdf) distance metrics
		@{ */

		/** Computes the squared mahalanobis distance of a vector X given the mean MU and the covariance *inverse* COV_inv
		  *  \f[ d^2 =  (X-MU)^\top \Sigma^{-1} (X-MU)  \f]
		  */
		template<class VECTORLIKE1,class VECTORLIKE2,class MAT>
		typename MAT::Scalar mahalanobisDistance2(
			const VECTORLIKE1 &X,
			const VECTORLIKE2 &MU,
			const MAT &COV )
		{
			MRPT_START
			#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
				ASSERT_( !X.empty() );
				ASSERT_( X.size()==MU.size() );
				ASSERT_( X.size()==size(COV,1) && COV.isSquare() );
			#endif
			const size_t N = X.size();
			Eigen::Matrix<typename MAT::Scalar,Eigen::Dynamic,1> X_MU(N);
			for (size_t i=0;i<N;i++) X_MU[i]=X[i]-MU[i];
			const Eigen::Matrix<typename MAT::Scalar,Eigen::Dynamic,1>  z = COV.llt().solve(X_MU);
			return z.dot(z);
			MRPT_END
		}


		/** Computes the mahalanobis distance of a vector X given the mean MU and the covariance *inverse* COV_inv
		  *  \f[ d = \sqrt{ (X-MU)^\top \Sigma^{-1} (X-MU) }  \f]
		  */
		template<class VECTORLIKE1,class VECTORLIKE2,class MAT>
		inline typename VECTORLIKE1::Scalar mahalanobisDistance(
			const VECTORLIKE1 &X,
			const VECTORLIKE2 &MU,
			const MAT &COV )
		{
			return std::sqrt( mahalanobisDistance2(X,MU,COV) );
		}


		/** Computes the squared mahalanobis distance between two *non-independent* Gaussians, given the two covariance matrices and the vector with the difference of their means.
		  *  \f[ d^2 = \Delta_\mu^\top (\Sigma_1 + \Sigma_2 - 2 \Sigma_12 )^{-1} \Delta_\mu  \f]
		  */
		template<class VECTORLIKE,class MAT1,class MAT2,class MAT3>
		typename MAT1::Scalar
		mahalanobisDistance2(
			const VECTORLIKE &mean_diffs,
			const MAT1 &COV1,
			const MAT2 &COV2,
			const MAT3 &CROSS_COV12 )
		{
			MRPT_START
			#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
				ASSERT_( !mean_diffs.empty() );
				ASSERT_( mean_diffs.size()==size(COV1,1));
				ASSERT_( COV1.isSquare() && COV2.isSquare() );
				ASSERT_( size(COV1,1)==size(COV2,1));
			#endif
			const size_t N = size(COV1,1);
			MAT1 COV = COV1;
			COV+=COV2;
			COV.substract_An(CROSS_COV12,2);
			MAT1 COV_inv;
			COV.inv_fast(COV_inv);
			return multiply_HCHt_scalar(mean_diffs,COV_inv);
			MRPT_END
		}

		/** Computes the mahalanobis distance between two *non-independent* Gaussians (or independent if CROSS_COV12=NULL), given the two covariance matrices and the vector with the difference of their means.
		  *  \f[ d = \sqrt{ \Delta_\mu^\top (\Sigma_1 + \Sigma_2 - 2 \Sigma_12 )^{-1} \Delta_\mu } \f]
		  */
		template<class VECTORLIKE,class MAT1,class MAT2,class MAT3> inline typename VECTORLIKE::Scalar
		mahalanobisDistance(
			const VECTORLIKE &mean_diffs,
			const MAT1 &COV1,
			const MAT2 &COV2,
			const MAT3 &CROSS_COV12 )
		{
			return std::sqrt( mahalanobisDistance( mean_diffs, COV1,COV2,CROSS_COV12 ));
		}

		/** Computes the squared mahalanobis distance between a point and a Gaussian, given the covariance matrix and the vector with the difference between the mean and the point.
		  *  \f[ d^2 = \Delta_\mu^\top \Sigma^{-1} \Delta_\mu  \f]
		  */
		template<class VECTORLIKE,class MATRIXLIKE>
		inline typename MATRIXLIKE::Scalar
		mahalanobisDistance2(const VECTORLIKE &delta_mu,const MATRIXLIKE &cov)
		{
			ASSERTDEB_(cov.isSquare())
			ASSERTDEB_(size_t(cov.getColCount())==size_t(delta_mu.size()))
			return multiply_HCHt_scalar(delta_mu,cov.inverse());
		}

		/** Computes the mahalanobis distance between a point and a Gaussian, given the covariance matrix and the vector with the difference between the mean and the point.
		  *  \f[ d^2 = \sqrt( \Delta_\mu^\top \Sigma^{-1} \Delta_\mu ) \f]
		  */
		template<class VECTORLIKE,class MATRIXLIKE>
		inline typename MATRIXLIKE::Scalar
		mahalanobisDistance(const VECTORLIKE &delta_mu,const MATRIXLIKE &cov)
		{
			return std::sqrt(mahalanobisDistance2(delta_mu,cov));
		}

		/** Computes the integral of the product of two Gaussians, with means separated by "mean_diffs" and covariances "COV1" and "COV2".
		  *  \f[ D = \frac{1}{(2 \pi)^{0.5 N} \sqrt{}  }  \exp( \Delta_\mu^\top (\Sigma_1 + \Sigma_2 - 2 \Sigma_12)^{-1} \Delta_\mu)  \f]
		  */
		template <typename T>
		T productIntegralTwoGaussians(
			const std::vector<T> &mean_diffs,
			const CMatrixTemplateNumeric<T> &COV1,
			const CMatrixTemplateNumeric<T> &COV2
			)
		{
			const size_t vector_dim = mean_diffs.size();
			ASSERT_(vector_dim>=1)

			CMatrixTemplateNumeric<T> C = COV1;
			C+= COV2;	// Sum of covs:
			const T cov_det = C.det();
			CMatrixTemplateNumeric<T> C_inv;
			C.inv_fast(C_inv);

			return std::pow( M_2PI, -0.5*vector_dim ) * (1.0/std::sqrt( cov_det ))
				* exp( -0.5 * mean_diffs.multiply_HCHt_scalar(C_inv) );
		}

		/** Computes the integral of the product of two Gaussians, with means separated by "mean_diffs" and covariances "COV1" and "COV2".
		  *  \f[ D = \frac{1}{(2 \pi)^{0.5 N} \sqrt{}  }  \exp( \Delta_\mu^\top (\Sigma_1 + \Sigma_2)^{-1} \Delta_\mu)  \f]
		  */
		template <typename T, size_t DIM>
		T productIntegralTwoGaussians(
			const std::vector<T> &mean_diffs,
			const CMatrixFixedNumeric<T,DIM,DIM> &COV1,
			const CMatrixFixedNumeric<T,DIM,DIM> &COV2
			)
		{
			ASSERT_(mean_diffs.size()==DIM);

			CMatrixFixedNumeric<T,DIM,DIM> C = COV1;
			C+= COV2;	// Sum of covs:
			const T cov_det = C.det();
			CMatrixFixedNumeric<T,DIM,DIM> C_inv(mrpt::math::UNINITIALIZED_MATRIX);
			C.inv_fast(C_inv);

			return std::pow( M_2PI, -0.5*DIM ) * (1.0/std::sqrt( cov_det ))
				* exp( -0.5 * mean_diffs.multiply_HCHt_scalar(C_inv) );
		}

		/** Computes both, the integral of the product of two Gaussians and their square Mahalanobis distance.
		  * \sa productIntegralTwoGaussians, mahalanobisDistance2
		  */
		template <typename T, class VECLIKE,class MATLIKE1, class MATLIKE2>
		void productIntegralAndMahalanobisTwoGaussians(
			const VECLIKE 	&mean_diffs,
			const MATLIKE1 	&COV1,
			const MATLIKE2 	&COV2,
			T 				&maha2_out,
			T 				&intprod_out,
			const MATLIKE1	*CROSS_COV12=NULL
			)
		{
			const size_t vector_dim = mean_diffs.size();
			ASSERT_(vector_dim>=1)

			MATLIKE1 C = COV1;
			C+= COV2;	// Sum of covs:
			if (CROSS_COV12) { C-=*CROSS_COV12; C-=*CROSS_COV12; }
			const T cov_det = C.det();
			MATLIKE1 C_inv;
			C.inv_fast(C_inv);

			maha2_out = mean_diffs.multiply_HCHt_scalar(C_inv);
			intprod_out = std::pow( M_2PI, -0.5*vector_dim ) * (1.0/std::sqrt( cov_det ))*exp(-0.5*maha2_out);
		}

		/** Computes both, the logarithm of the PDF and the square Mahalanobis distance between a point (given by its difference wrt the mean) and a Gaussian.
		  * \sa productIntegralTwoGaussians, mahalanobisDistance2, normalPDF, mahalanobisDistance2AndPDF
		  */
		template <typename T, class VECLIKE,class MATRIXLIKE>
		void mahalanobisDistance2AndLogPDF(
			const VECLIKE 		&diff_mean,
			const MATRIXLIKE	&cov,
			T 					&maha2_out,
			T 					&log_pdf_out)
		{
			MRPT_START
			ASSERTDEB_(cov.isSquare())
			ASSERTDEB_(size_t(cov.getColCount())==size_t(diff_mean.size()))
			MATRIXLIKE C_inv;
			cov.inv(C_inv);
			maha2_out = multiply_HCHt_scalar(diff_mean,C_inv);
			log_pdf_out = static_cast<typename MATRIXLIKE::Scalar>(-0.5)* (
				maha2_out+
				static_cast<typename MATRIXLIKE::Scalar>(cov.getColCount())*::log(static_cast<typename MATRIXLIKE::Scalar>(M_2PI))+
				::log(cov.det())
				);
			MRPT_END
		}

		/** Computes both, the PDF and the square Mahalanobis distance between a point (given by its difference wrt the mean) and a Gaussian.
		  * \sa productIntegralTwoGaussians, mahalanobisDistance2, normalPDF
		  */
		template <typename T, class VECLIKE,class MATRIXLIKE>
		inline void mahalanobisDistance2AndPDF(
			const VECLIKE 		&diff_mean,
			const MATRIXLIKE	&cov,
			T 					&maha2_out,
			T 					&pdf_out)
		{
			mahalanobisDistance2AndLogPDF(diff_mean,cov,maha2_out,pdf_out);
			pdf_out = std::exp(pdf_out); // log to linear
		}


		/** Computes covariances and mean of any vector of containers, given optional weights for the different samples.
		  * \param elements Any kind of vector of vectors/arrays, eg. std::vector<mrpt::math::CVectorDouble>, with all the input samples, each sample in a "row".
		  * \param covariances Output estimated covariance; it can be a fixed/dynamic matrix or a matrixview.
		  * \param means Output estimated mean; it can be CVectorDouble/CArrayDouble, etc...
		  * \param weights_mean If !=NULL, it must point to a vector of size()==number of elements, with normalized weights to take into account for the mean.
		  * \param weights_cov If !=NULL, it must point to a vector of size()==number of elements, with normalized weights to take into account for the covariance.
		  * \param elem_do_wrap2pi If !=NULL; it must point to an array of "bool" of size()==dimension of each element, stating if it's needed to do a wrap to [-pi,pi] to each dimension.
		  * \sa This method is used in mrpt::math::unscented_transform_gaussian
		  * \ingroup stats_grp
		  */
		template<class VECTOR_OF_VECTORS, class MATRIXLIKE,class VECTORLIKE,class VECTORLIKE2,class VECTORLIKE3>
		inline void covariancesAndMeanWeighted(   // Done inline to speed-up the special case expanded in covariancesAndMean() below.
			const VECTOR_OF_VECTORS &elements,
			MATRIXLIKE &covariances,
			VECTORLIKE &means,
			const VECTORLIKE2 *weights_mean,
			const VECTORLIKE3 *weights_cov,
			const bool *elem_do_wrap2pi = NULL
			)
		{
			ASSERTMSG_(elements.size()!=0,"No samples provided, so there is no way to deduce the output size.")
			typedef typename MATRIXLIKE::Scalar T;
			const size_t DIM = elements[0].size();
			means.resize(DIM);
			covariances.setSize(DIM,DIM);
			const size_t nElms=elements.size();
			const T NORM=1.0/nElms;
			if (weights_mean) { ASSERTDEB_(size_t(weights_mean->size())==size_t(nElms)) }
			// The mean goes first:
			for (size_t i=0;i<DIM;i++)
			{
				T  accum = 0;
				if (!elem_do_wrap2pi || !elem_do_wrap2pi[i])
				{	// i'th dimension is a "normal", real number:
					if (weights_mean)
					{
						for (size_t j=0;j<nElms;j++)
							accum+= (*weights_mean)[j] * elements[j][i];
					}
					else
					{
						for (size_t j=0;j<nElms;j++) accum+=elements[j][i];
						accum*=NORM;
					}
				}
				else
				{	// i'th dimension is a circle in [-pi,pi]: we need a little trick here:
					double accum_L=0,accum_R=0;
					double Waccum_L=0,Waccum_R=0;
					for (size_t j=0;j<nElms;j++)
					{
						double ang = elements[j][i];
						const double w   = weights_mean!=NULL ? (*weights_mean)[j] : NORM;
						if (fabs( ang )>0.5*M_PI)
						{	// LEFT HALF: 0,2pi
							if (ang<0) ang = (M_2PI + ang);
							accum_L  += ang * w;
							Waccum_L += w;
						}
						else
						{	// RIGHT HALF: -pi,pi
							accum_R += ang * w;
							Waccum_R += w;
						}
					}
					if (Waccum_L>0)	accum_L /= Waccum_L;  // [0,2pi]
					if (Waccum_R>0)	accum_R /= Waccum_R;  // [-pi,pi]
					if (accum_L>M_PI) accum_L -= M_2PI;	// Left side to [-pi,pi] again:
					accum = (accum_L* Waccum_L + accum_R * Waccum_R );	// The overall result:
				}
				means[i]=accum;
			}
			// Now the covariance:
			for (size_t i=0;i<DIM;i++)
				for (size_t j=0;j<=i;j++)	// Only 1/2 of the matrix
				{
					typename MATRIXLIKE::Scalar elem=0;
					if (weights_cov)
					{
						ASSERTDEB_(size_t(weights_cov->size())==size_t(nElms))
						for (size_t k=0;k<nElms;k++)
						{
							const T Ai = (elements[k][i]-means[i]);
							const T Aj = (elements[k][j]-means[j]);
							if (!elem_do_wrap2pi || !elem_do_wrap2pi[i])
									elem+= (*weights_cov)[k] * Ai * Aj;
							else	elem+= (*weights_cov)[k] * mrpt::math::wrapToPi(Ai) * mrpt::math::wrapToPi(Aj);
						}
					}
					else
					{
						for (size_t k=0;k<nElms;k++)
						{
							const T Ai = (elements[k][i]-means[i]);
							const T Aj = (elements[k][j]-means[j]);
							if (!elem_do_wrap2pi || !elem_do_wrap2pi[i])
									elem+= Ai * Aj;
							else	elem+= mrpt::math::wrapToPi(Ai) * mrpt::math::wrapToPi(Aj);
						}
						elem*=NORM;
					}
					covariances.get_unsafe(i,j) = elem;
					if (i!=j) covariances.get_unsafe(j,i)=elem;
				}
		}

		/** Computes covariances and mean of any vector of containers.
		  * \param elements Any kind of vector of vectors/arrays, eg. std::vector<mrpt::math::CVectorDouble>, with all the input samples, each sample in a "row".
		  * \param covariances Output estimated covariance; it can be a fixed/dynamic matrix or a matrixview.
		  * \param means Output estimated mean; it can be CVectorDouble/CArrayDouble, etc...
		  * \param elem_do_wrap2pi If !=NULL; it must point to an array of "bool" of size()==dimension of each element, stating if it's needed to do a wrap to [-pi,pi] to each dimension.
		  * \ingroup stats_grp
		  */
		template<class VECTOR_OF_VECTORS, class MATRIXLIKE,class VECTORLIKE>
		void covariancesAndMean(const VECTOR_OF_VECTORS &elements,MATRIXLIKE &covariances,VECTORLIKE &means, const bool *elem_do_wrap2pi = NULL)
		{   // The function below is inline-expanded here:
			covariancesAndMeanWeighted<VECTOR_OF_VECTORS,MATRIXLIKE,VECTORLIKE,CVectorDouble,CVectorDouble>(elements,covariances,means,NULL,NULL,elem_do_wrap2pi);
		}


		/** Computes the weighted histogram for a vector of values and their corresponding weights.
		  *  \param values [IN] The N values
		  *  \param weights [IN] The weights for the corresponding N values (don't need to be normalized)
		  *  \param binWidth [IN] The desired width of the bins
		  *  \param out_binCenters [OUT] The centers of the M bins generated to cover from the minimum to the maximum value of "values" with the given "binWidth"
		  *  \param out_binValues [OUT] The ratio of values at each given bin, such as the whole vector sums up the unity.
		  *  \sa weightedHistogramLog
		  */
		template<class VECTORLIKE1,class VECTORLIKE2>
			void  weightedHistogram(
				const VECTORLIKE1	&values,
				const VECTORLIKE1	&weights,
				float				binWidth,
				VECTORLIKE2	&out_binCenters,
				VECTORLIKE2	&out_binValues )
			{
				MRPT_START
				typedef typename mrpt::math::ContainerType<VECTORLIKE1>::element_t TNum;

				ASSERT_( values.size() == weights.size() );
				ASSERT_( binWidth > 0 );
				TNum	minBin = minimum( values );
				unsigned int	nBins = static_cast<unsigned>(ceil((maximum( values )-minBin) / binWidth));

				// Generate bin center and border values:
				out_binCenters.resize(nBins);
				out_binValues.clear(); out_binValues.resize(nBins,0);
				TNum halfBin = TNum(0.5)*binWidth;;
				VECTORLIKE2   binBorders(nBins+1,minBin-halfBin);
				for (unsigned int i=0;i<nBins;i++)
				{
					binBorders[i+1] = binBorders[i]+binWidth;
					out_binCenters[i] = binBorders[i]+halfBin;
				}

				// Compute the histogram:
				TNum totalSum = 0;
				typename VECTORLIKE1::const_iterator itVal, itW;
				for (itVal = values.begin(), itW = weights.begin(); itVal!=values.end(); ++itVal, ++itW )
				{
					int idx = round(((*itVal)-minBin)/binWidth);
					if (idx>=int(nBins)) idx=nBins-1;
					ASSERTDEB_(idx>=0);
					out_binValues[idx] += *itW;
					totalSum+= *itW;
				}

				if (totalSum)
					out_binValues /= totalSum;


				MRPT_END
			}

		/** Computes the weighted histogram for a vector of values and their corresponding log-weights.
		  *  \param values [IN] The N values
		  *  \param weights [IN] The log-weights for the corresponding N values (don't need to be normalized)
		  *  \param binWidth [IN] The desired width of the bins
		  *  \param out_binCenters [OUT] The centers of the M bins generated to cover from the minimum to the maximum value of "values" with the given "binWidth"
		  *  \param out_binValues [OUT] The ratio of values at each given bin, such as the whole vector sums up the unity.
		  *  \sa weightedHistogram
		  */
		template<class VECTORLIKE1,class VECTORLIKE2>
		void  weightedHistogramLog(
			const VECTORLIKE1	&values,
			const VECTORLIKE1	&log_weights,
			float				binWidth,
			VECTORLIKE2	&out_binCenters,
			VECTORLIKE2	&out_binValues )
		{
			MRPT_START
			typedef typename mrpt::math::ContainerType<VECTORLIKE1>::element_t TNum;

			ASSERT_( values.size() == log_weights.size() );
			ASSERT_( binWidth > 0 );
			TNum	minBin = minimum( values );
			unsigned int	nBins = static_cast<unsigned>(ceil((maximum( values )-minBin) / binWidth));

			// Generate bin center and border values:
			out_binCenters.resize(nBins);
			out_binValues.clear(); out_binValues.resize(nBins,0);
			TNum halfBin = TNum(0.5)*binWidth;;
			VECTORLIKE2   binBorders(nBins+1,minBin-halfBin);
			for (unsigned int i=0;i<nBins;i++)
			{
				binBorders[i+1] = binBorders[i]+binWidth;
				out_binCenters[i] = binBorders[i]+halfBin;
			}

			// Compute the histogram:
			const TNum max_log_weight = maximum(log_weights);
			TNum totalSum = 0;
			typename VECTORLIKE1::const_iterator itVal, itW;
			for (itVal = values.begin(), itW = log_weights.begin(); itVal!=values.end(); ++itVal, ++itW )
			{
				int idx = round(((*itVal)-minBin)/binWidth);
				if (idx>=int(nBins)) idx=nBins-1;
				ASSERTDEB_(idx>=0);
				const TNum w = exp(*itW-max_log_weight);
				out_binValues[idx] += w;
				totalSum+= w;
			}

			if (totalSum)
				out_binValues /= totalSum;

			MRPT_END
		}

		/** A numerically-stable method to compute average likelihood values with strongly different ranges (unweighted likelihoods: compute the arithmetic mean).
		  *  This method implements this equation:
		  *
		  *  \f[ return = - \log N + \log  \sum_{i=1}^N e^{ll_i-ll_{max}} + ll_{max} \f]
		  *
		  * See also the <a href="http://www.mrpt.org/Averaging_Log-Likelihood_Values:Numerical_Stability">tutorial page</a>.
		  * \ingroup stats_grp
		  */
		double BASE_IMPEXP averageLogLikelihood( const CVectorDouble &logLikelihoods );

		/** Computes the average of a sequence of angles in radians taking into account the correct wrapping in the range \f$ ]-\pi,\pi [ \f$, for example, the mean of (2,-2) is \f$ \pi \f$, not 0.
		  * \ingroup stats_grp
		  */
		double BASE_IMPEXP averageWrap2Pi(const CVectorDouble &angles );

		/** A numerically-stable method to average likelihood values with strongly different ranges (weighted likelihoods).
		  *  This method implements this equation:
		  *
		  *  \f[ return = \log \left( \frac{1}{\sum_i e^{lw_i}} \sum_i  e^{lw_i} e^{ll_i}  \right) \f]
		  *
		  * See also the <a href="http://www.mrpt.org/Averaging_Log-Likelihood_Values:Numerical_Stability">tutorial page</a>.
		  * \ingroup stats_grp
		  */
		double BASE_IMPEXP  averageLogLikelihood(
			const CVectorDouble &logWeights,
			const CVectorDouble &logLikelihoods );

		/**  @} */  // end of grouping container_ops_grp

	} // End of MATH namespace
} // End of namespace

#endif
