/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_math_distributions_H
#define  mrpt_math_distributions_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>

#include <mrpt/math/ops_matrices.h>
#include <mrpt/math/ops_vectors.h>

/*---------------------------------------------------------------
		Namespace
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace math
	{
		/** \addtogroup stats_grp Statistics functions, probability distributions
		  *  \ingroup mrpt_base_grp
		  * @{ */

		/** Evaluates the univariate normal (Gaussian) distribution at a given point "x".
		  */
		double BASE_IMPEXP  normalPDF(double x, double mu, double std);

		/** Evaluates the multivariate normal (Gaussian) distribution at a given point "x".
		  *  \param  x   A vector or column or row matrix with the point at which to evaluate the pdf.
		  *  \param  mu  A vector or column or row matrix with the Gaussian mean.
		  *  \param  cov_inv  The inverse covariance (information) matrix of the Gaussian.
		  *  \param  scaled_pdf If set to true, the PDF will be scaled to be in the range [0,1], in contrast to its integral from [-inf,+inf] being 1.
		  */
		template <class VECTORLIKE1,class VECTORLIKE2,class MATRIXLIKE>
		inline typename MATRIXLIKE::Scalar
			normalPDFInf(
				const VECTORLIKE1  & x,
				const VECTORLIKE2  & mu,
				const MATRIXLIKE   & cov_inv,
				const bool scaled_pdf = false )
		{
			MRPT_START
			typedef typename MATRIXLIKE::Scalar T;
			ASSERTDEB_(cov_inv.isSquare())
			ASSERTDEB_(size_t(cov_inv.getColCount())==size_t(x.size()) && size_t(cov_inv.getColCount())==size_t(mu.size()))
			T ret = ::exp( static_cast<T>(-0.5) * mrpt::math::multiply_HCHt_scalar((x-mu).eval(), cov_inv ) );
			return scaled_pdf ? ret : ret * ::sqrt(cov_inv.det() / ::pow(static_cast<T>(M_2PI),static_cast<T>( size(cov_inv,1) )) );
			MRPT_END
		}

		/** Evaluates the multivariate normal (Gaussian) distribution at a given point "x".
		  *  \param  x   A vector or column or row matrix with the point at which to evaluate the pdf.
		  *  \param  mu  A vector or column or row matrix with the Gaussian mean.
		  *  \param  cov  The covariance matrix of the Gaussian.
		  *  \param  scaled_pdf If set to true, the PDF will be scaled to be in the range [0,1], in contrast to its integral from [-inf,+inf] being 1.
		  */
		template <class VECTORLIKE1,class VECTORLIKE2,class MATRIXLIKE>
		inline typename MATRIXLIKE::Scalar
			normalPDF(
				const VECTORLIKE1  & x,
				const VECTORLIKE2  & mu,
				const MATRIXLIKE   & cov,
				const bool scaled_pdf = false )
		{
			return normalPDFInf(x,mu,cov.inverse(),scaled_pdf);
		}

		/** Evaluates the multivariate normal (Gaussian) distribution at a given point given its distance vector "d" from the Gaussian mean.
		  */
		template <typename VECTORLIKE,typename MATRIXLIKE>
		typename MATRIXLIKE::Scalar
		normalPDF(const VECTORLIKE &d,const MATRIXLIKE &cov)
		{
			MRPT_START
			ASSERTDEB_(cov.isSquare())
			ASSERTDEB_(size_t(cov.getColCount())==size_t(d.size()))
			return std::exp( static_cast<typename MATRIXLIKE::Scalar>(-0.5)*mrpt::math::multiply_HCHt_scalar(d,cov.inverse()))
			/ (::pow(
					static_cast<typename MATRIXLIKE::Scalar>(M_2PI),
					static_cast<typename MATRIXLIKE::Scalar>(0.5*cov.getColCount()))
				* ::sqrt(cov.det()));
			MRPT_END
		}

		/** Kullback-Leibler divergence (KLD) between two independent multivariate Gaussians.
		  *
		  * \f$ D_\mathrm{KL}(\mathcal{N}_0 \| \mathcal{N}_1) = { 1 \over 2 } ( \log_e ( { \det \Sigma_1 \over \det \Sigma_0 } ) + \mathrm{tr} ( \Sigma_1^{-1} \Sigma_0 ) + ( \mu_1 - \mu_0 )^\top \Sigma_1^{-1} ( \mu_1 - \mu_0 ) - N ) \f$
		  */
		template <typename VECTORLIKE1,typename MATRIXLIKE1,typename VECTORLIKE2,typename MATRIXLIKE2>
		double KLD_Gaussians(
			const VECTORLIKE1 &mu0, const MATRIXLIKE1 &cov0,
			const VECTORLIKE2 &mu1, const MATRIXLIKE2 &cov1)
		{
			MRPT_START
			ASSERT_(size_t(mu0.size())==size_t(mu1.size()) && size_t(mu0.size())==size_t(size(cov0,1)) && size_t(mu0.size())==size_t(size(cov1,1)) && cov0.isSquare() && cov1.isSquare() )
			const size_t N = mu0.size();
			MATRIXLIKE2 cov1_inv;
			cov1.inv(cov1_inv);
			const VECTORLIKE1 mu_difs = mu0-mu1;
			return 0.5*( log(cov1.det()/cov0.det()) + (cov1_inv*cov0).trace() + multiply_HCHt_scalar(mu_difs,cov1_inv) - N );
			MRPT_END
		}


		/** The complementary error function of a Normal distribution
		  */
		double BASE_IMPEXP  erfc(const double x);

		/** The error function of a Normal distribution
		  */
		double BASE_IMPEXP   erf(const double x);

		/** Evaluates the Gaussian distribution quantile for the probability value p=[0,1].
		  *  The employed approximation is that from Peter J. Acklam (pjacklam@online.no),
		  *  freely available in http://home.online.no/~pjacklam.
		  */
		double BASE_IMPEXP  normalQuantile(double p);

		/** Evaluates the Gaussian cumulative density function.
		  *  The employed approximation is that from W. J. Cody
		  *  freely available in http://www.netlib.org/specfun/erf
		  *  \note Equivalent to MATLAB normcdf(x,mu,s) with p=(x-mu)/s
		  */
		double  BASE_IMPEXP normalCDF(double p);

		/** The "quantile" of the Chi-Square distribution, for dimension "dim" and probability 0<P<1 (the inverse of chi2CDF)
		  * An aproximation from the Wilson-Hilferty transformation is used.
		  *  \note Equivalent to MATLAB chi2inv(), but note that this is just an approximation, which becomes very poor for small values of "P".
		  */
		double  BASE_IMPEXP chi2inv(double P, unsigned int dim=1);

		/*! Cumulative non-central chi square distribution (approximate).

			Computes approximate values of the cumulative density of a chi square distribution with \a degreesOfFreedom,
			and noncentrality parameter \a noncentrality at the given argument
			\a arg, i.e. the probability that a random number drawn from the distribution is below \a arg
			It uses the approximate transform into a normal distribution due to Wilson and Hilferty
			(see Abramovitz, Stegun: "Handbook of Mathematical Functions", formula 26.3.32).
			The algorithm's running time is independent of the inputs. The accuracy is only
			about 0.1 for few degrees of freedom, but reaches about 0.001 above dof = 5.

			\note Function code from the Vigra project (http://hci.iwr.uni-heidelberg.de/vigra/); code under "MIT X11 License", GNU GPL-compatible.
		* \sa noncentralChi2PDF_CDF
		*/
		double BASE_IMPEXP noncentralChi2CDF(unsigned int degreesOfFreedom, double noncentrality, double arg);

		/*! Cumulative chi square distribution.

			Computes the cumulative density of a chi square distribution with \a degreesOfFreedom
			and tolerance \a accuracy at the given argument \a arg, i.e. the probability that
			a random number drawn from the distribution is below \a arg
			by calling <tt>noncentralChi2CDF(degreesOfFreedom, 0.0, arg, accuracy)</tt>.

			\note Function code from the Vigra project (http://hci.iwr.uni-heidelberg.de/vigra/); code under "MIT X11 License", GNU GPL-compatible.
		*/
		double BASE_IMPEXP chi2CDF(unsigned int degreesOfFreedom, double arg);

		/*! Chi square distribution PDF.
		 *	Computes the density of a chi square distribution with \a degreesOfFreedom
		 *	and tolerance \a accuracy at the given argument \a arg
		 *	by calling <tt>noncentralChi2(degreesOfFreedom, 0.0, arg, accuracy)</tt>.
		 *	\note Function code from the Vigra project (http://hci.iwr.uni-heidelberg.de/vigra/); code under "MIT X11 License", GNU GPL-compatible.
		 *
		 * \note Equivalent to MATLAB's chi2pdf(arg,degreesOfFreedom)
		 */
		double BASE_IMPEXP chi2PDF(unsigned int degreesOfFreedom, double arg, double accuracy = 1e-7);

		/** Returns the 'exact' PDF (first) and CDF (second) of a Non-central chi-squared probability distribution, using an iterative method.
		  * \note Equivalent to MATLAB's ncx2cdf(arg,degreesOfFreedom,noncentrality)
		  */
		std::pair<double, double> BASE_IMPEXP noncentralChi2PDF_CDF(unsigned int degreesOfFreedom, double noncentrality, double arg, double eps = 1e-7);

		/** Return the mean and the 10%-90% confidence points (or with any other confidence value) of a set of samples by building the cummulative CDF of all the elements of the container.
		  *  The container can be any MRPT container (CArray, matrices, vectors).
		  * \param confidenceInterval A number in the range (0,1) such as the confidence interval will be [100*confidenceInterval, 100*(1-confidenceInterval)].
		  */
		template <typename CONTAINER>
		void confidenceIntervals(
			const CONTAINER &data,
			typename mrpt::math::ContainerType<CONTAINER>::element_t &out_mean,
			typename mrpt::math::ContainerType<CONTAINER>::element_t &out_lower_conf_interval,
			typename mrpt::math::ContainerType<CONTAINER>::element_t &out_upper_conf_interval,
			const double confidenceInterval = 0.1,
			const size_t histogramNumBins = 1000 )
		{
			MRPT_START
			ASSERT_(data.size()!=0)  // don't use .empty() here to allow using matrices
			ASSERT_(confidenceInterval>0 && confidenceInterval<1)

			out_mean = mean(data);
			typename mrpt::math::ContainerType<CONTAINER>::element_t x_min,x_max;
			minimum_maximum(data,x_min,x_max);

			const typename mrpt::math::ContainerType<CONTAINER>::element_t binWidth = (x_max-x_min)/histogramNumBins;

			const std::vector<double> H = mrpt::math::histogram(data,x_min,x_max,histogramNumBins);
			std::vector<double> Hc;
			cumsum(H,Hc); // CDF
			Hc*=1.0/ mrpt::math::maximum(Hc);

			std::vector<double>::iterator it_low  = std::lower_bound(Hc.begin(),Hc.end(),confidenceInterval);   ASSERT_(it_low!=Hc.end())
			std::vector<double>::iterator it_high = std::upper_bound(Hc.begin(),Hc.end(),1-confidenceInterval); ASSERT_(it_high!=Hc.end())
			const size_t idx_low = std::distance(Hc.begin(),it_low);
			const size_t idx_high = std::distance(Hc.begin(),it_high);
			out_lower_conf_interval = x_min + idx_low * binWidth;
			out_upper_conf_interval = x_min + idx_high * binWidth;

			MRPT_END
		}

		/** @} */

	} // End of MATH namespace

} // End of namespace


#endif
