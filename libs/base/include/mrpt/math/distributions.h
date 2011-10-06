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
#ifndef  mrpt_math_distributions_H
#define  mrpt_math_distributions_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>

#include <mrpt/math/ops_matrices.h>

/*---------------------------------------------------------------
		Namespace
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace math
	{
		using namespace mrpt::utils;

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
		inline typename MATRIXLIKE::value_type
			normalPDFInf(
				const VECTORLIKE1  & x,
				const VECTORLIKE2  & mu,
				const MATRIXLIKE   & cov_inv,
				const bool scaled_pdf = false )
		{
			MRPT_START
			typedef typename MATRIXLIKE::value_type T;
			ASSERTDEB_(cov_inv.isSquare())
			ASSERTDEB_(size_t(cov_inv.getColCount())==size_t(x.size()) && size_t(cov_inv.getColCount())==size_t(mu.size()))
			T ret = ::exp( static_cast<T>(-0.5) * mrpt::math::multiply_HCHt_scalar((x-mu), cov_inv ) );
			return scaled_pdf ? ret : ret * ::sqrt(cov_inv.det()) / ::pow(static_cast<T>(M_2PI),static_cast<T>( size(cov_inv,1) ));
			MRPT_END
		}

		/** Evaluates the multivariate normal (Gaussian) distribution at a given point "x".
		  *  \param  x   A vector or column or row matrix with the point at which to evaluate the pdf.
		  *  \param  mu  A vector or column or row matrix with the Gaussian mean.
		  *  \param  cov  The covariance matrix of the Gaussian.
		  *  \param  scaled_pdf If set to true, the PDF will be scaled to be in the range [0,1], in contrast to its integral from [-inf,+inf] being 1.
		  */
		template <class VECTORLIKE1,class VECTORLIKE2,class MATRIXLIKE>
		inline typename MATRIXLIKE::value_type
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
		typename MATRIXLIKE::value_type
		normalPDF(const VECTORLIKE &d,const MATRIXLIKE &cov)
		{
			MRPT_START
			ASSERTDEB_(cov.isSquare())
			ASSERTDEB_(size_t(cov.getColCount())==size_t(d.size()))
			return std::exp( static_cast<typename MATRIXLIKE::value_type>(-0.5)*mrpt::math::multiply_HCHt_scalar(d,cov.inverse()))
			/ (::pow(
					static_cast<typename MATRIXLIKE::value_type>(M_2PI),
					static_cast<typename MATRIXLIKE::value_type>(cov.getColCount()))
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
#ifdef HAVE_ERF
		inline double erfc(double x) { return ::erfc(x); }
#else
		double BASE_IMPEXP  erfc(double x);
#endif

		/** The error function of a Normal distribution
		  */
#ifdef HAVE_ERF
		inline double erf(double x) { return ::erf(x); }
#else
		double BASE_IMPEXP   erf(double x);
#endif
		/** Evaluates the Gaussian distribution quantile for the probability value p=[0,1].
		  *  The employed approximation is that from Peter J. Acklam (pjacklam@online.no),
		  *  freely available in http://home.online.no/~pjacklam.
		  */
		double BASE_IMPEXP  normalQuantile(double p);

		/** Evaluates the Gaussian cumulative density function.
		  *  The employed approximation is that from W. J. Cody
		  *  freely available in http://www.netlib.org/specfun/erf
		  */
		double  BASE_IMPEXP normalCDF(double p);

		/** The "quantile" of the Chi-Square distribution, for dimension "dim" and probability 0<P<1 (the inverse of chi2CDF)
		  * An aproximation from the Wilson-Hilferty transformation is used.
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
		*/
		template <class T>
		double noncentralChi2CDF(unsigned int degreesOfFreedom, T noncentrality, T arg)
		{
			const double a = degreesOfFreedom + noncentrality;
			const double b = (a + noncentrality) / square(a);
			const double t = (std::pow((double)arg / a, 1.0/3.0) - (1.0 - 2.0 / 9.0 * b)) / std::sqrt(2.0 / 9.0 * b);
			return 0.5*(1.0 + mrpt::math::erf(t/std::sqrt(2.0)));
		}

		/*! Cumulative chi square distribution.

			Computes the cumulative density of a chi square distribution with \a degreesOfFreedom
			and tolerance \a accuracy at the given argument \a arg, i.e. the probability that
			a random number drawn from the distribution is below \a arg
			by calling <tt>noncentralChi2CDF(degreesOfFreedom, 0.0, arg, accuracy)</tt>.

			\note Function code from the Vigra project (http://hci.iwr.uni-heidelberg.de/vigra/); code under "MIT X11 License", GNU GPL-compatible.
		*/
		inline double chi2CDF(unsigned int degreesOfFreedom, double arg)
		{
			return noncentralChi2CDF(degreesOfFreedom, 0.0, arg);
		}

		namespace detail
		{
			template <class T>
			void noncentralChi2OneIteration(T arg, T & lans, T & dans, T & pans, unsigned int & j)
			{
				double tol = -50.0;
				if(lans < tol)
				{
					lans = lans + std::log(arg / j);
					dans = std::exp(lans);
				}
				else
				{
					dans = dans * arg / j;
				}
				pans = pans - dans;
				j += 2;
			}

			template <class T>
			std::pair<double, double> noncentralChi2CDF_exact(unsigned int degreesOfFreedom, T noncentrality, T arg, T eps)
			{
				ASSERTMSG_(noncentrality >= 0.0 && arg >= 0.0 && eps > 0.0,"noncentralChi2P(): parameters must be positive.");
				if (arg == 0.0 && degreesOfFreedom > 0)
					return std::make_pair(0.0, 0.0);

				// Determine initial values
				double b1 = 0.5 * noncentrality,
					   ao = std::exp(-b1),
					   eps2 = eps / ao,
					   lnrtpi2 = 0.22579135264473,
					   probability, density, lans, dans, pans, sum, am, hold;
				unsigned int maxit = 500,
					i, m;
				if(degreesOfFreedom % 2)
				{
					i = 1;
					lans = -0.5 * (arg + std::log(arg)) - lnrtpi2;
					dans = std::exp(lans);
					pans = erf(std::sqrt(arg/2.0));
				}
				else
				{
					i = 2;
					lans = -0.5 * arg;
					dans = std::exp(lans);
					pans = 1.0 - dans;
				}

				// Evaluate first term
				if(degreesOfFreedom == 0)
				{
					m = 1;
					degreesOfFreedom = 2;
					am = b1;
					sum = 1.0 / ao - 1.0 - am;
					density = am * dans;
					probability = 1.0 + am * pans;
				}
				else
				{
					m = 0;
					degreesOfFreedom = degreesOfFreedom - 1;
					am = 1.0;
					sum = 1.0 / ao - 1.0;
					while(i < degreesOfFreedom)
						detail::noncentralChi2OneIteration(arg, lans, dans, pans, i);
					degreesOfFreedom = degreesOfFreedom + 1;
					density = dans;
					probability = pans;
				}
				// Evaluate successive terms of the expansion
				for(++m; m<maxit; ++m)
				{
					am = b1 * am / m;
					detail::noncentralChi2OneIteration(arg, lans, dans, pans, degreesOfFreedom);
					sum = sum - am;
					density = density + am * dans;
					hold = am * pans;
					probability = probability + hold;
					if((pans * sum < eps2) && (hold < eps2))
						break; // converged
				}
				if(m == maxit)
					THROW_EXCEPTION("noncentralChi2P(): no convergence.");
				return std::make_pair(0.5 * ao * density, std::min(1.0, std::max(0.0, ao * probability)));
			}
		} // namespace detail

		/*! Chi square distribution.

			Computes the density of a chi square distribution with \a degreesOfFreedom
			and tolerance \a accuracy at the given argument \a arg
			by calling <tt>noncentralChi2(degreesOfFreedom, 0.0, arg, accuracy)</tt>.

			\note Function code from the Vigra project (http://hci.iwr.uni-heidelberg.de/vigra/); code under "MIT X11 License", GNU GPL-compatible.
		*/
		inline double chi2PDF(unsigned int degreesOfFreedom, double arg, double accuracy = 1e-7)
		{
			return detail::noncentralChi2CDF_exact(degreesOfFreedom, 0.0, arg, accuracy).first;
		}

		/** Return the mean and the 10%-90% confidence points (or with any other confidence value) of a set of samples by building the cummulative CDF of all the elements of the container.
		  *  The container can be any MRPT container (CArray, matrices, vectors).
		  * \param confidenceInterval A number in the range (0,1) such as the confidence interval will be [100*confidenceInterval, 100*(1-confidenceInterval)].
		  */
		template <typename CONTAINER>
		void confidenceIntervals(
			const CONTAINER &data,
			typename CONTAINER::value_type &out_mean,
			typename CONTAINER::value_type &out_lower_conf_interval,
			typename CONTAINER::value_type &out_upper_conf_interval,
			const double confidenceInterval = 0.1,
			const size_t histogramNumBins = 1000 )
		{
			MRPT_START
			ASSERT_(data.size()!=0)  // don't use .empty() here to allow using matrices
			ASSERT_(confidenceInterval>0 && confidenceInterval<1)

			out_mean = mean(data);
			typename CONTAINER::value_type x_min,x_max;
			minimum_maximum(data,x_min,x_max);

			//std::vector<typename CONTAINER::value_type> xs;
			//linspace(x_min,x_max,histogramNumBins, xs);
			const typename CONTAINER::value_type binWidth = (x_max-x_min)/histogramNumBins;

			const vector_double H = mrpt::math::histogram(data,x_min,x_max,histogramNumBins);
			vector_double Hc;
			cumsum(H,Hc); // CDF
			Hc*=1.0/Hc.maximum();

			vector_double::iterator it_low  = std::lower_bound(Hc.begin(),Hc.end(),confidenceInterval);   ASSERT_(it_low!=Hc.end())
			vector_double::iterator it_high = std::upper_bound(Hc.begin(),Hc.end(),1-confidenceInterval); ASSERT_(it_high!=Hc.end())
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
