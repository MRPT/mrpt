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
#ifndef  MRPT_MATH_H
#define  MRPT_MATH_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/CHistogram.h>

#include <mrpt/math/ops_vectors.h>
#include <mrpt/math/ops_matrices.h>

#include <numeric>
#include <cmath>

/*---------------------------------------------------------------
		Namespace
  ---------------------------------------------------------------*/
namespace mrpt
{
	/** This base provides a set of functions for maths stuff. \ingroup mrpt_base_grp
	 */
	namespace math
	{
	    using namespace mrpt::utils;

		/** \addtogroup container_ops_grp
		  * @{ */

		/** Loads one row of a text file as a numerical std::vector.
		  * \return false on EOF or invalid format.
		  * The body of the function is implemented in MATH.cpp
			*/
		bool BASE_IMPEXP loadVector( utils::CFileStream &f, std::vector<int> &d);

		/** Loads one row of a text file as a numerical std::vector.
		  * \return false on EOF or invalid format.
		  * The body of the function is implemented in MATH.cpp
			*/
		bool BASE_IMPEXP loadVector( utils::CFileStream &f, std::vector<double> &d);


        /** Returns true if the number is NaN. */
        bool BASE_IMPEXP  isNaN(float  f) MRPT_NO_THROWS;

        /** Returns true if the number is NaN. */
        bool  BASE_IMPEXP isNaN(double f) MRPT_NO_THROWS;

        /** Returns true if the number is non infinity. */
        bool BASE_IMPEXP  isFinite(float f) MRPT_NO_THROWS;

        /** Returns true if the number is non infinity.  */
        bool  BASE_IMPEXP isFinite(double f) MRPT_NO_THROWS;

        void BASE_IMPEXP medianFilter( const std::vector<double> &inV, std::vector<double> &outV, const int &winSize, const int &numberOfSigmas = 2 );

#ifdef HAVE_LONG_DOUBLE
		/** Returns true if the number is NaN. */
        bool  BASE_IMPEXP isNaN(long double f) MRPT_NO_THROWS;

        /** Returns true if the number is non infinity. */
        bool BASE_IMPEXP  isFinite(long double f) MRPT_NO_THROWS;
#endif

		/** Generates an equidistant sequence of numbers given the first one, the last one and the desired number of points.
		  \sa sequence */
		template<typename T,typename VECTOR>
		void linspace(T first,T last, size_t count, VECTOR &out_vector)
		{
			if (count<2)
			{
				out_vector.assign(count,last);
				return;
			}
			else
			{
				out_vector.resize(count);
				const T incr = (last-first)/T(count-1);
				T c = first;
				for (size_t i=0;i<count;i++,c+=incr)
					out_vector[i] = static_cast<typename VECTOR::value_type>(c);
			}
		}

		/** Generates an equidistant sequence of numbers given the first one, the last one and the desired number of points.
		  \sa sequence */
		template<class T>
		inline Eigen::Matrix<T,Eigen::Dynamic,1> linspace(T first,T last, size_t count)
		{
			Eigen::Matrix<T,Eigen::Dynamic,1> ret;
			mrpt::math::linspace(first,last,count,ret);
			return ret;
		}

		/** Generates a sequence of values [first,first+STEP,first+2*STEP,...]   \sa linspace, sequenceStdVec */
		template<class T,T STEP>
		inline Eigen::Matrix<T,Eigen::Dynamic,1> sequence(T first,size_t length)
		{
			Eigen::Matrix<T,Eigen::Dynamic,1> ret(length);
			if (!length) return ret;
			size_t i=0;
			while (length--) { ret[i++]=first; first+=STEP; }
			return ret;
		}

		/** Generates a sequence of values [first,first+STEP,first+2*STEP,...]   \sa linspace, sequence */
		template<class T,T STEP>
		inline std::vector<T> sequenceStdVec(T first,size_t length)
		{
			std::vector<T> ret(length);
			if (!length) return ret;
			size_t i=0;
			while (length--) { ret[i++]=first; first+=STEP; }
			return ret;
		}

		/** Generates a vector of all ones of the given length. */
		template<class T> inline Eigen::Matrix<T,Eigen::Dynamic,1> ones(size_t count)
		{
			Eigen::Matrix<T,Eigen::Dynamic,1> v(count);
			v.setOnes();
			return v;
		}

		/** Generates a vector of all zeros of the given length. */
		template<class T> inline Eigen::Matrix<T,Eigen::Dynamic,1> zeros(size_t count)
		{
			Eigen::Matrix<T,Eigen::Dynamic,1> v(count);
			v.setZero();
			return v;
		}


		/** Modifies the given angle to translate it into the [0,2pi[ range.
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
		  */
		template <class T>
		inline void wrapTo2PiInPlace(T &a)
		{
			bool was_neg = a<0;
			a = fmod(a, static_cast<T>(M_2PI) );
			if (was_neg) a+=static_cast<T>(M_2PI);
		}

		/** Modifies the given angle to translate it into the [0,2pi[ range.
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
		  */
		template <class T>
		inline T wrapTo2Pi(T a)
		{
			wrapTo2PiInPlace(a);
			return a;
		}

		/** Modifies the given angle to translate it into the ]-pi,pi] range.
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapTo2Pi, wrapToPiInPlace, unwrap2PiSequence
		  */
		template <class T>
		inline T wrapToPi(T a)
		{
			return wrapTo2Pi( a + static_cast<T>(M_PI) )-static_cast<T>(M_PI);
		}

		/** Modifies the given angle to translate it into the ]-pi,pi] range.
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapToPi,wrapTo2Pi, unwrap2PiSequence
		  */
		template <class T>
		inline void wrapToPiInPlace(T &a)
		{
			a = wrapToPi(a);
		}


		/** Normalize a vector, such as its norm is the unity.
		  *  If the vector has a null norm, the output is a null vector.
		  */
		template<class VEC1,class VEC2>
		void normalize(const VEC1 &v, VEC2 &out_v)
		{
			typename VEC1::value_type total=0;
			const size_t N = v.size();
			for (size_t i=0;i<N;i++)
				total += square(v[i]);
			total = std::sqrt(total);
			if (total)
			{
				out_v = v * (1.0/total);
			}
			else out_v.assign(v.size(),0);
		}

		/** Computes covariances and mean of any vector of containers, given optional weights for the different samples.
		  * \param elements Any kind of vector of vectors/arrays, eg. std::vector<vector_double>, with all the input samples, each sample in a "row".
		  * \param covariances Output estimated covariance; it can be a fixed/dynamic matrix or a matrixview.
		  * \param means Output estimated mean; it can be vector_double/CArrayDouble, etc...
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
			typedef typename VECTORLIKE::value_type T;
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
					typename MATRIXLIKE::value_type elem=0;
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
		  * \param elements Any kind of vector of vectors/arrays, eg. std::vector<vector_double>, with all the input samples, each sample in a "row".
		  * \param covariances Output estimated covariance; it can be a fixed/dynamic matrix or a matrixview.
		  * \param means Output estimated mean; it can be vector_double/CArrayDouble, etc...
		  * \param elem_do_wrap2pi If !=NULL; it must point to an array of "bool" of size()==dimension of each element, stating if it's needed to do a wrap to [-pi,pi] to each dimension.
		  * \ingroup stats_grp
		  */
		template<class VECTOR_OF_VECTORS, class MATRIXLIKE,class VECTORLIKE>
		void covariancesAndMean(const VECTOR_OF_VECTORS &elements,MATRIXLIKE &covariances,VECTORLIKE &means, const bool *elem_do_wrap2pi = NULL)
		{   // The function below is inline-expanded here:
			covariancesAndMeanWeighted<VECTOR_OF_VECTORS,MATRIXLIKE,VECTORLIKE,vector_double,vector_double>(elements,covariances,means,NULL,NULL,elem_do_wrap2pi);
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
				typedef typename VECTORLIKE1::value_type TNum;

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
				typedef typename VECTORLIKE1::value_type TNum;

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


			/** Extract a column from a vector of vectors, and store it in another vector.
			  *  - Input data can be: std::vector<vector_double>, std::deque<std::list<double> >, std::list<CArrayDouble<5> >, etc. etc.
			  *  - Output is the sequence:  data[0][idx],data[1][idx],data[2][idx], etc..
			  *
			  *  For the sake of generality, this function does NOT check the limits in the number of column, unless it's implemented in the [] operator of each of the "rows".
			  */
			template <class VECTOR_OF_VECTORS, class VECTORLIKE>
			inline void extractColumnFromVectorOfVectors(const size_t colIndex, const VECTOR_OF_VECTORS &data, VECTORLIKE &out_column)
			{
				const size_t N = data.size();
				out_column.resize(N);
				for (size_t i=0;i<N;i++)
					out_column[i]=data[i][colIndex];
			}

		/** Computes the factorial of an integer number and returns it as a 64-bit integer number.
		  */
		uint64_t BASE_IMPEXP  factorial64(unsigned int n);

		/** Computes the factorial of an integer number and returns it as a double value (internally it uses logarithms for avoiding overflow).
		  */
		double BASE_IMPEXP  factorial(unsigned int n);

		/** Round up to the nearest power of two of a given number
		  */
		template <class T>
		T round2up(T val)
		{
			T n = 1;
			while (n < val)
			{
				n <<= 1;
				if (n<=1)
					THROW_EXCEPTION("Overflow!");
			}
			return n;
		}

		/** Round a decimal number up to the given 10'th power (eg, to 1000,100,10, and also fractions)
		  *  power10 means round up to: 1 -> 10, 2 -> 100, 3 -> 1000, ...  -1 -> 0.1, -2 -> 0.01, ...
		  */
		template <class T>
		T round_10power(T val, int power10)
		{
			long double F = ::pow((long double)10.0,-(long double)power10);
			long int t = round_long( val * F );
			return T(t/F);
		}

		/** Calculate the correlation between two matrices
		  *  (by AJOGD @ JAN-2007)
		  */
		template<class T>
		double  correlate_matrix(const CMatrixTemplateNumeric<T> &a1, const CMatrixTemplateNumeric<T> &a2)
		{
			if ((a1.getColCount()!=a2.getColCount())|(a1.getRowCount()!=a2.getRowCount()))
				THROW_EXCEPTION("Correlation Error!, images with no same size");

			int i,j;
			T x1,x2;
			T syy=0, sxy=0, sxx=0, m1=0, m2=0 ,n=a1.getRowCount()*a2.getColCount();

			//find the means
			for (i=0;i<a1.getRowCount();i++)
			{
				for (j=0;j<a1.getColCount();j++)
				{
					m1 += a1(i,j);
					m2 += a2(i,j);
				}
			}
			m1 /= n;
			m2 /= n;

			for (i=0;i<a1.getRowCount();i++)
			{
				for (j=0;j<a1.getColCount();j++)
				{
					x1 = a1(i,j) - m1;
					x2 = a2(i,j) - m2;
					sxx += x1*x1;
					syy += x2*x2;
					sxy += x1*x2;
				}
			}

			return sxy / sqrt(sxx * syy);
		}

		/** A numerically-stable method to compute average likelihood values with strongly different ranges (unweighted likelihoods: compute the arithmetic mean).
		  *  This method implements this equation:
		  *
		  *  \f[ return = - \log N + \log  \sum_{i=1}^N e^{ll_i-ll_{max}} + ll_{max} \f]
		  *
		  * See also the <a href="http://www.mrpt.org/Averaging_Log-Likelihood_Values:Numerical_Stability">tutorial page</a>.
		  * \ingroup stats_grp
		  */
		double BASE_IMPEXP averageLogLikelihood( const vector_double &logLikelihoods );

		/** Computes the average of a sequence of angles in radians taking into account the correct wrapping in the range \f$ ]-\pi,\pi [ \f$, for example, the mean of (2,-2) is \f$ \pi \f$, not 0.
		  * \ingroup stats_grp
		  */
		double BASE_IMPEXP averageWrap2Pi(const vector_double &angles );

		/** A numerically-stable method to average likelihood values with strongly different ranges (weighted likelihoods).
		  *  This method implements this equation:
		  *
		  *  \f[ return = \log \left( \frac{1}{\sum_i e^{lw_i}} \sum_i  e^{lw_i} e^{ll_i}  \right) \f]
		  *
		  * See also the <a href="http://www.mrpt.org/Averaging_Log-Likelihood_Values:Numerical_Stability">tutorial page</a>.
		  * \ingroup stats_grp
		  */
		double BASE_IMPEXP  averageLogLikelihood(
			const vector_double &logWeights,
			const vector_double &logLikelihoods );

		/** Generates a string with the MATLAB commands required to plot an confidence interval (ellipse) for a 2D Gaussian ('float' version)..
		  *  \param cov22 The 2x2 covariance matrix
		  *  \param mean  The 2-length vector with the mean
		  *  \param stdCount How many "quantiles" to get into the area of the ellipse: 2: 95%, 3:99.97%,...
		  *  \param style A matlab style string, for colors, line styles,...
		  *  \param nEllipsePoints The number of points in the ellipse to generate
		  * \ingroup stats_grp
		  */
		std::string BASE_IMPEXP  MATLAB_plotCovariance2D(
			const CMatrixFloat  &cov22,
			const vector_float  &mean,
			const float         &stdCount,
			const std::string   &style = std::string("b"),
			const size_t        &nEllipsePoints = 30 );

		/** Generates a string with the MATLAB commands required to plot an confidence interval (ellipse) for a 2D Gaussian ('double' version).
		  *  \param cov22 The 2x2 covariance matrix
		  *  \param mean  The 2-length vector with the mean
		  *  \param stdCount How many "quantiles" to get into the area of the ellipse: 2: 95%, 3:99.97%,...
		  *  \param style A matlab style string, for colors, line styles,...
		  *  \param nEllipsePoints The number of points in the ellipse to generate
		  * \ingroup stats_grp
		  */
		std::string BASE_IMPEXP  MATLAB_plotCovariance2D(
			const CMatrixDouble  &cov22,
			const vector_double  &mean,
			const float         &stdCount,
			const std::string   &style = std::string("b"),
			const size_t        &nEllipsePoints = 30 );


		/** Efficiently compute the inverse of a 4x4 homogeneous matrix by only transposing the rotation 3x3 part and solving the translation with dot products.
		  *  This is a generic template which works with:
		  *    MATRIXLIKE: CMatrixTemplateNumeric, CMatrixFixedNumeric
		  */
		template <class MATRIXLIKE1,class MATRIXLIKE2>
		void homogeneousMatrixInverse(const MATRIXLIKE1 &M, MATRIXLIKE2 &out_inverse_M)
		{
			MRPT_START
			ASSERT_( M.isSquare() && size(M,1)==4);

			/* Instead of performing a generic 4x4 matrix inversion, we only need to
			  transpose the rotation part, then replace the translation part by
			  three dot products. See, for example:
			 https://graphics.stanford.edu/courses/cs248-98-fall/Final/q4.html

				[ux vx wx tx] -1   [ux uy uz -dot(u,t)]
				[uy vy wy ty]      [vx vy vz -dot(v,t)]
				[uz vz wz tz]    = [wx wy wz -dot(w,t)]
				[ 0  0  0  1]      [ 0  0  0     1    ]
			*/

			out_inverse_M.setSize(4,4);

			// 3x3 rotation part:
			out_inverse_M.set_unsafe(0,0, M.get_unsafe(0,0));
			out_inverse_M.set_unsafe(0,1, M.get_unsafe(1,0));
			out_inverse_M.set_unsafe(0,2, M.get_unsafe(2,0));

			out_inverse_M.set_unsafe(1,0, M.get_unsafe(0,1));
			out_inverse_M.set_unsafe(1,1, M.get_unsafe(1,1));
			out_inverse_M.set_unsafe(1,2, M.get_unsafe(2,1));

			out_inverse_M.set_unsafe(2,0, M.get_unsafe(0,2));
			out_inverse_M.set_unsafe(2,1, M.get_unsafe(1,2));
			out_inverse_M.set_unsafe(2,2, M.get_unsafe(2,2));

			const double tx = -M.get_unsafe(0,3);
			const double ty = -M.get_unsafe(1,3);
			const double tz = -M.get_unsafe(2,3);

			const double tx_ = tx*M.get_unsafe(0,0)+ty*M.get_unsafe(1,0)+tz*M.get_unsafe(2,0);
			const double ty_ = tx*M.get_unsafe(0,1)+ty*M.get_unsafe(1,1)+tz*M.get_unsafe(2,1);
			const double tz_ = tx*M.get_unsafe(0,2)+ty*M.get_unsafe(1,2)+tz*M.get_unsafe(2,2);

			out_inverse_M.set_unsafe(0,3, tx_ );
			out_inverse_M.set_unsafe(1,3, ty_ );
			out_inverse_M.set_unsafe(2,3, tz_ );

			out_inverse_M.set_unsafe(3,0,  0);
			out_inverse_M.set_unsafe(3,1,  0);
			out_inverse_M.set_unsafe(3,2,  0);
			out_inverse_M.set_unsafe(3,3,  1);

			MRPT_END
		}
		//! \overload
		template <class IN_ROTMATRIX,class IN_XYZ, class OUT_ROTMATRIX, class OUT_XYZ>
		void homogeneousMatrixInverse(
			const IN_ROTMATRIX  & in_R,
			const IN_XYZ        & in_xyz,
			OUT_ROTMATRIX & out_R,
			OUT_XYZ       & out_xyz
			)
		{
			MRPT_START
			ASSERT_( in_R.isSquare() && size(in_R,1)==3 && in_xyz.size()==3)
			out_R.setSize(3,3);
			out_xyz.resize(3);

			// translation part:
			typedef typename IN_XYZ::value_type T;
			const T tx = -in_xyz[0];
			const T ty = -in_xyz[1];
			const T tz = -in_xyz[2];

			out_xyz[0] = tx*in_R.get_unsafe(0,0)+ty*in_R.get_unsafe(1,0)+tz*in_R.get_unsafe(2,0);
			out_xyz[1] = tx*in_R.get_unsafe(0,1)+ty*in_R.get_unsafe(1,1)+tz*in_R.get_unsafe(2,1);
			out_xyz[2] = tx*in_R.get_unsafe(0,2)+ty*in_R.get_unsafe(1,2)+tz*in_R.get_unsafe(2,2);

			// 3x3 rotation part: transpose
			out_R = in_R.adjoint();

			MRPT_END
		}
		//! \overload
		template <class MATRIXLIKE>
		inline void homogeneousMatrixInverse(MATRIXLIKE &M)
		{
			ASSERTDEB_( M.isSquare() && size(M,1)==4);
			// translation:
			const double tx = -M(0,3);
			const double ty = -M(1,3);
			const double tz = -M(2,3);
			M(0,3) = tx*M(0,0)+ty*M(1,0)+tz*M(2,0);
			M(1,3) = tx*M(0,1)+ty*M(1,1)+tz*M(2,1);
			M(2,3) = tx*M(0,2)+ty*M(1,2)+tz*M(2,2);
			// 3x3 rotation part:
			std::swap( M(1,0),M(0,1) );
			std::swap( M(2,0),M(0,2) );
			std::swap( M(1,2),M(2,1) );
		}


		/** Estimate the Jacobian of a multi-dimensional function around a point "x", using finite differences of a given size in each input dimension.
		  *  The template argument USERPARAM is for the data can be passed to the functor.
		  *   If it is not required, set to "int" or any other basic type.
		  *
		  *  This is a generic template which works with:
		  *    VECTORLIKE: vector_float, vector_double, CArrayNumeric<>, double [N], ...
		  *    MATRIXLIKE: CMatrixTemplateNumeric, CMatrixFixedNumeric
		  */
		template <class VECTORLIKE,class VECTORLIKE2, class VECTORLIKE3, class MATRIXLIKE, class USERPARAM >
		void estimateJacobian(
			const VECTORLIKE 	&x,
			void 				(*functor) (const VECTORLIKE &x,const USERPARAM &y, VECTORLIKE3  &out),
			const VECTORLIKE2 	&increments,
			const USERPARAM		&userParam,
			MATRIXLIKE 			&out_Jacobian )
		{
			MRPT_START
			ASSERT_(x.size()>0 && increments.size() == x.size());

			size_t m = 0;		// will determine automatically on the first call to "f":
			const size_t n = x.size();

			for (size_t j=0;j<n;j++) { ASSERT_( increments[j]>0 ) }		// Who knows...

			VECTORLIKE3	f_minus, f_plus;
			VECTORLIKE	x_mod(x);

			// Evaluate the function "i" with increments in the "j" input x variable:
			for (size_t j=0;j<n;j++)
			{
				// Create the modified "x" vector:
				x_mod[j]=x[j]+increments[j];
				functor(x_mod,userParam,   f_plus);

				x_mod[j]=x[j]-increments[j];
				functor(x_mod,userParam,   f_minus);

				x_mod[j]=x[j]; // Leave as original
				const double Ax_2_inv = 0.5/increments[j];

				// The first time?
				if (j==0)
				{
					m = f_plus.size();
					out_Jacobian.setSize(m,n);
				}

				for (size_t i=0;i<m;i++)
					out_Jacobian.get_unsafe(i,j) = Ax_2_inv* (f_plus[i]-f_minus[i]);

			} // end for j

			MRPT_END
		}

		/** Assignment operator for initializing a std::vector from a C array (The vector will be automatically set to the correct size).
		  * \code
		  *	 vector_double  v;
		  *  const double numbers[] = { 1,2,3,5,6,7,8,9,10 };
		  *  loadVector( v, numbers );
		  * \endcode
		  * \note This operator performs the appropiate type castings, if required.
		  */
		template <typename T, typename At, size_t N>
		std::vector<T>& loadVector( std::vector<T> &v, At (&theArray)[N] )
		{
			MRPT_COMPILE_TIME_ASSERT(N!=0)
			v.resize(N);
			for (size_t i=0; i < N; i++)
				v[i] = static_cast<T>(theArray[i]);
			return v;
		}
		//! \overload
		template <typename Derived, typename At, size_t N>
		Eigen::EigenBase<Derived>& loadVector( Eigen::EigenBase<Derived> &v, At (&theArray)[N] )
		{
			MRPT_COMPILE_TIME_ASSERT(N!=0)
			v.derived().resize(N);
			for (size_t i=0; i < N; i++)
				(v.derived())[i] = static_cast<typename Derived::Scalar>(theArray[i]);
			return v;
		}

		/** Modify a sequence of angle values such as no consecutive values have a jump larger than PI in absolute value.
		  * \sa wrapToPi
		  */
		void unwrap2PiSequence(vector_double &x);

		/** A versatile template to build vectors on-the-fly in a style close to MATLAB's  v=[a b c d ...]
		  *  The first argument of the template is the vector length, and the second the type of the numbers.
		  *  Some examples:
		  *
		  *  \code
		  *    vector_double  = make_vector<4,double>(1.0,3.0,4.0,5.0);
		  *    vector_float   = make_vector<2,float>(-8.12, 3e4);
		  *  \endcode
		  */
		template <size_t N, typename T>
		std::vector<T> make_vector(const T val1, ...)
		{
			MRPT_COMPILE_TIME_ASSERT( N>0 )
			std::vector<T>	ret;
			ret.reserve(N);

			ret.push_back(val1);

			va_list args;
			va_start(args,val1);
			for (size_t i=0;i<N-1;i++)
				ret.push_back( va_arg(args,T) );

			va_end(args);
			return ret;
		}

		/**  @} */  // end of grouping container_ops_grp

		/** \addtogroup stats_grp
		  * @{
		  */

		/** @name Probability density distributions (pdf) distance metrics
		@{ */

		/** Computes the squared mahalanobis distance of a vector X given the mean MU and the covariance *inverse* COV_inv
		  *  \f[ d^2 =  (X-MU)^\top \Sigma^{-1} (X-MU)  \f]
		  */
		template<class VECTORLIKE1,class VECTORLIKE2,class MAT>
		typename VECTORLIKE1::value_type mahalanobisDistance2(
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
			mrpt::dynamicsize_vector<typename VECTORLIKE1::value_type> X_MU(N);
			for (size_t i=0;i<N;i++) X_MU[i]=X[i]-MU[i];
			return multiply_HCHt_scalar(X_MU, COV.inv() );
			MRPT_END
		}


		/** Computes the mahalanobis distance of a vector X given the mean MU and the covariance *inverse* COV_inv
		  *  \f[ d = \sqrt{ (X-MU)^\top \Sigma^{-1} (X-MU) }  \f]
		  */
		template<class VECTORLIKE1,class VECTORLIKE2,class MAT>
		inline typename VECTORLIKE1::value_type mahalanobisDistance(
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
		typename VECTORLIKE::value_type
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
		template<class VECTORLIKE,class MAT1,class MAT2,class MAT3> inline typename VECTORLIKE::value_type
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
		inline typename MATRIXLIKE::value_type
		mahalanobisDistance2(const VECTORLIKE &delta_mu,const MATRIXLIKE &cov)
		{
			ASSERTDEB_(cov.isSquare())
			ASSERTDEB_(cov.getColCount()==delta_mu.size())
			return multiply_HCHt_scalar(delta_mu,cov.inverse());
		}

		/** Computes the mahalanobis distance between a point and a Gaussian, given the covariance matrix and the vector with the difference between the mean and the point.
		  *  \f[ d^2 = \sqrt( \Delta_\mu^\top \Sigma^{-1} \Delta_\mu ) \f]
		  */
		template<class VECTORLIKE,class MATRIXLIKE>
		inline typename MATRIXLIKE::value_type
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
			CMatrixFixedNumeric<T,DIM,DIM> C_inv(UNINITIALIZED_MATRIX);
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
			log_pdf_out = static_cast<typename MATRIXLIKE::value_type>(-0.5)* (
				maha2_out+
				static_cast<typename MATRIXLIKE::value_type>(cov.getColCount())*::log(static_cast<typename MATRIXLIKE::value_type>(M_2PI))+
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

		/** @} */
		/** @} */  // end of grouping statistics

		/** @addtogroup interpolation_grp Interpolation, least-squares fit, splines
		  * \ingroup mrpt_base_grp
		  *  @{ */

		/** Interpolate a data sequence "ys" ranging from "x0" to "x1" (equally spaced), to obtain the approximation of the sequence at the point "x".
		  *  If the point "x" is out of the range [x0,x1], the closest extreme "ys" value is returned.
		  * \sa spline, interpolate2points
		  */
		template <class T,class VECTOR>
		T interpolate(
			const T			&x,
			const VECTOR	&ys,
			const T			&x0,
			const T			&x1 )
		{
			MRPT_START
			ASSERT_(x1>x0); ASSERT_(!ys.empty());
			const size_t N = ys.size();
			if (x<=x0)	return ys[0];
			if (x>=x1)	return ys[N-1];
			const T Ax = (x1-x0)/T(N);
			const size_t i = int( (x-x0)/Ax );
			if (i>=N-1) return ys[N-1];
			const T Ay = ys[i+1]-ys[i];
			return ys[i] + (x-(x0+i*Ax))*Ay/Ax;
			MRPT_END
		}

		/** Linear interpolation/extrapolation: evaluates at "x" the line (x0,y0)-(x1,y1).
		  *  If wrap2pi is true, output is wrapped to ]-pi,pi] (It is assumed that input "y" values already are in the correct range).
		  * \sa spline, interpolate, leastSquareLinearFit
		  */
		double BASE_IMPEXP interpolate2points(const double x, const double x0, const double y0, const double x1, const double y1, bool wrap2pi = false);

		/** Interpolates the value of a function in a point "t" given 4 SORTED points where "t" is between the two middle points
		  *  If wrap2pi is true, output "y" values are wrapped to ]-pi,pi] (It is assumed that input "y" values already are in the correct range).
		  * \sa leastSquareLinearFit
		  */
		double BASE_IMPEXP  spline(const double t, const vector_double &x, const vector_double &y, bool wrap2pi = false);

		/** Interpolates or extrapolates using a least-square linear fit of the set of values "x" and "y", evaluated at a single point "t".
		  *  The vectors x and y must have size >=2, and all values of "x" must be different.
		  *  If wrap2pi is true, output "y" values are wrapped to ]-pi,pi] (It is assumed that input "y" values already are in the correct range).
		  * \sa spline
		  * \sa getRegressionLine, getRegressionPlane
		  */
		template <typename NUMTYPE,class VECTORLIKE>
		NUMTYPE leastSquareLinearFit(const NUMTYPE t, const VECTORLIKE &x, const VECTORLIKE &y, bool wrap2pi = false)
		{
			MRPT_START

			// http://en.wikipedia.org/wiki/Linear_least_squares
			ASSERT_(x.size()==y.size());
			ASSERT_(x.size()>1);

			const size_t N = x.size();

			typedef typename VECTORLIKE::value_type NUM;

			// X= [1 columns of ones, x' ]
			const NUM x_min = x.minimum();
			CMatrixTemplateNumeric<NUM> Xt(2,N);
			for (size_t i=0;i<N;i++)
			{
				Xt.set_unsafe(0,i, 1);
				Xt.set_unsafe(1,i, x[i]-x_min);
			}

			CMatrixTemplateNumeric<NUM> XtX;
			XtX.multiply_AAt(Xt);

			CMatrixTemplateNumeric<NUM> XtXinv;
			XtX.inv_fast(XtXinv);

			CMatrixTemplateNumeric<NUM> XtXinvXt;	// B = inv(X' * X)*X'  (pseudoinverse)
			XtXinvXt.multiply(XtXinv,Xt);

			VECTORLIKE B;
			XtXinvXt.multiply_Ab(y,B);

			ASSERT_(B.size()==2)

			NUM ret = B[0] + B[1]*(t-x_min);

			// wrap?
			if (!wrap2pi)
					return ret;
			else 	return mrpt::math::wrapToPi(ret);

			MRPT_END
		}

		/** Interpolates or extrapolates using a least-square linear fit of the set of values "x" and "y", evaluated at a sequence of points "ts" and returned at "outs".
		  *  If wrap2pi is true, output "y" values are wrapped to ]-pi,pi] (It is assumed that input "y" values already are in the correct range).
		  * \sa spline, getRegressionLine, getRegressionPlane
		  */
		template <class VECTORLIKE1,class VECTORLIKE2,class VECTORLIKE3>
		void leastSquareLinearFit(
			const VECTORLIKE1 &ts,
			VECTORLIKE2 &outs,
			const VECTORLIKE3 &x,
			const VECTORLIKE3 &y,
			bool wrap2pi = false)
		{
			MRPT_START

			// http://en.wikipedia.org/wiki/Linear_least_squares
			ASSERT_(x.size()==y.size());
			ASSERT_(x.size()>1);

			const size_t N = x.size();

			// X= [1 columns of ones, x' ]
			typedef typename VECTORLIKE3::value_type NUM;
			const NUM x_min = x.minimum();
			CMatrixTemplateNumeric<NUM> Xt(2,N);
			for (size_t i=0;i<N;i++)
			{
				Xt.set_unsafe(0,i, 1);
				Xt.set_unsafe(1,i, x[i]-x_min);
			}

			CMatrixTemplateNumeric<NUM> XtX;
			XtX.multiply_AAt(Xt);

			CMatrixTemplateNumeric<NUM> XtXinv;
			XtX.inv_fast(XtXinv);

			CMatrixTemplateNumeric<NUM> XtXinvXt;	// B = inv(X' * X)*X' (pseudoinverse)
			XtXinvXt.multiply(XtXinv,Xt);

			VECTORLIKE3 B;
			XtXinvXt.multiply_Ab(y,B);

			ASSERT_(B.size()==2)

			const size_t tsN = size_t(ts.size());
			outs.resize(tsN);
			if (!wrap2pi)
				for (size_t k=0;k<tsN;k++)
					outs[k] = B[0] + B[1]*(ts[k]-x_min);
			else
				for (size_t k=0;k<tsN;k++)
					outs[k] = mrpt::math::wrapToPi( B[0] + B[1]*(ts[k]-x_min) );
			MRPT_END
		}

		/** @} */  // end grouping interpolation_grp

	} // End of MATH namespace

} // End of namespace

#endif
