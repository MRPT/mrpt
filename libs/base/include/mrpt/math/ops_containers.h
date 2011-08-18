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
#ifndef  mrpt_math_container_ops_H
#define  mrpt_math_container_ops_H

#include <mrpt/math/math_frwds.h>  // Fordward declarations

#include <mrpt/math/lightweight_geom_data.h>  // Fordward declarations

#include <functional>
#include <algorithm>
#include <cmath>

/** \addtogroup container_ops_grp Vector and matrices mathematical operations and other utilities
  *  \ingroup mrpt_base_grp
  *  @{ */

/** \file ops_containers.h
  * This file implements several operations that operate element-wise on individual or pairs of containers.
  *  Containers here means any of: mrpt::math::CVectorTemplace, mrpt::math::CArray, mrpt::math::CMatrixFixedNumeric, mrpt::math::CMatrixTemplate.
  *
  *  In general, any container having a type "mrpt_autotype" self-referencing to the type itself, and a dummy struct mrpt_container<>
  *   which is only used as a way to force the compiler to assure that BOTH containers are valid ones in binary operators.
  *   This restrictions
  *   have been designed as a way to provide "polymorphism" at a template level, so the "+,-,..." operators do not
  *   generate ambiguities for ANY type, and limiting them to MRPT containers.
  *
  *   In some cases, the containers provide specializations of some operations, for increased performance.
  */

#include <algorithm>
#include <numeric>
#include <functional>

#include <mrpt/math/CHistogram.h>  // Used in ::histogram()


namespace mrpt
{
	namespace math
	{
		/** Computes the normalized or normal histogram of a sequence of numbers given the number of bins and the limits.
		  *  In any case this is a "linear" histogram, i.e. for matrices, all the elements are taken as if they were a plain sequence, not taking into account they were in columns or rows.
		  *  If desired, out_bin_centers can be set to receive the bins centers.
		  */
		template<class CONTAINER>
		vector_double histogram(
			const CONTAINER &v,
			double limit_min,
			double limit_max,
			size_t number_bins,
			bool do_normalization = false,
			vector_double *out_bin_centers = NULL)
		{
			mrpt::math::CHistogram	H( limit_min, limit_max, number_bins );
			vector_double ret(number_bins);
			vector_double dummy_ret_bins;
			H.add(v);
			if (do_normalization)
					H.getHistogramNormalized( out_bin_centers ? *out_bin_centers : dummy_ret_bins, ret );
			else	H.getHistogram( out_bin_centers ? *out_bin_centers : dummy_ret_bins, ret );
			return ret;
		}

		/** Computes the cumulative sum of all the elements, saving the result in another container.
		  *  This works for both matrices (even mixing their types) and vectores/arrays (even mixing types),
		  *  and even to store the cumsum of any matrix into any vector/array, but not in opposite direction.
		  * \sa sum */
		template <class CONTAINER1,class CONTAINER2>
		inline void cumsum(const CONTAINER1 &in_data, CONTAINER2 &out_cumsum)
		{
			out_cumsum.resizeLike(in_data);
			typename CONTAINER1::value_type last=0;
			const size_t N = in_data.size();
			for (size_t i=0;i<N;i++)
				last = out_cumsum[i] = last + in_data[i];
		}

		/** Computes the cumulative sum of all the elements
		  * \sa sum  */
		template<class CONTAINER>
		inline CONTAINER cumsum(const CONTAINER &in_data)
		{
			CONTAINER ret;
			cumsum(in_data,ret);
			return ret;
		}

		template <class CONTAINER> inline typename CONTAINER::value_type norm_inf(const CONTAINER &v) { return v.norm_inf(); }
		template <class CONTAINER> inline typename CONTAINER::value_type norm(const CONTAINER &v) { return v.norm(); }
		template <class CONTAINER> inline typename CONTAINER::value_type maximum(const CONTAINER &v) { return v.maximum(); }
		template <class CONTAINER> inline typename CONTAINER::value_type minimum(const CONTAINER &v) { return v.minimum(); }

		template <typename T> inline T maximum(const std::vector<T> &v)
		{
			ASSERT_(!v.empty())
			T m = v[0];
			for (size_t i=0;i<v.size();i++) mrpt::utils::keep_max(m,v[i]);
			return m;
		}
		template <typename T> inline T minimum(const std::vector<T> &v)
		{
			ASSERT_(!v.empty())
			T m = v[0];
			for (size_t i=0;i<v.size();i++) mrpt::utils::keep_min(m,v[i]);
			return m;
		}

		/** \name Container initializer from pose classes
		  * @{
		  */

		/** Conversion of poses to MRPT containers (vector/matrix) */
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPoint2D &p) {
			C.resize(2,1);
			for (size_t i=0;i<2;i++)  C.coeffRef(i,0)=p[i];
			return C;
		}
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPoint3D &p) {
			C.resize(3,1);
			for (size_t i=0;i<3;i++)  C.coeffRef(i,0)=p[i];
			return C;
		}
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPose2D &p) {
			C.resize(3,1);
			for (size_t i=0;i<3;i++)  C.coeffRef(i,0)=p[i];
			return C;
		}
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPose3D &p) {
			C.resize(6,1);
			for (size_t i=0;i<6;i++)  C.coeffRef(i,0)=p[i];
			return C;
		}
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPose3DQuat &p) {
			C.resize(7,1);
			for (size_t i=0;i<7;i++)  C.coeffRef(i,0)=p[i];
			return C;
		}

		/** @} */



		/** \name Generic container element-wise operations - Miscelaneous
		  * @{
		  */

		/** Accumulate the squared-norm of a vector/array/matrix into "total" (this function is compatible with std::accumulate). */
		template <class CONTAINER>
		typename CONTAINER::value_type squareNorm_accum(const typename CONTAINER::value_type total, const CONTAINER &v);
		template <class CONTAINER> typename CONTAINER::value_type squareNorm_accum(const typename CONTAINER::value_type total, const CONTAINER &v) {
			return total+v.squaredNorm();
		}

		/** Compute the square norm of anything implementing [].
		  \sa norm */
		template<size_t N,class T,class U>
		inline T squareNorm(const U &v)	{
			T res=0;
			for (size_t i=0;i<N;i++) res+=square(v[i]);
			return res;
		}

		/** v1·v2: The dot product of two containers (vectors/arrays/matrices) */
		template <class CONTAINER1,class CONTAINER2>
		inline typename CONTAINER1::value_type
		dotProduct(const CONTAINER1 &v1,const CONTAINER1 &v2)
		{
			return v1.dot(v2);
		}

		/** v1·v2: The dot product of any two objects supporting []  */
		template<size_t N,class T,class U,class V>
		inline T dotProduct(const U &v1,const V &v2)	{
			T res=0;
			for (size_t i=0;i<N;i++) res+=v1[i]*v2[i];
			return res;
		}

		/** Computes the sum of all the elements.
		  * \note If used with containers of integer types (uint8_t, int, etc...) this could overflow. In those cases, use sumRetType the second argument RET to specify a larger type to hold the sum.
		   \sa cumsum  */
		template <class CONTAINER> inline typename CONTAINER::value_type sum(const CONTAINER &v) { return v.sum(); }

		/// \overload
		template <typename T> inline T sum(const std::vector<T> &v) { return std::accumulate(v.begin(),v.end(),T(0)); }

		/** Computes the sum of all the elements, with a custom return type.
		   \sa sum, cumsum  */
		template <class CONTAINER,typename RET> inline RET sumRetType(const CONTAINER &v) { return v.sumRetType<RET>(); }

		/** Computes the mean value of a vector  \return The mean, as a double number.
		  * \sa math::stddev,math::meanAndStd  */
		template <class CONTAINER>
		inline double mean(const CONTAINER &v)
		{
			if (v.empty())
			     return 0;
			else return sum(v)/static_cast<double>(v.size());
		}

		/** Return the maximum and minimum values of a std::vector */
		template <typename T>
		inline void minimum_maximum(const std::vector<T> &V, T&curMin,T&curMax)
		{
			ASSERT_(V.size()!=0)
			const size_t N=V.size();
			curMin=curMax=V[0];
			for (size_t i=1;i<N;i++)
			{
				mrpt::utils::keep_min(curMin,V[i]);
				mrpt::utils::keep_max(curMax,V[i]);
			}
		}

		/** Return the maximum and minimum values of a Eigen-based vector or matrix */
		template <class Derived>
		inline void minimum_maximum(
			const Eigen::MatrixBase<Derived> &V,
			typename Eigen::MatrixBase<Derived>::value_type &curMin,
			typename Eigen::MatrixBase<Derived>::value_type &curMax)
		{
			V.minimum_maximum(curMin,curMax);
		}

		/** Counts the number of elements that appear in both STL-like containers (comparison through the == operator)
		  *  It is assumed that no repeated elements appear within each of the containers.  */
		template <class CONTAINER1,class CONTAINER2>
		size_t  countCommonElements(const CONTAINER1 &a,const CONTAINER2 &b)
		{
		    size_t ret=0;
			for (typename CONTAINER1::const_iterator it1 = a.begin();it1!=a.end();it1++)
			    for (typename CONTAINER2::const_iterator it2 = b.begin();it2!=b.end();it2++)
                    if ( (*it1) == (*it2) )
                         ret++;
			return ret;
		}

		/** Adjusts the range of all the elements such as the minimum and maximum values being those supplied by the user.  */
		template <class CONTAINER>
		void  adjustRange(CONTAINER &m, const typename CONTAINER::value_type minVal,const typename CONTAINER::value_type maxVal)
		{
			if (size_t(m.size())==0) return;
			typename CONTAINER::value_type curMin,curMax;
			minimum_maximum(m,curMin,curMax);
			const typename CONTAINER::value_type curRan = curMax-curMin;
			m -= (curMin+minVal);
			if (curRan!=0) m *= (maxVal-minVal)/curRan;
		}


		/** Computes the standard deviation of a vector
		  * \param v The set of data
		  * \param out_mean The output for the estimated mean
		  * \param out_std The output for the estimated standard deviation
		  * \param unbiased If set to true or false the std is normalized by "N-1" or "N", respectively.
		  * \sa math::mean,math::stddev
		  */
		template<class VECTORLIKE>
		void  meanAndStd(
			const VECTORLIKE &v,
			double			&out_mean,
			double			&out_std,
			bool			unbiased = true)
		{
			if (v.size()<2)
			{
				out_std = 0;
				out_mean = (v.size()==1) ? *v.begin() : 0;
			}
			else
			{
				// Compute the mean:
				const size_t N = v.size();
				out_mean = mrpt::math::sum(v) / static_cast<double>(N);
				// Compute the std:
				double	vector_std=0;
				for (size_t i=0;i<N;i++) vector_std += mrpt::utils::square( v[i]-out_mean);
				out_std = std::sqrt(vector_std  / static_cast<double>(N - (unbiased ? 1:0)) );
			}
		}


		/** Computes the standard deviation of a vector
		  * \param v The set of data
		  * \param unbiased If set to true or false the std is normalized by "N-1" or "N", respectively.
		  * \sa math::mean,math::meanAndStd
		  */
		template<class VECTORLIKE>
		inline double  stddev(const VECTORLIKE &v, bool unbiased = true)
		{
			double m,s;
			meanAndStd(v,m,s,unbiased);
			return s;
		}

		/** Computes the mean vector and covariance from a list of values given as a vector of vectors, where each row is a sample.
		  * \param v The set of data, as a vector of N vectors of M elements.
		  * \param out_mean The output M-vector for the estimated mean.
		  * \param out_cov The output MxM matrix for the estimated covariance matrix.
		  * \sa mrpt::math::meanAndCovMat, math::mean,math::stddev, math::cov
		  */
		template<class VECTOR_OF_VECTOR, class VECTORLIKE, class MATRIXLIKE>
		void  meanAndCovVec(
			const VECTOR_OF_VECTOR &v,
			VECTORLIKE &out_mean,
			MATRIXLIKE &out_cov
			)
		{
			const size_t N = v.size();
			ASSERTMSG_(N>0,"The input vector contains no elements");
			const double N_inv = 1.0/N;

			const size_t M = v[0].size();
			ASSERTMSG_(M>0,"The input vector contains rows of length 0");

			// First: Compute the mean
			out_mean.assign(M,0);
			for (size_t i=0;i<N;i++)
				for (size_t j=0;j<M;j++)
					out_mean[j]+=v[i][j];
			out_mean*=N_inv;

			// Second: Compute the covariance
			//  Save only the above-diagonal part, then after averaging
			//  duplicate that part to the other half.
			out_cov.zeros(M,M);
			for (size_t i=0;i<N;i++)
			{
				for (size_t j=0;j<M;j++)
					out_cov.get_unsafe(j,j)+=square(v[i][j]-out_mean[j]);

				for (size_t j=0;j<M;j++)
					for (size_t k=j+1;k<M;k++)
						out_cov.get_unsafe(j,k)+=(v[i][j]-out_mean[j])*(v[i][k]-out_mean[k]);
			}
			for (size_t j=0;j<M;j++)
				for (size_t k=j+1;k<M;k++)
					out_cov.get_unsafe(k,j) = out_cov.get_unsafe(j,k);
			out_cov*=N_inv;
		}

		/** Computes the covariance matrix from a list of values given as a vector of vectors, where each row is a sample.
		  * \param v The set of data, as a vector of N vectors of M elements.
		  * \param out_cov The output MxM matrix for the estimated covariance matrix.
		  * \sa math::mean,math::stddev, math::cov, meanAndCovVec
		  */
		template<class VECTOR_OF_VECTOR>
		inline Eigen::MatrixXd covVector( const VECTOR_OF_VECTOR &v )
		{
			vector_double   m;
			Eigen::MatrixXd C;
			meanAndCovVec(v,m,C);
			return C;
		}


		/** Normalised Cross Correlation between two vector patches
		  * The Matlab code for this is
		  * a = a - mean2(a);
		  * b = b - mean2(b);
		  * r = sum(sum(a.*b))/sqrt(sum(sum(a.*a))*sum(sum(b.*b)));
		  */
		template <class CONT1,class CONT2>
		double ncc_vector( const CONT1 &patch1, const CONT2 &patch2 )
		{
			ASSERT_( patch1.size()==patch2.size() )

			double numerator = 0, sum_a = 0, sum_b = 0, result, a_mean, b_mean;
			a_mean = patch1.mean();
			b_mean = patch2.mean();

			const size_t N = patch1.size();
			for(size_t i=0;i<N;++i)
			{
				numerator += (patch1[i]-a_mean)*(patch2[i]-b_mean);
				sum_a += mrpt::utils::square(patch1[i]-a_mean);
				sum_b += mrpt::utils::square(patch2[i]-b_mean);
			}
			ASSERTMSG_(sum_a*sum_b!=0,"Divide by zero when normalizing.")
			result=numerator/std::sqrt(sum_a*sum_b);
			return result;
		}

		/** @} Misc ops */

	} // End of math namespace
} // End of mrpt namespace

/**  @} */  // end of grouping

#endif
