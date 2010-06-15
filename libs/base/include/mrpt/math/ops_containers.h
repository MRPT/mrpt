/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

		/** \name Generic container element-wise operations - Basic arithmetic
		  * @{
		  */

		/** a+=b */
		template <class CONTAINER1,class CONTAINER2>
		inline RET_CONT1_ASSERT_MRPTCONTAINERS(CONTAINER1,CONTAINER2) &
		operator +=(CONTAINER1 &a, const CONTAINER2 &b)
		{
			ASSERT_(a.size()==b.size());
			typename CONTAINER1::iterator ita = a.begin();
			typename CONTAINER2::const_iterator itb = b.begin();
			const typename CONTAINER1::iterator last=a.end();
			while (ita!=last) { *(ita++)+=*(itb++); }
			return a;
		}

		/** return a+b */
		template <class CONTAINER1,class CONTAINER2>
		inline RET_CONT1_ASSERT_MRPTCONTAINERS(CONTAINER1,CONTAINER2)
		operator +(const CONTAINER1 &a, const CONTAINER2 &b)
		{
			CONTAINER1 ret = a;
			ret+=b;
			return ret;
		}

		/** a-=b */
		template <class CONTAINER1,class CONTAINER2>
		inline RET_CONT1_ASSERT_MRPTCONTAINERS(CONTAINER1,CONTAINER2) &
		operator -=(CONTAINER1 &a, const CONTAINER2 &b)
		{
			ASSERT_(a.size()==b.size());
			typename CONTAINER1::iterator ita = a.begin();
			typename CONTAINER2::const_iterator itb = b.begin();
			const typename CONTAINER1::iterator last=a.end();
			while (ita!=last) { *(ita++)-=*(itb++); }
			return a;
		}

		/** return a-b */
		template <class CONTAINER1,class CONTAINER2>
		inline RET_CONT1_ASSERT_MRPTCONTAINERS(CONTAINER1,CONTAINER2)
		operator -(const CONTAINER1 &a, const CONTAINER2 &b)
		{
			CONTAINER1 ret = a;
			ret-=b;
			return ret;
		}

		/** a/=b (element-wise division) */
		template<class CONTAINER1,class CONTAINER2> CONTAINER1 &divide_elementwise(CONTAINER1 &a, const CONTAINER2 &b)
		{
			ASSERT_(a.size()==b.size());
			typename CONTAINER1::iterator ita = a.begin();
			typename CONTAINER2::const_iterator itb = b.begin();
			const typename CONTAINER1::iterator last=a.end();
			while (ita!=last) { *(ita++)/=*(itb++); }
			return a;
		}

		/** Sum a scalar to all the elements */
		template <class CONTAINER>
		inline RET_CONT_ASSERT_MRPTCONTAINER(CONTAINER) & operator+=(CONTAINER &c, typename CONTAINER::value_type  val)
		{
			const typename CONTAINER::iterator last=c.end();
			for (typename CONTAINER::iterator it=c.begin();it!=last;++it)  *it += val;
			return c;
		}

		/** Sum a scalar to all the elements */
		template <class CONTAINER>
		inline RET_CONT_ASSERT_MRPTCONTAINER(CONTAINER) operator +(CONTAINER &c, typename CONTAINER::value_type  val)
		{
			CONTAINER ret = c;
			ret+=val;
			return ret;
		}

		/** Substract a scalar to all the elements */
		template <class CONTAINER>
		inline RET_CONT_ASSERT_MRPTCONTAINER(CONTAINER) & operator -= (CONTAINER &c, typename CONTAINER::value_type  val)
		{
			const typename CONTAINER::iterator last=c.end();
			for (typename CONTAINER::iterator it=c.begin();it!=last;++it)  *it -= val;
			return c;
		}

		/** Substract a scalar to all the elements */
		template <class CONTAINER>
		inline RET_CONT_ASSERT_MRPTCONTAINER(CONTAINER) operator -(CONTAINER &c, typename CONTAINER::value_type  val)
		{
			CONTAINER ret = c;
			ret-=val;
			return ret;
		}

		/** Multiplies all the elements by a scalar C *= s */
		template <class CONTAINER>
		inline RET_CONT_ASSERT_MRPTCONTAINER(CONTAINER) & operator *= (CONTAINER &c, typename CONTAINER::value_type  val)
		{
			const typename CONTAINER::iterator last=c.end();
			for (typename CONTAINER::iterator it=c.begin();it!=last;++it)  *it *= val;
			return c;
		}

		/** Multiplies all the elements by a scalar C2 = C*s */
		template <class CONTAINER>
		inline RET_CONT_ASSERT_MRPTCONTAINER(CONTAINER) operator *(const CONTAINER &c, const typename CONTAINER::value_type  val)
		{
			CONTAINER ret = c;
			ret*=val;
			return ret;
		}
		/** Multiplies all the elements by a scalar C2= s*C */
		template <class CONTAINER>
		inline RET_CONT_ASSERT_MRPTCONTAINER(CONTAINER) operator *(const typename CONTAINER::value_type  val, const CONTAINER &c)
		{
			CONTAINER ret = c;
			ret*=val;
			return ret;
		}

		/** Divides all the elements by a scalar */
		template <class CONTAINER>
		inline RET_CONT_ASSERT_MRPTCONTAINER(CONTAINER) & operator /= (CONTAINER &c, typename CONTAINER::value_type  val)
		{
			typename CONTAINER::value_type k = 1/val; // Multiply is cheaper..
			const typename CONTAINER::iterator last=c.end();
			for (typename CONTAINER::iterator it=c.begin();it!=last;++it)  *it *= k;
			return c;
		}

		/** Divides all the elements by a scalar */
		template <class CONTAINER>
		inline RET_CONT_ASSERT_MRPTCONTAINER(CONTAINER) operator /(CONTAINER &c, typename CONTAINER::value_type  val)
		{
			CONTAINER ret = c;
			ret/=val;
			return ret;
		}


		/** @} end Arithmetic  */


		/** \name Generic container element-wise operations - Miscelaneous
		  * @{
		  */

		/** Conversion of poses to MRPT containers (vector/matrix) */
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPoint2D &p) {
			ASSERT_(C.size()==2)
			for (size_t i=0;i<2;i++)  C._A(i)=p[i];
			return C;
		}
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPoint3D &p) {
			ASSERT_(C.size()==3)
			for (size_t i=0;i<3;i++)  C._A(i)=p[i];
			return C;
		}
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPose2D &p) {
			ASSERT_(C.size()==3)
			for (size_t i=0;i<3;i++)  C._A(i)=p[i];
			return C;
		}
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPose3D &p) {
			ASSERT_(C.size()==6)
			for (size_t i=0;i<6;i++)  C._A(i)=p[i];
			return C;
		}
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPose3DQuat &p) {
			ASSERT_(C.size()==7)
			for (size_t i=0;i<7;i++)  C._A(i)=p[i];
			return C;
		}

		/** A template for counting how many elements in a container are non-Zero. */
		template <class CONTAINER>
		size_t countNonZero(const CONTAINER &a)
		{
			typename CONTAINER::const_iterator	it_a;
			size_t count=0;
			for (it_a=a.begin(); it_a!=a.end(); it_a++) if (*it_a) count++;
			return count;
		}

		/** Finds the maximum value (and the corresponding zero-based index) from a given container.
		  */
		template <class CONTAINER>
		typename CONTAINER::value_type
		maximum(const CONTAINER &v, size_t *maxIndex)
		{
			typename CONTAINER::const_iterator maxIt = std::max_element(v.begin(),v.end());
			if (maxIndex) *maxIndex = std::distance(v.begin(),maxIt);
			return *maxIt;
		}

		/** Finds the maximum value (and the corresponding zero-based index) from a given vector.
		  * \sa maximum, minimum_maximum
		  */
		template <class CONTAINER>
		typename CONTAINER::value_type
		minimum(const CONTAINER &v, size_t *minIndex)
		{
			typename CONTAINER::const_iterator minIt = std::min_element(v.begin(),v.end());
			if (minIndex) *minIndex = std::distance(v.begin(),minIt);
			return *minIt;
		}

		/** Compute the minimum and maximum of a vector at once
		  * \sa maximum, minimum
		  */
		template <class CONTAINER>
		void minimum_maximum(
			const CONTAINER &v,
			typename CONTAINER::mrpt_autotype::value_type & out_min,
			typename CONTAINER::mrpt_autotype::value_type& out_max,
			size_t *minIndex,
			size_t *maxIndex)
		{
			const size_t N = v.size();
			if (N)
			{
				typename CONTAINER::const_iterator it = v.begin();
				size_t min_idx=0,max_idx=0;
				out_max = out_min = *it;
				size_t i;
				for (i=0;i<N;++it,++i)
				{
					if (*it<out_min)
					{
						out_min=*it;
						min_idx = i;
					}
					if (*it>out_max)
					{
						out_max=*it;
						max_idx = i;
					}
				}
				if (minIndex) *minIndex = min_idx;
				if (maxIndex) *maxIndex = max_idx;
			}
			else {
				out_min=out_max=0;
				if (minIndex) *minIndex = 0;
				if (maxIndex) *maxIndex = 0;
			}
		}
		/** Compute the norm-infinite of a vector ($f[ ||\mathbf{v}||_\infnty $f]), ie the maximum absolute value of the elements.
		  */
		template <class CONTAINER>
		typename CONTAINER::value_type
		norm_inf(const CONTAINER &v, size_t *maxIndex)
		{
			double	M=0;
			int		i,M_idx=-1;
			typename CONTAINER::const_iterator	it;
			for (i=0, it=v.begin(); it!=v.end();it++,i++)
			{
				double	it_abs = fabs( static_cast<double>(*it));
				if (it_abs>M || M_idx==-1)
				{
					M = it_abs;
					M_idx = i;
				}
			}
			if (maxIndex) *maxIndex = M_idx;
			return static_cast<typename CONTAINER::mrpt_autotype::value_type>(M);
		}


		/** Compute the square norm of a vector/array/matrix (the Euclidean distance to the origin, taking all the elements as a single vector).
		  \sa norm */
		template <class CONTAINER>
		typename CONTAINER::value_type
		squareNorm(const CONTAINER &v)
		{
			typename CONTAINER::value_type	total=0;
			typename CONTAINER::const_iterator	it;
			for (it=v.begin(); it!=v.end();it++)
				total += square(*it);
			return total;
		}

		/** Compute the square norm of anything implementing [].
		  \sa norm */
		template<size_t N,class T,class U>
		inline T squareNorm(const U &v)	{
			T res=0;
			for (size_t i=0;i<N;i++) res+=square(v[i]);
			return res;
		}

		/** Compute the norm of a vector/array/matrix (the Euclidean distance to the origin, taking all the elements as a single vector).
		  \sa squareNorm */
		template <class CONTAINER>
		inline typename CONTAINER::value_type
		norm(const CONTAINER &v)
		{
			return ::sqrt(squareNorm(v));
		}

		/** v1·v2: The dot product of two containers (vectors/arrays/matrices) */
		template <class CONTAINER1,class CONTAINER2>
		inline typename CONTAINER1::value_type
		dotProduct(const CONTAINER1 &v1,const CONTAINER1 &v2)
		{
			ASSERT_(v1.size()==v2.size());
			const size_t N = v1.size();
			typename CONTAINER1::value_type res=0;
			typename CONTAINER1::const_iterator	it1;
			typename CONTAINER2::const_iterator	it2;
			size_t i;
			for (i=0,it1=v1.begin(),it2=v2.begin();i<N;++i,++it1,++it2) res+=(*it1)*(*it2);
			return res;
		}

		/** v1·v2: The dot product of any two objects supporting []  */
		template<size_t N,class T,class U,class V>
		inline T dotProduct(const U &v1,const V &v2)	{
			T res=0;
			for (size_t i=0;i<N;i++) res+=v1[i]*v2[i];
			return res;
		}

		/** Computes the mean value of a vector  \return The mean, as a double number.
		  * \sa math::stddev,math::meanAndStd  */
		template <class CONTAINER>
		inline double
		mean(const CONTAINER &v)
		{
			if (v.empty())
				return 0;
			else
				return std::accumulate(v.begin(),v.end(), 0.0) / double(v.size());
		}

		/** Computes the sum of all the elements.
		  * \note If used with containers of integer types (uint8_t, int, etc...) this could overflow. In those cases, use sumRetType the second argument RET to specify a larger type to hold the sum.
		   \sa cumsum  */
		template <class CONTAINER>
		inline typename CONTAINER::value_type sum(const CONTAINER &v)
		{
			return std::accumulate(v.begin(),v.end(), static_cast<typename CONTAINER::value_type>(0) );
		}

		/** Computes the sum of all the elements, with a custom return type.
		   \sa sum, cumsum  */
		template <class CONTAINER,typename RET>
		inline RET sumRetType(const CONTAINER &v)
		{
			return std::accumulate(v.begin(),v.end(), static_cast<RET>(0) );
		}

		/** Computes the cumulative sum of all the elements, saving the result in another container.
		  *  This works for both matrices (even mixing their types) and vectores/arrays (even mixing types),
		  *  and even to store the cumsum of any matrix into any vector/array, but not in opposite direction.
		  * \sa sum */
		template <class CONTAINER1,class CONTAINER2>
		void cumsum(const CONTAINER1 &in_data, CONTAINER2 &out_cumsum)
		{
			out_cumsum.resize(in_data.size());
			typename CONTAINER1::value_type last=0;
			const size_t N = in_data.size();
			typename CONTAINER1::const_iterator	it1;
			typename CONTAINER2::iterator	it2;
			size_t i;
			for (i=0,it1=in_data.begin(),it2=out_cumsum.begin();i<N;++i,++it1,++it2)
				last = (*it2) = last + (*it1);
		}

		/** Computes the cumulative sum of all the elements
		  * \sa sum  */
		template<class CONTAINER>
		inline CONTAINER
		cumsum(const CONTAINER &in_data)
		{
			CONTAINER ret;
			ret.resize(in_data.size());
			cumsum(in_data,ret);
			return ret;
		}

		/** Counts the number of elements that appear in both containers (comparison through the == operator)
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

		/** Computes the normalized or normal histogram of a sequence of numbers given the number of bins and the limits.
		  *  In any case this is a "linear" histogram, i.e. for matrices, all the elements are taken as if they were a plain sequence, not taking into account they were in columns or rows.
		  *  If desired, out_bin_centers can be set to receive the bins centers.
		  */
		template<class CONTAINER>
		std::vector<double> histogram(
			const CONTAINER &v,
			double limit_min,
			double limit_max,
			size_t number_bins,
			bool do_normalization,
			std::vector<double>  *out_bin_centers
			)
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

		namespace detail
		{
			template <class CONTAINER>
			void saveToTextFileAsVector(
				const CONTAINER &M,
				const std::string &file,
				mrpt::math::TMatrixTextFileFormat fileFormat,
				bool asColumnVector)
			{
				using namespace mrpt::system;
				MRPT_START
				FILE	*f=os::fopen(file.c_str(),"wt");
				if (!f) THROW_EXCEPTION_CUSTOM_MSG1("saveToTextFile: Error opening file '%s' for writing a matrix as text.", file.c_str());

				for (typename CONTAINER::const_iterator it=M.begin();it!=M.end();++it)
				{
					switch(fileFormat)
					{
					case MATRIX_FORMAT_ENG: os::fprintf(f,"%.16e ",static_cast<double>(*it)); break;
					case MATRIX_FORMAT_FIXED: os::fprintf(f,"%.16f ",static_cast<double>(*it)); break;
					case MATRIX_FORMAT_INT: os::fprintf(f,"%i ",static_cast<int>(*it)); break;
					default:
						THROW_EXCEPTION("Unsupported value for the parameter 'fileFormat'!");
					};
					// If saving as a column, save a newline char:
					if (asColumnVector) os::fprintf(f,"\n");
				}
				os::fclose(f);
				MRPT_END
			}
		}

		/** @} Misc ops */

		/** \name Generic container element-wise operations - Mathematical functions
		  * @{
		  */
		/** out = abs(in), element by element */
		template <class CONTAINER1,class CONTAINER2>
		void Abs(const CONTAINER1 &dat_in, CONTAINER2 &dat_out) {
			dat_out.resize(dat_in.size());
			std::transform<typename CONTAINER1::const_iterator,typename CONTAINER2::iterator,typename CONTAINER1::value_type (*)(typename CONTAINER1::value_type)>
				(dat_in.begin(),dat_in.end(), dat_out.begin(), &::abs );
		}
		/** return abs(in), element by element */
		template <class CONTAINER>  inline CONTAINER Abs(const CONTAINER &dat_in)
		{ CONTAINER ret; Abs(dat_in,ret); return ret; }

		/** out = log(in), element by element */
		template <class CONTAINER1,class CONTAINER2>
		void Log(const CONTAINER1 &dat_in, CONTAINER2 &dat_out) {
			dat_out.resize(dat_in.size());
			std::transform<typename CONTAINER1::const_iterator,typename CONTAINER2::iterator,typename CONTAINER1::value_type (*)(typename CONTAINER1::value_type)>
				(dat_in.begin(),dat_in.end(), dat_out.begin(), &::log );
		}
		/** return log(in), element by element */
		template <class CONTAINER>  inline CONTAINER Log(const CONTAINER &dat_in)
		{ CONTAINER ret; Log(dat_in,ret); return ret; }

		/** out = exp(in), element by element */
		template <class CONTAINER1,class CONTAINER2>
		void Exp(const CONTAINER1 &dat_in, CONTAINER2 &dat_out) {
			dat_out.resize(dat_in.size());
			std::transform<typename CONTAINER1::const_iterator,typename CONTAINER2::iterator,typename CONTAINER1::value_type (*)(typename CONTAINER1::value_type)>
				(dat_in.begin(),dat_in.end(), dat_out.begin(), &::exp );
		}
		/** return exp(in), element by element */
		template <class CONTAINER>  inline CONTAINER Exp(const CONTAINER &dat_in)
		{ CONTAINER ret; Exp(dat_in,ret); return ret; }


		/** Logical equal-to == operator between any pair of matrix or vectors */
		template <class CONTAINER1,class CONTAINER2>
		RET_TYPE_ASSERT_MRPTCONTAINER(CONTAINER1, bool)
		operator == (const CONTAINER1 &m1, const CONTAINER2 &m2)
		{
		   if (m1.size()!=m2.size()) return false;
		   typename CONTAINER1::const_iterator it1=m1.begin();
		   typename CONTAINER2::const_iterator it2=m2.begin();
		   const size_t N = m1.size();
		   for(size_t i=0;i<N;++i, ++it1,++it2)
				if (*it1!=*it2)
					 return false;
		   return true;
		}

		/** Logical not-equal-to != operator between any pair of matrix or vectors  */
		template <class CONTAINER1,class CONTAINER2>
		inline RET_TYPE_ASSERT_MRPTCONTAINER(CONTAINER1, bool)
		operator != (const CONTAINER1 &m1, const CONTAINER2 &m2)
		{
			return !(m1 == m2);
		}

		/** Computes the standard deviation of a vector
		  * \param v The set of data
		  * \param unbiased If set to true or false the std is normalized by "N-1" or "N", respectively.
		  * \sa math::mean,math::meanAndStd
		  */
		template<class VECTORLIKE>
		double  stddev(const VECTORLIKE &v, bool unbiased)
		{
			if (v.size()<2)
				return 0;
			else
			{
				// Compute the mean:
				double	 vector_std=0,vector_mean = 0;
				for (typename VECTORLIKE::const_iterator it = v.begin();it!=v.end();++it) vector_mean += (*it);
				vector_mean /= static_cast<double>(v.size());
				// Compute the std:
				for (typename VECTORLIKE::const_iterator it = v.begin();it!=v.end();++it) vector_std += square((*it)-vector_mean);
				vector_std = sqrt(vector_std  / static_cast<double>(v.size() - (unbiased ? 1:0)) );
				return vector_std;
			}
		}

		/** Computes the mean vector and covariance from a list of values given as a vector of vectors, where each row is a sample.
		  * \param v The set of data, as a vector of N vectors of M elements.
		  * \param out_mean The output M-vector for the estimated mean.
		  * \param out_cov The output MxM matrix for the estimated covariance matrix.
		  * \sa math::mean,math::stddev, math::cov
		  */
		template<class VECTOR_OF_VECTOR, class VECTORLIKE, class MATRIXLIKE>
		void  meanAndCov(
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
				bool			unbiased)
		{
			if (v.size()<2)
			{
				out_std = 0;
				out_mean = (v.size()==1) ? *v.begin() : 0;
			}
			else
			{
				// Compute the mean:
				typename VECTORLIKE::const_iterator it;
				out_std=0,out_mean = 0;
				for (it = v.begin();it!=v.end();it++) out_mean += (*it);
				out_mean /= static_cast<double>(v.size());

				// Compute the std:
				for (it = v.begin();it!=v.end();it++) out_std += mrpt::utils::square(static_cast<double>(*it)-out_mean);
				out_std = std::sqrt(out_std / static_cast<double>((v.size() - (unbiased ? 1:0)) ));
			}
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

			typename CONT1::const_iterator it1;
			typename CONT2::const_iterator it2;
			const size_t N = patch1.size();
			size_t i;
			for(i=0,it1=patch1.begin(),it2=patch2.begin();i<N; ++it1, ++it2, ++i)
			{
				numerator += (*it1-a_mean)*(*it2-b_mean);
				sum_a += mrpt::utils::square(*it1-a_mean);
				sum_b += mrpt::utils::square(*it2-b_mean);
			}
			ASSERTMSG_(sum_a*sum_b!=0,"Divide by zero when normalizing.")
			result=numerator/sqrt(sum_a*sum_b);
			return result;
		}


		/** @} Math ops */

	} // End of math namespace
} // End of mrpt namespace


#endif
