/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_math_vector_ops_H
#define  mrpt_math_vector_ops_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <iomanip>  // for setprecision(), etc.
#include <iterator> // std::ostream_iterator

// Many of the functions originally in this file are now in ops_containers.h
#include <mrpt/math/ops_containers.h>


namespace mrpt
{
	namespace utils { class CFileStream; }

	namespace math
	{
		// Frwd. decl.
		template <typename T, std::size_t N> class CArrayNumeric;

		/** \addtogroup container_ops_grp
		  *  @{ */

		/** \name Generic std::vector element-wise operations
		  * @{
		  */

		/** a*=b (element-wise multiplication) */
		template <typename T1,typename T2>
		inline std::vector<T1>& operator *=(std::vector<T1>&a, const std::vector<T2>&b)
		{
			ASSERT_EQUAL_(a.size(),b.size())
			const size_t N=a.size();
			for (size_t i=0;i<N;i++) a[i]*=b[i];
			return a;
		}

		/** a*=k (multiplication by a constant) */
		template <typename T1>
		inline std::vector<T1>& operator *=(std::vector<T1>&a, const T1 b)
		{
			const size_t N=a.size();
			for (size_t i=0;i<N;i++) a[i]*=b;
			return a;
		}

		/** a*b (element-wise multiplication) */
		template <typename T1,typename T2>
		inline std::vector<T1> operator *(const std::vector<T1>&a, const std::vector<T2>&b)
		{
			ASSERT_EQUAL_(a.size(),b.size())
			const size_t N=a.size();
			std::vector<T1> ret(N);
			for (size_t i=0;i<N;i++) ret[i]=a[i]*b[i];
			return ret;
		}

		/** a+=b (element-wise sum) */
		template <typename T1,typename T2>
		inline std::vector<T1>& operator +=(std::vector<T1>&a, const std::vector<T2>&b)
		{
			ASSERT_EQUAL_(a.size(),b.size())
			const size_t N=a.size();
			for (size_t i=0;i<N;i++) a[i]+=b[i];
			return a;
		}

		/** a+=b (sum a constant) */
		template <typename T1>
		inline std::vector<T1>& operator +=(std::vector<T1>&a, const T1 b)
		{
			const size_t N=a.size();
			for (size_t i=0;i<N;i++) a[i]+=b;
			return a;
		}

		/** a+b (element-wise sum) */
		template <typename T1,typename T2>
		inline std::vector<T1> operator +(const std::vector<T1>&a, const std::vector<T2>&b)
		{
			ASSERT_EQUAL_(a.size(),b.size())
			const size_t N=a.size();
			std::vector<T1> ret(N);
			for (size_t i=0;i<N;i++) ret[i]=a[i]+b[i];
			return ret;
		}

		template <typename T1,typename T2>
		inline std::vector<T1> operator -(const std::vector<T1> &v1, const std::vector<T2>&v2) {
			ASSERT_EQUAL_(v1.size(),v2.size())
			std::vector<T1> res(v1.size());
			for (size_t i=0;i<v1.size();i++) res[i]=v1[i]-v2[i];
			return res;
		}

		/** @} */


		/** A template function for printing out the contents of a std::vector variable.
			*/
		template <class T>
		std::ostream& operator << (std::ostream& out, const std::vector<T> &d)
		{
			const std::streamsize old_pre = out.precision();
			const std::ios_base::fmtflags old_flags = out.flags();
			out << "[" << std::fixed << std::setprecision(4);
			std::copy(d.begin(),d.end(), std::ostream_iterator<T>(out," "));
			out << "]";
			out.flags(old_flags);
			out.precision(old_pre);
			return out;
		}

		/** A template function for printing out the contents of a std::vector variable.
			*/
		template <class T>
		std::ostream& operator << (std::ostream& out, std::vector<T> *d)
		{
			const std::streamsize old_pre = out.precision();
			const std::ios_base::fmtflags old_flags = out.flags();
			out << "[" << std::fixed << std::setprecision(4);
			copy(d->begin(),d->end(), std::ostream_iterator<T>(out," "));
			out << "]";
			out.flags(old_flags);
			out.precision(old_pre);
			return out;
		}

		/** Binary dump of a CArrayNumeric<T,N> to a stream. */
		template <typename T,size_t N>
		mrpt::utils::CStream& operator << (mrpt::utils::CStream& ostrm, const CArrayNumeric<T,N>& a)
		{
			ostrm << mrpt::utils::TTypeName< CArrayNumeric<T,N> >::get();
			if (N) ostrm.WriteBufferFixEndianness<T>(&a[0],N);
			return ostrm;
		}

		/** Binary read of a CArrayNumeric<T,N> from a stream. */
		template <typename T,size_t N>
		mrpt::utils::CStream& operator >> (mrpt::utils::CStream& istrm, CArrayNumeric<T,N>& a)
		{
			static const std::string namExpect = mrpt::utils::TTypeName< CArrayNumeric<T,N> >::get();
			std::string nam;
			istrm >> nam;
			ASSERTMSG_(nam==namExpect, format("Error deserializing: expected '%s', got '%s'", namExpect.c_str(),nam.c_str() ) )
			if (N) istrm.ReadBufferFixEndianness<T>(&a[0],N);
			return istrm;
		}


		/**  @} */  // end of grouping

	} // End of math namespace

} // End of mrpt namespace


#endif
