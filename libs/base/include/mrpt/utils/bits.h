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

#ifndef UTILSDEFS_H
#error "This file is intended for include from utils_defs.h only!"
#endif

#include <mrpt/utils/SSE_types.h>  // needed by SSE intrinsics used in some inline functions below.

/** This is the global namespace for all Mobile Robot Programming Toolkit (MRPT) libraries.
 */
namespace mrpt
{
	/** A std::string version of C sprintf.
	  *  You can call this to obtain a std::string using printf-like syntax.
	  *  Based on very nice code by Paul Senzee, published at http://senzee.blogspot.com/2006/05/c-formatting-stdstring.html
	  *  Function implemented in format.cpp
	  */
	std::string BASE_IMPEXP format(const char *fmt, ...) MRPT_printf_format_check(1,2);

	namespace system
	{
		// Forward definition: (Required for Visual C++ 6 implementation of THROW_EXCEPTION...)
		std::string BASE_IMPEXP extractFileName(const std::string &filePath);
		std::string BASE_IMPEXP stack_trace(bool calling_from_exception);
	}

	namespace math
	{
		bool BASE_IMPEXP isNaN(float v) MRPT_NO_THROWS;
		bool BASE_IMPEXP isNaN(double v) MRPT_NO_THROWS;
		bool BASE_IMPEXP isFinite(float v) MRPT_NO_THROWS;
		bool BASE_IMPEXP isFinite(double v) MRPT_NO_THROWS;

		// This inline function is used everywhere, so just move it here even it's not a forward declaration!
		/*! Returns the size of the matrix in the i'th dimension: 1=rows, 2=columns (MATLAB-compatible function)
		  *  \note Template argument MATRIXLIKE can be: CMatrixTemplate, CMatrixTemplateNumeric, CMatrixFixedNumeric
		  */
		template <class MATRIXLIKE>
		inline size_t size( const MATRIXLIKE& m, int dim )
		{
			if (dim==1) return m.getRowCount();
			else if (dim==2) return m.getColCount();
			else THROW_EXCEPTION_CUSTOM_MSG1("size: Queried matrix dimension must be 1 or 2. Called with i=%i",dim);
		}
	}

	namespace utils
	{
		class CFileStream;
		void BASE_IMPEXP global_profiler_enter(const char *func_name) MRPT_NO_THROWS;
		void BASE_IMPEXP global_profiler_leave(const char *func_name) MRPT_NO_THROWS;

		struct CProfilerProxy {
			const char*f;
			CProfilerProxy(const char*func_name) : f(func_name) { global_profiler_enter(f); }
			~CProfilerProxy() { global_profiler_leave(f); }
		};

		/** Degrees to radians */
		inline double DEG2RAD(const double x) { return x*M_PI/180.0;	}
		/** Degrees to radians */
		inline float DEG2RAD(const float x) { return x*M_PIf/180.0f; }
		/** Degrees to radians */
		inline float DEG2RAD(const int x) { return x*M_PIf/180.0f; }
		/** Radians to degrees */
		inline double RAD2DEG(const double x) { return x*180.0/M_PI; }
		/** Radians to degrees */
		inline float RAD2DEG(const float x) { return x*180.0f/M_PIf; }

#	ifdef HAVE_LONG_DOUBLE
		/** Degrees to radians */
		inline long double DEG2RAD(const long double x) { return x*M_PIl/180.0;	}
		/** Radians to degrees */
		inline long double RAD2DEG(const long double x) { return x*180.0/M_PIl; }
#	endif

		/** Returns the sign of X as "1" or "-1" */
		template <typename T>
		inline int sign(T x) { return x<0 ? -1:1; }

		/** Returns the sign of X as "0", "1" or "-1" */
		template <typename T>
		inline int signWithZero(T x)	{ return x==0?0:sign(x);}

		/** Returns the closer integer (int) to x */
		template <typename T>
		inline int round(const T value)
		{
		#if MRPT_HAS_SSE2
			__m128d t = _mm_set_sd( value );
			return _mm_cvtsd_si32(t);
		#elif (defined WIN32 || defined _WIN32) && !defined WIN64 && !defined _WIN64 && defined _MSC_VER
			int t;
			__asm
			{
				fld value;
				fistp t;
			}
			return t;
		#elif defined HAVE_LRINT || defined __GNUC__
			return static_cast<int>(lrint(value));
		#else
			return static_cast<int>(value + 0.5);
		#endif
		}

		/** Returns the closer integer (long) to x */
		template <typename T>
		inline long round_long(const T value)
		{
		#if MRPT_HAS_SSE2 && MRPT_WORD_SIZE==64
			__m128d t = _mm_set_sd( value );
			return _mm_cvtsd_si64(t);
		#elif (defined WIN32 || defined _WIN32) && !defined WIN64 && !defined _WIN64 && defined _MSC_VER
			long t;
			__asm
			{
				fld value;
				fistp t;
			}
			return t;
		#elif defined HAVE_LRINT || defined __GNUC__
			return lrint(value);
		#else
			return static_cast<long>(value + 0.5);
		#endif
		}

		/** Rounds toward zero  */
		template <typename T>
		inline int fix(T x) { return  x>0 ? static_cast<int>(floor(static_cast<double>(x))) : static_cast<int>(ceil(static_cast<double>(x))) ; }

		/** Inline function for the square of a number. */
		template<class T>
		inline T square(const T x)    { return x*x; }

		/** Utility to get a cast'ed pointer from a smart pointer */
		template <class R, class P>
		inline R* getAs(stlplus::smart_ptr_clone<P> &o) { return static_cast<R*>( & (*o) ); }

		/** Utility to get a cast'ed pointer from a smart pointer */
		template <class R, class P>
		inline const R* getAs(const stlplus::smart_ptr_clone<P> &o) { return static_cast<const R*>( & (*o) ); }

		/** Reverse the order of the bytes of a given type (useful for transforming btw little/big endian)  */
		template <class T> inline void reverseBytesInPlace(T& v_in_out)
		{
			uint8_t *ptr = reinterpret_cast<uint8_t*>(&v_in_out);
			std::reverse(ptr,ptr+sizeof(T));
		}

		/** Reverse the order of the bytes of a given type (useful for transforming btw little/big endian)  */
		template <class T> inline void reverseBytes(const T &v_in, T& v_out)
		{
			v_out = v_in;
			reverseBytesInPlace(v_out);
		}


		/** If the second argument is below the first one, set the first argument to this lower value. */
		template <typename T,typename K>
		inline void keep_min(T &var,  const K test_val) {
			if (test_val<var) var = test_val;
		}

		/** If the second argument is above the first one, set the first argument to this higher value. */
		template <typename T,typename K>
		inline void keep_max(T &var,  const K test_val) {
			if (test_val>var) var = test_val;
		}

		/** Calls "delete" to free an object only if the pointer is not NULL, then set the pointer to NULL. */
		template <class T>
		void delete_safe(T *& ptr) {
			if (ptr) {
				delete ptr;
				ptr = NULL;
			}
		}

		/** Like calling a std::vector<>'s clear() method, but really forcing deallocating the memory. */
		template <class T>
		inline void vector_strong_clear(std::vector<T> & v) { std::vector<T> dummy; dummy.swap(v); }

	} // End of namespace
} // end of namespace

