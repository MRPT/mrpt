/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

		/** Efficient and portable evaluation of the absolute difference of two unsigned integer values 
		  * (but will also work for signed and floating point types) */
		template <typename T> 
		inline T abs_diff(const T a, const T b) {
			return std::max(a,b) - std::min(a,b);
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

