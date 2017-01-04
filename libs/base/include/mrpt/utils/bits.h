/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config.h>

#define _USE_MATH_DEFINES // (For VS to define M_PI, etc. in cmath)
#include <cmath> // floor()
#include <string>
#include <mrpt/base/link_pragmas.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/mrpt_stdint.h>

/** This is the global namespace for all Mobile Robot Programming Toolkit (MRPT) libraries. */
namespace mrpt
{
	/** A std::string version of C sprintf.
	  *  You can call this to obtain a std::string using printf-like syntax.
	  *  Based on very nice code by Paul Senzee, published at http://senzee.blogspot.com/2006/05/c-formatting-stdstring.html
	  *  Function implemented in format.cpp
	  */
	std::string BASE_IMPEXP format(const char *fmt, ...) MRPT_printf_format_check(1,2);

	namespace math
	{
		bool BASE_IMPEXP isNaN(float v) MRPT_NO_THROWS;
		bool BASE_IMPEXP isNaN(double v) MRPT_NO_THROWS;
		bool BASE_IMPEXP isFinite(float v) MRPT_NO_THROWS;
		bool BASE_IMPEXP isFinite(double v) MRPT_NO_THROWS;

		// This inline function is used everywhere, so just move it here even it's not a forward declaration!
		/*! Returns the size of the matrix in the i'th dimension: 1=rows, 2=columns (MATLAB-compatible function)
		  *  \note Template argument MATRIXLIKE can be: mrpt::math::CMatrixTemplate, mrpt::math::CMatrixTemplateNumeric, mrpt::math::CMatrixFixedNumeric
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

#ifdef DEG2RAD  // functions are preferred over macros
#undef DEG2RAD
#endif
#ifdef RAD2DEG
#undef RAD2DEG
#endif
#if !defined(M_PI)
#	define M_PI 3.14159265358979323846
#endif

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

#define DEG2RAD DEG2RAD  // This is required to avoid other libs (like PCL) to #define their own versions of DEG2RAD
#define RAD2DEG RAD2DEG  // This is required to avoid other libs (like PCL) to #define their own versions of RAD2DEG

		/** Returns the sign of X as "1" or "-1" */
		template <typename T>
		inline int sign(T x) { return x<0 ? -1:1; }

		/** Returns the sign of X as "0", "1" or "-1" */
		template <typename T>
		inline int signWithZero(T x)	{ return (x==0 || x==-0)?0:sign(x);}

		/** Returns the lowest, possitive among two numbers. If both are non-positive (<=0), the lowest one is returned. */
		template <typename T>
		T lowestPositive(const T a, const T b)
		{
			if (a > 0 && a <= b)
				return a; // a positive and smaller than b
			else if (b > 0)
			     return b;  // b is positive and either smaller than a or a is negative
			else return a; // at least b is negative, we might not have an answer
		}

		/** Efficient and portable evaluation of the absolute difference of two unsigned integer values
		  * (but will also work for signed and floating point types) */
		template <typename T>
		inline T abs_diff(const T a, const T b) {
			return std::max(a,b) - std::min(a,b);
		}

		template<typename T> inline const T  min3(const T& A, const T& B,const T& C) { return std::min<T>(A, std::min<T>(B,C) ); }
		template<typename T> inline const T  max3(const T& A, const T& B,const T& C) { return std::max<T>(A, std::max<T>(B,C) ); }

		/** Rounds toward zero  */
		template <typename T>
		inline int fix(T x) { return  x>0 ? static_cast<int>(floor(static_cast<double>(x))) : static_cast<int>(ceil(static_cast<double>(x))) ; }

		/** Inline function for the square of a number. */
		template<class T>
		inline T square(const T x)    { return x*x; }

		/** Utility to get a cast'ed pointer from a smart pointer */
		template <class R, class SMART_PTR>
		inline R* getAs(SMART_PTR &o) { return static_cast<R*>( & (*o) ); }

		/** Utility to get a cast'ed pointer from a smart pointer */
		template <class R, class SMART_PTR>
		inline const R* getAs(const SMART_PTR &o) { return static_cast<const R*>( & (*o) ); }

		/** Reverse the order of the bytes of a given type (useful for transforming btw little/big endian)  */
		void BASE_IMPEXP reverseBytesInPlace(bool& v_in_out);
		void BASE_IMPEXP reverseBytesInPlace(uint8_t& v_in_out);
		void BASE_IMPEXP reverseBytesInPlace(int8_t& v_in_out);
		void BASE_IMPEXP reverseBytesInPlace(uint16_t& v_in_out);
		void BASE_IMPEXP reverseBytesInPlace(int16_t& v_in_out);
		void BASE_IMPEXP reverseBytesInPlace(uint32_t& v_in_out);
		void BASE_IMPEXP reverseBytesInPlace(int32_t& v_in_out);
		void BASE_IMPEXP reverseBytesInPlace(uint64_t& v_in_out);
		void BASE_IMPEXP reverseBytesInPlace(int64_t& v_in_out);
		void BASE_IMPEXP reverseBytesInPlace(float& v_in_out);
		void BASE_IMPEXP reverseBytesInPlace(double& v_in_out);
#ifdef HAVE_LONG_DOUBLE
		void BASE_IMPEXP reverseBytesInPlace(long double& v_in_out);
#endif

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
		/** Saturate the value of var (the variable gets modified) so it does not get out of [min,max]. */
		template <typename T>
		inline void saturate(T &var, const T sat_min, const T sat_max) {
			if (var>sat_max) var = sat_max;
			if (var<sat_min) var = sat_min;
		}
		/** Like saturate() but it returns the value instead of modifying the variable */
		template <typename T>
		inline T saturate_val(const T &value, const T sat_min, const T sat_max) {
			T var=value;
			if (var>sat_max) var = sat_max;
			if (var<sat_min) var = sat_min;
			return var;
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
		template <class VECTOR_T>
		inline void vector_strong_clear(VECTOR_T & v) { VECTOR_T dummy; dummy.swap(v); }

		/** Returns the smaller number >=len such that it's a multiple of 4 */
		template <typename T>
		T length2length4N(T len) {
			if (0!=(len & 0x03)) len+= (4 - (len & 0x03));
			return len;
		}

#define SELBYTE0(v)  (v & 0xff)
#define SELBYTE1(v)  ((v>>8) & 0xff)
#define SELBYTE2(v)  ((v>>16) & 0xff)
#define SELBYTE3(v)  ((v>>24) & 0xff)

#define MAKEWORD16B(__LOBYTE,__HILOBYTE)  ((__LOBYTE) | ((__HILOBYTE)<<8))
#define MAKEWORD32B(__LOWORD16,__HIWORD16)  ((__LOWORD16) | ((__HIWORD16)<<16))
#define MAKEWORD64B(__LOWORD32,__HIWORD32)  ((__LOWORD32) | ((__HIWORD32)<<32))

	} // End of namespace
} // end of namespace
