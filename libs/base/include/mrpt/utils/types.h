/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
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

#ifndef mrpt_utils_types_H
#define mrpt_utils_types_H

#include <vector>
#include <list>
#include <set>
#include <map>
#include <string>
#include <stdexcept>
#include <cstdarg>
#include <iostream>
#include <sstream>

#include <ctime>

// Define macros in platform dependant stdint.h header:
#ifndef __STDC_FORMAT_MACROS
#	define __STDC_FORMAT_MACROS
#endif
#ifndef __STDC_CONSTANT_MACROS
#	define __STDC_CONSTANT_MACROS
#endif
#ifndef __STDC_LIMIT_MACROS
#	define __STDC_LIMIT_MACROS
#endif

// Standard elemental types:
#include "pstdint.h"  // The "portable stdint header file"

#if HAVE_INTTYPES_H
#	include <inttypes.h>
#elif defined(_MSC_VER)
#	include	<mrpt/utils/msvc_inttypes.h>
#endif

// SSE2, SSE3 types:
#if MRPT_HAS_SSE2
	#include <emmintrin.h>
	#include <mmintrin.h>
#endif

// needed here for a few basic types used in Eigen MRPT's plugin:
#include <mrpt/math/math_frwds.h>

// --------------------------------------------------
// Include the Eigen3 library headers, including
//  MRPT's extensions:
// --------------------------------------------------
#include <iostream> // These headers are assumed by <mrpt/math/eigen_plugins.h>:
#include <fstream>
#include <sstream>
#ifdef EIGEN_MAJOR_VERSION
#	error **FATAL ERROR**: MRPT headers must be included before Eigen headers.
#endif
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/StdDeque>

#if !EIGEN_VERSION_AT_LEAST(2,90,0)
#error MRPT needs version 3.0.0-beta of Eigen or newer
#endif

// Template implementations that need to be after all Eigen includes:
#include EIGEN_MATRIXBASE_PLUGIN_POST_IMPL
// --------------------------------------------------
//  End of Eigen includes
// --------------------------------------------------


// This must be put inside any MRPT class that inherits from an Eigen class:
#define MRPT_EIGEN_DERIVED_CLASS_CTOR_OPERATOR_EQUAL(_CLASS_) \
	/*! Assignment operator from any other Eigen class */ \
    template<typename OtherDerived> \
    inline mrpt_autotype & operator= (const Eigen::MatrixBase <OtherDerived>& other) { \
        /*Base::operator=(other.template cast<typename Base::Scalar>());*/ \
        Base::operator=(other); \
        return *this; \
    } \
	/*! Constructor from any other Eigen class */ \
    template<typename OtherDerived> \
	inline _CLASS_(const Eigen::MatrixBase <OtherDerived>& other) : Base(other.template cast<typename Base::Scalar>()) { } \

namespace mrpt
{
	/** The base class of MRPT vectors, actually, Eigen column matrices of dynamic size with specialized constructors that resemble std::vector. */
	template <typename T>
	struct dynamicsize_vector : public Eigen::Matrix<T,Eigen::Dynamic,1>
	{
		typedef Eigen::Matrix<T,Eigen::Dynamic,1> Base;
		typedef dynamicsize_vector<T> mrpt_autotype;
		MRPT_EIGEN_DERIVED_CLASS_CTOR_OPERATOR_EQUAL(dynamicsize_vector)

		/** Default constructor: empty vector */
		inline dynamicsize_vector() : Base() {}
		/** Constructor, initializes to a given initial size */
		inline dynamicsize_vector(size_t N) : Base(N,1) { Base::derived().setZero(); }
		/** Constructor, initializes to a given initial size, all elements to a given value */
		inline dynamicsize_vector(size_t N, T init_val) : Base(N,1) { Base::derived().assign(init_val); }
		/** Constructor, initializes from a std::vector<> of scalars */
		template <typename R>
		inline dynamicsize_vector(const std::vector<R>& v) : Base(v.size(),1) { for (size_t i=0;i<v.size();i++) (*this)[i]=v[i]; }
		/** Overloaded resize method that mimics std::vector::resize(SIZE,DEFAULT_VALUE) instead of resize(nrows,ncols) \note This method exists for backward compatibility in MRPT  */
		inline void resize(const size_t N, const T default_val) { Base::derived().resize(N,1); Base::derived().setConstant(default_val); }
		/** Normal resize of the vector (preserving old contents). */
		inline void resize(const size_t N) { Base::derived().conservativeResize(N); }
		/** Reset the vector to a 0-length */
		inline void clear() { *this = dynamicsize_vector<T>(); }
		/** DOES NOTHING (it's here for backward compatibility) */
		inline void reserve(size_t dummy_size) { }
	};

	typedef dynamicsize_vector<float>  vector_float;
	typedef dynamicsize_vector<double> vector_double;

	typedef std::vector<int8_t>      vector_signed_byte;
	typedef std::vector<int16_t>     vector_signed_word;
	typedef std::vector<int32_t>     vector_int;
	typedef std::vector<int64_t>     vector_long;
	typedef std::vector<size_t>      vector_size_t;
	typedef std::vector<uint8_t>     vector_byte;
	typedef std::vector<uint16_t>    vector_word;
	typedef std::vector<uint32_t>	 vector_uint;
	typedef std::vector<bool>        vector_bool;	//!<  A type for passing a vector of bools.
	typedef std::vector<std::string> vector_string;	//!<  A type for passing a vector of strings.

	/** Helper types for STL containers with Eigen memory allocators. */
	template <class TYPE1,class TYPE2=TYPE1>
	struct aligned_containers
	{
		typedef std::pair<TYPE1,TYPE2> pair_t;
		typedef std::vector<TYPE1, Eigen::aligned_allocator<TYPE1> > vector_t;
		typedef std::deque<TYPE1, Eigen::aligned_allocator<TYPE1> > deque_t;
		typedef std::map<TYPE1,TYPE2,std::less<TYPE1>,Eigen::aligned_allocator<std::pair<const TYPE1,TYPE2> > > map_t;
		typedef std::multimap<TYPE1,TYPE2,std::less<TYPE1>,Eigen::aligned_allocator<std::pair<const TYPE1,TYPE2> > > multimap_t;
	};

	namespace utils
	{
		/** For performing type casting from a pointer to its numeric value.
		*/
		#if defined(_MSC_VER) && (_MSC_VER>=1300)
			typedef unsigned long long POINTER_TYPE;
		#else
			typedef unsigned long POINTER_TYPE;
		#endif

		/** A RGB color - 8bit */
		struct BASE_IMPEXP TColor
		{
			inline TColor() : R(0),G(0),B(0),A(255) { }
			inline TColor(uint8_t r,uint8_t g,uint8_t b, uint8_t alpha=255) : R(r),G(g),B(b),A(alpha) { }
			inline explicit TColor(const unsigned int color_RGB_24bit) : R(uint8_t(color_RGB_24bit>>16)),G(uint8_t(color_RGB_24bit>>8)),B(uint8_t(color_RGB_24bit)),A(255) { }
			uint8_t R,G,B,A;
			/** Operator for implicit conversion into an int binary representation 0xRRGGBB */
			inline operator unsigned int(void) const { return (((unsigned int)R)<<16) | (((unsigned int)G)<<8) | B; }

			static TColor red; //!< Predefined colors
			static TColor green;//!< Predefined colors
			static TColor blue;//!< Predefined colors
			static TColor white;//!< Predefined colors
			static TColor black;//!< Predefined colors
			static TColor gray;	//!< Predefined colors
		};

		/** A RGB color - floats in the range [0,1] */
		struct BASE_IMPEXP TColorf
		{
			TColorf(float r=0,float g=0,float b=0, float alpha=1.0f) : R(r),G(g),B(b),A(alpha) { }
			explicit TColorf(const TColor &col) : R(col.R*(1.f/255)),G(col.G*(1.f/255)),B(col.B*(1.f/255)),A(col.A*(1.f/255)) { }
			float R,G,B,A;
		};

		/** A pair (x,y) of pixel coordinates (subpixel resolution). */
		struct BASE_IMPEXP TPixelCoordf
		{
			float x,y;

			/** Default constructor: undefined values of x,y */
			TPixelCoordf() : x(),y() {}

			/** Constructor from x,y values */
			TPixelCoordf(const float _x,const float _y) : x(_x), y(_y) { }
		};

		std::ostream BASE_IMPEXP & operator <<(std::ostream& o, const TPixelCoordf& p); //!< Prints TPixelCoordf as "(x,y)"

		/** A pair (x,y) of pixel coordinates (integer resolution). */
		struct BASE_IMPEXP TPixelCoord
		{
			TPixelCoord() : x(0),y(0) { }
			TPixelCoord(const int _x,const int _y) : x(_x), y(_y) { }

			int x,y;
		};

		std::ostream BASE_IMPEXP & operator <<(std::ostream& o, const TPixelCoord& p); //!< Prints TPixelCoord as "(x,y)"

		typedef TPixelCoord TImageSize; //!< A type for image sizes.

		/** For usage when passing a dynamic number of (numeric) arguments to a function, by name.
		  *  \code
		  *    TParameters<double> p;  // or TParametersDouble
		  *    p["v_max"] = 1.0;  // Write
		  *    ...
		  *    cout << p["w_max"]; // Read, even if "p" was const.
		  *  \endcode
		  *
		  *  A default list of parameters can be passed to the constructor as a sequence
		  *   of pairs "name, value", which MUST end in a NULL name string. Names MUST BE "const char*"
		  *   (that is, "old plain strings" are OK), not std::string objects!.
		  *  See this example:
		  *
		  *  \code
		  *    TParameters<double> p("par1",2.0, "par2",-4.5, "par3",9.0, NULL); // MUST end with a NULL
		  *  \endcode
		  *
		  *  <b>VERY IMPORTANT:</b> If you use the NULL-ended constructor above, make sure all the values are of the proper
		  *    type or it will crash in runtime. For example, in a TParametersDouble all values must be double's, so
		  *    if you type "10" the compiler will make it an "int". Instead, write "10.0".
		  *
		  * \sa the example in MRPT/samples/params-by-name
		  */
		template <typename T>
		struct TParameters : public std::map<std::string,T>
		{
			typedef std::map<std::string,T> BASE;
			/** Default constructor (initializes empty) */
			TParameters() : BASE() { }
			/** Constructor with a list of initial values (see the description and use example in mrpt::utils::TParameters) */
			TParameters(const char *nam1,...) : BASE() {
				if (!nam1) return; // No parameters
				T val;
				va_list args;
				va_start(args,nam1);
				// 1st one out of the loop:
				val = va_arg(args,T);
				BASE::operator[](std::string(nam1)) = val;
				// Loop until NULL:
				const char *nam;
				do{
					nam = va_arg(args,const char*);
					if (nam) {
						val = va_arg(args,T);
						BASE::operator[](std::string(nam)) = val;
					}
				} while (nam);
				va_end(args);
			}
			inline bool has(const std::string &s) const { return std::map<std::string,T>::end()!=BASE::find(s); }
			/** A const version of the [] operator, for usage as read-only.
			  * \exception std::logic_error On parameter not present. Please, check existence with "has" before reading.
			  */
			inline T operator[](const std::string &s) const {
				typename BASE::const_iterator it =BASE::find(s);
				if (BASE::end()==it)
					throw std::logic_error(std::string("Parameter '")+s+std::string("' is not present.").c_str());
				return it->second;
			}
			/** A const version of the [] operator and with a default value in case the parameter is not set (for usage as read-only).
			  */
			inline T getWithDefaultVal(const std::string &s, const T& defaultVal) const {
				typename BASE::const_iterator it =BASE::find(s);
				if (BASE::end()==it)
						return defaultVal;
				else 	return it->second;
			}
			/** The write (non-const) version of the [] operator. */
			inline T & operator[](const std::string &s) { return BASE::operator[](s); }

			/** Dumps to console the output from getAsString() */
			inline void dumpToConsole() const { std::cout << getAsString(); }

			/** Returns a multi-like string representation of the parameters like : 'nam   = val\nnam2   = val2...' */
			inline std::string getAsString() const { std::string s; getAsString(s); return s; }

			/** Returns a multi-like string representation of the parameters like : 'nam   = val\nnam2   = val2...' */
			void getAsString(std::string &s) const {
				size_t maxStrLen = 10;
				for (typename BASE::const_iterator it=BASE::begin();it!=BASE::end();++it) maxStrLen = std::max(maxStrLen, it->first.size() );
				maxStrLen++;
				std::stringstream str;
				for (typename BASE::const_iterator it=BASE::begin();it!=BASE::end();++it)
					str << it->first << std::string(maxStrLen-it->first.size(),' ') << " = " << it->second << std::endl;
				s = str.str();
			}
		};

		typedef TParameters<double>       TParametersDouble; //!< See the generic template mrpt::utils::TParameters
		typedef TParameters<std::string>  TParametersString; //!< See the generic template mrpt::utils::TParameters

		typedef uint64_t TNodeID;  //!< The type for node IDs in graphs of different types.
		typedef std::pair<TNodeID,TNodeID> TPairNodeIDs; //!< A pair of node IDs.
		#define INVALID_NODEID  static_cast<TNodeID>(-1)

	} // end namespace
}

#endif

