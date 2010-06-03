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

#ifndef mrpt_utils_types_H
#define mrpt_utils_types_H

#include <vector>
#include <list>
#include <set>
#include <map>
#include <string>
#include <stdexcept>

// Standard elemental types:
#include "pstdint.h"  // The "portable stdint header file"

#define __STDC_FORMAT_MACROS
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

//!< See ops_containers.h
#define DECLARE_MRPT_CONTAINER_TYPES \
		template <class ANOTHERCONT> struct mrpt_container { \
			typedef void void_type; \
			typedef ANOTHERCONT dum_type; \
			typedef mrpt_autotype cont_type; \
			typedef typename mrpt_autotype::value_type element_type; \
		};

#define COMMON_DECLARE_MRPT_CONTAINER_IS_MATRIX \
		typedef void mrpt_matrix_tag; \
		template <class ANOTHERCONT> struct mrpt_matrix_tag_templ { typedef void void_type; typedef ANOTHERCONT dum_type; typedef mrpt_autotype cont_type; \
		};

#define DECLARE_MRPT_CONTAINER_IS_MATRIX \
		COMMON_DECLARE_MRPT_CONTAINER_IS_MATRIX \
		enum dummy_enum {  mrpt_matrix_type_ncols = -1, \
				mrpt_matrix_type_nrows = -1 }; \

#define DECLARE_MRPT_CONTAINER_IS_MATRIX_FIXED(NROWS,NCOLS) \
		COMMON_DECLARE_MRPT_CONTAINER_IS_MATRIX \
		enum dummy_enum {  mrpt_matrix_type_ncols = NCOLS, \
				mrpt_matrix_type_nrows = NROWS };


#define DECLARE_MRPT_CONTAINER_IS_VECTOR \
		typedef void mrpt_vector_tag; \
		template <class ANOTHERCONT> struct mrpt_vector_tag_templ { typedef void void_type; typedef ANOTHERCONT dum_type; typedef mrpt_autotype cont_type; }; \
		enum dummy_enum {  mrpt_vector_type_len = -1 }; \

#define DECLARE_MRPT_CONTAINER_IS_VECTOR_FIXED(LEN) \
		typedef void mrpt_vector_tag; \
		template <class ANOTHERCONT> struct mrpt_vector_tag_templ { typedef void void_type; typedef ANOTHERCONT dum_type; typedef mrpt_autotype cont_type; }; \
		enum dummy_enum {  mrpt_vector_type_len = LEN }; \

/** A metaprogramming approach to assure that one type is a MRPT containers, to limit the scope of generic templates.
  *  If the type fulfills the condition, the macro expands into a simple "void" which can be used in function declarations. */
#define RET_VOID_ASSERT_MRPTCONTAINER(_CONTAINER) \
	typename _CONTAINER::template mrpt_container<void>::void_type

/** A metaprogramming approach to assure that two types are MRPT containers, to limit the scope of generic templates.
  *  If both types fulfill the condition, the macro expands into a simple "void" which can be used in function declarations. */
#define RET_VOID_ASSERT_MRPTCONTAINERS(_CONTAINER1,_CONTAINER2) \
	typename _CONTAINER1::template mrpt_container<typename _CONTAINER2::mrpt_autotype>::void_type

/** A metaprogramming approach to assure that two types are MRPT containers, to limit the scope of generic templates.
  *  If both types fulfill the condition, the macro expands into a "_CONTAINER1" type which can be used in function declarations. */
#define RET_CONT1_ASSERT_MRPTCONTAINERS(_CONTAINER1,_CONTAINER2) \
	typename _CONTAINER1::template mrpt_container<typename _CONTAINER2::mrpt_autotype>::cont_type

/** A metaprogramming approach to assure that two types are MRPT containers, to limit the scope of generic templates.
  *  If both types fulfill the condition, the macro expands into the type of "_CONTAINER1"'s elements, which can be used in function declarations. */
#define RET_ELEMENT_ASSERT_MRPTCONTAINERS(_CONTAINER1,_CONTAINER2) \
	typename _CONTAINER1::template mrpt_container<typename _CONTAINER2::mrpt_autotype>::element_type

/** A metaprogramming approach to assure that one type is a MRPT container, to limit the scope of generic templates.
  *  If the type fulfills the condition, the macro expands into a "_CONTAINER" type which can be used in function declarations. */
#define RET_CONT_ASSERT_MRPTCONTAINER(_CONTAINER) \
	typename _CONTAINER::mrpt_autotype

/** A metaprogramming approach to assure that one type is a MRPT container, to limit the scope of generic templates.
  *  If the type fulfills the condition, the macro expands into the type of each "_CONTAINER"'s elements, which can be used in function declarations. */
#define RET_ELEMENT_ASSERT_MRPTCONTAINER(_CONTAINER) \
	typename _CONTAINER::mrpt_autotype::value_type

/** A metaprogramming approach to assure that one type is a MRPT container, to limit the scope of generic templates.
  *  If the type fulfills the condition, the macro expands into the type "_RETTYPE", which can be used in function declarations. */
#define RET_TYPE_ASSERT_MRPTCONTAINER(_CONTAINER,_RETTYPE) \
	typename _CONTAINER::template mrpt_container<_RETTYPE>::dum_type

/** A metaprogramming approach to assure that a type is a MRPT matrix, to limit the scope of generic templates.
  *  If the type fulfills the condition, the macro expands into a simple "void" which can be used in function declarations. */
#define RET_VOID_ASSERT_MRPTMATRIX(_MATRIX) \
	typename _MATRIX::template mrpt_matrix_tag_templ<void>::void_type

/** A metaprogramming approach to assure that a type is a MRPT matrix, to limit the scope of generic templates.
  *  If the type fulfills the condition, the macro expands into the type "_RETTYPE", which can be used in function declarations. */
#define RET_TYPE_ASSERT_MRPTMATRIX(_MATRIX,_RETTYPE) \
	typename _MATRIX::template mrpt_matrix_tag_templ<_RETTYPE>::dum_type

/** A metaprogramming approach to assure that a type is a MRPT matrix, to limit the scope of generic templates.
  *  If both types fulfill the condition, the macro expands into the type of _MATRIX which can be used in function declarations. */
#define RET_MAT_ASSERT_MRPTMATRIX(_MATRIX) \
	typename _MATRIX::template mrpt_matrix_tag_templ<void>::cont_type

/** A metaprogramming approach to assure that two types are MRPT matrices, to limit the scope of generic templates.
  *  If both types fulfill the condition, the macro expands into a simple "void" which can be used in function declarations. */
#define RET_VOID_ASSERT_MRPTMATRICES(_MATRIX1,_MATRIX2) \
	typename _MATRIX1::template mrpt_matrix_tag_templ<typename _MATRIX2::mrpt_matrix_tag>::void_type

/** A metaprogramming approach to assure that two types are MRPT matrices, to limit the scope of generic templates.
  *  If both types fulfill the condition, the macro expands into the type of _MATRIX1 which can be used in function declarations. */
#define RET_MAT1_ASSERT_MRPTMATRICES(_MATRIX1,_MATRIX2) \
	typename _MATRIX1::template mrpt_matrix_tag_templ<typename _MATRIX2::mrpt_matrix_tag>::cont_type



/** A metaprogramming approach to assure that two types are MRPT vectors, to limit the scope of generic templates.
  *  If both types fulfill the condition, the macro expands into a "_CONTAINER1" type which can be used in function declarations. */
#define RET_CONT1_ASSERT_MRPTVECTORS(_VECTOR1,_VECTOR2) \
	typename _VECTOR1::template mrpt_vector_tag_templ<typename _VECTOR2::mrpt_vector_tag>::cont_type


// needed here for DECLARE_COMMON_CONTAINERS_MEMBERS
#include <mrpt/math/math_frwds.h>


namespace mrpt
{
	/** Numeric vectors compatible with mrpt/math/ops_containers.h:  */
	template <typename _TYPE>
	struct mrpt_base_vector : public std::vector<_TYPE>
	{
		inline mrpt_base_vector() : std::vector<_TYPE>() {}
		inline mrpt_base_vector(const std::vector<_TYPE> &o) : std::vector<_TYPE>(o) {}
		inline mrpt_base_vector<_TYPE> & operator =(const std::vector<_TYPE> &o) { std::vector<_TYPE>::operator =(o); return *this; }
		inline mrpt_base_vector(size_t N) : std::vector<_TYPE>(N) { }
		inline mrpt_base_vector(size_t N,_TYPE val) : std::vector<_TYPE>(N,val) { }
		template<typename ITERATOR> inline mrpt_base_vector(const ITERATOR &b,const ITERATOR &e):std::vector<_TYPE>(b,e)	{}
		typedef mrpt_base_vector<_TYPE> mrpt_autotype;
		DECLARE_MRPT_CONTAINER_TYPES
		DECLARE_MRPT_CONTAINER_IS_VECTOR
		DECLARE_COMMON_CONTAINERS_MEMBERS(_TYPE)
	};

	typedef mrpt_base_vector<float>		vector_float;
	typedef mrpt_base_vector<double>	vector_double;

	typedef mrpt_base_vector<int8_t>	vector_signed_byte;
	typedef mrpt_base_vector<int16_t>	vector_signed_word;
	typedef mrpt_base_vector<int32_t>	vector_int;
	typedef mrpt_base_vector<int64_t>	vector_long;
	typedef mrpt_base_vector<size_t>	vector_size_t;
	typedef mrpt_base_vector<uint8_t>	vector_byte;
	typedef mrpt_base_vector<uint16_t>	vector_word;
	typedef mrpt_base_vector<uint32_t>	vector_uint;

	typedef std::vector<bool> 			vector_bool;	//!<  A type for passing a vector of bools.
	typedef std::vector<std::string> 	vector_string;	//!<  A type for passing a vector of strings.

	namespace utils
	{
		// Functors:   Ret: NO  In:1-3
		typedef void (*TFunctor_noRet_1inputs)(const void *);	//!< A generic functor type for functions accepting 1 input arguments and returning nothing.
		typedef void (*TFunctor_noRet_2inputs)(const void *,const void *);	//!< A generic functor type for functions accepting 2 input arguments and returning nothing.
		typedef void (*TFunctor_noRet_3inputs)(const void *,const void *,const void *);	//!< A generic functor type for functions accepting 3 input arguments and returning nothing.
		// Functors:  Ret: double  In:1-3
		typedef double (*TFunctor_retDouble_1inputs)(const void *);	//!< A generic functor type for functions accepting 1 input arguments and returning a double value.
		typedef double (*TFunctor_retDouble_2inputs)(const void *,const void *);	//!< A generic functor type for functions accepting 2 input arguments and returning a double value.
		typedef double (*TFunctor_retDouble_3inputs)(const void *,const void *,const void *);	//!< A generic functor type for functions accepting 3 input arguments and returning a double value.
		// Functors:  Ret: vector  In: vector
		typedef void (*TFunctor_retVecDbl_inpVecDbl)(const vector_double &in, vector_double &out);	//!< A generic functor type for functions accepting 1 vector and returning 1 vector.
		typedef void (*TFunctor_retVecFlt_inpVecFlt)(const vector_float &in, vector_float &out);	//!< A generic functor type for functions accepting 1 vector and returning 1 vector.
		typedef void (*TFunctor_retVecInt_inpVecInt)(const vector_int &in, vector_int &out);	//!< A generic functor type for functions accepting 1 vector and returning 1 vector.
		// Functors:  Ret: vector  In: 2 x vector
		typedef void (*TFunctor_retVecDbl_inp2VecDbl)(const vector_double &x,const vector_double &y, vector_double &out);	//!< A generic functor type for functions accepting 2 vectors and returning 1 vector.
		typedef void (*TFunctor_retVecFlt_inp2VecFlt)(const vector_float &x,const vector_float &y, vector_float &out);		//!< A generic functor type for functions accepting 2 vectors and returning 1 vector.
		typedef void (*TFunctor_retVecInt_inp2VecInt)(const vector_int &x,const vector_int &y, vector_int &out);	//!< A generic functor type for functions accepting 2 vectors and returning 1 vector.
		// Functors:  Ret: double  In: vectors
		typedef double (*TFunctor_retDbl_inp1VecDbl)(const vector_double &in1);	//!< A generic functor type for functions accepting 1 vector and returning 1 double.
		typedef double (*TFunctor_retDbl_inp2VecDbl)(const vector_double &in1,const vector_double &in2);	//!< A generic functor type for functions accepting 2 vectors and returning 1 double.
		typedef double (*TFunctor_retDbl_inp3VecDbl)(const vector_double &in1,const vector_double &in2,const vector_double &in3);	//!< A generic functor type for functions accepting 3 vectors and returning 1 double.

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
			TColor(uint8_t r=0,uint8_t g=0,uint8_t b=0, uint8_t alpha=255) : R(r),G(g),B(b),A(alpha) { }
			uint8_t R,G,B,A;
			/** Operator for implicit conversion into an int binary representation 0xRRGGBB */
			operator int(void) const { return (((int)R)<<16) | (((int)G)<<8) | B; }

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
			float R,G,B,A;
		};

		/** A pair (x,y) of pixel coordinates (subpixel resolution). */
		struct BASE_IMPEXP TPixelCoordf
		{
			double x,y;

			/** Default constructor: undefined values of x,y */
			TPixelCoordf() : x(),y() {}

			/** Constructor from x,y values */
			TPixelCoordf(const double _x,const double _y) : x(_x), y(_y) { }
		};

		/** A pair (x,y) of pixel coordinates (integer resolution). */
		struct BASE_IMPEXP TPixelCoord
		{
			TPixelCoord() : x(0),y(0) { }
			TPixelCoord(const int _x,const int _y) : x(_x), y(_y) { }

			int x,y;
		};

		typedef TPixelCoord TImageSize; //!< A type for image sizes.

		/** For usage when passing a dynamic number of (numeric) arguments to a function, by name.
		  *  \code
		  *    TParameters<double> p;
		  *    p["v_max"] = 1.0;  // Write
		  *    ...
		  *    cout << p["w_max"]; // Read, even if "p" is const.
		  *  \endcode
		  */
		template <typename T>
		struct TParameters : public std::map<std::string,T>
		{
			inline bool has(const std::string &s) const { return std::map<std::string,T>::end()!=std::map<std::string,T>::find(s); }
			/** A const version of the [] operator, for usage as read-only.
			  * \exception std::logic_error On parameter not present. Please, check existence with "has" before reading.
			  */
			T operator[](const std::string &s) const {
				typename std::map<std::string,T>::const_iterator it =std::map<std::string,T>::find(s);
				if (std::map<std::string,T>::end()==it)
					throw std::logic_error(std::string("Parameter '")+s+std::string("' is not present.").c_str());
				return it->second;
			}
			/** The write (non-const) version of the [] operator. */
			inline T & operator[](const std::string &s) { return std::map<std::string,T>::operator[](s); }
		};
	} // end namespace
}

#endif

