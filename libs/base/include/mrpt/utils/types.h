/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_utils_types_H
#define mrpt_utils_types_H

#include <vector>  // For <Eigen/StdVector>
#include <deque>   // For <Eigen/StdDeque>
#include <string>
#include <stdexcept>

#include <mrpt/utils/mrpt_stdint.h>    // compiler-independent version of "stdint.h"
//#include <mrpt/utils/mrpt_inttypes.h>  // compiler-independent version of "inttypes.h"

// needed here for a few basic types used in Eigen MRPT's plugin:
#include <mrpt/math/math_frwds.h>

// --------------------------------------------------
// Include the Eigen3 library headers, including
//  MRPT's extensions:
// --------------------------------------------------
#include <iostream> // These headers are assumed by <mrpt/math/eigen_plugins.h>:
#include <fstream>
//#include <sstream>
#ifdef EIGEN_MAJOR_VERSION
#	error **FATAL ERROR**: MRPT headers must be included before Eigen headers.
#endif
#ifndef EIGEN_USE_NEW_STDVECTOR
#  define EIGEN_USE_NEW_STDVECTOR
#endif
#include <Eigen/Core>  // Was: <Eigen/Dense>
#include <Eigen/Eigenvalues>
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
	/** The base class of MRPT vectors, actually, Eigen column matrices of dynamic size with specialized constructors that resemble std::vector.
         * \note For a summary and classification of all MRPT vector, array and matrix classes see: http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
	 * \ingroup mrpt_base_grp
	 */
	template <typename T>
	struct dynamicsize_vector : public Eigen::Matrix<T,Eigen::Dynamic,1>
	{
		typedef T value_type;
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
		inline void resize(const size_t N, const T default_val) { if (static_cast<size_t>(Base::derived().size())!=N) Base::derived().resize(N,1); Base::derived().setConstant(default_val); }
		/** Normal resize of the vector (preserving old contents). */
		inline void resize(const size_t N) { if (static_cast<size_t>(Base::derived().size())!=N) Base::derived().conservativeResize(N); }
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

	namespace utils
	{
		/** For performing type casting from a pointer to its numeric value.
		*/
		#if defined(_MSC_VER) && (_MSC_VER>=1300)
			typedef unsigned long long POINTER_TYPE;
		#else
			typedef unsigned long POINTER_TYPE;
		#endif

		typedef uint64_t TNodeID;  //!< The type for node IDs in graphs of different types.
		typedef std::pair<TNodeID,TNodeID> TPairNodeIDs; //!< A pair of node IDs.
		#define INVALID_NODEID  static_cast<mrpt::utils::TNodeID>(-1)

	} // end namespace
}

#endif

