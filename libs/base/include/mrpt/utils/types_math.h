/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <vector>  // For <Eigen/StdVector>
#include <deque>   // For <Eigen/StdDeque>

// needed here for a few basic types used in Eigen MRPT's plugin:
#include <mrpt/math/math_frwds.h>

// --------------------------------------------------
// Include the Eigen3 library headers, including
//  MRPT's extensions:
// --------------------------------------------------
#include <fstream> // These headers are assumed by <mrpt/math/eigen_plugins.h>:
#include <ctime>
#ifdef EIGEN_MATRIXBASE_H
#	error **FATAL ERROR**: MRPT headers must be included before <Eigen/Dense> headers.
#endif
#ifndef EIGEN_USE_NEW_STDVECTOR
#  define EIGEN_USE_NEW_STDVECTOR
#endif
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
	typedef Eigen::Matrix<float,Eigen::Dynamic,1>  vector_float;
	typedef Eigen::Matrix<double,Eigen::Dynamic,1> vector_double;

	namespace utils
	{
		class CStream;

		CStream BASE_IMPEXP & operator<<(CStream&s, const vector_float  &a);
		CStream BASE_IMPEXP & operator<<(CStream&s, const vector_double &a);
		CStream BASE_IMPEXP & operator>>(CStream&in, vector_double &a);
		CStream BASE_IMPEXP & operator>>(CStream&in, vector_float &a);
	}
}
