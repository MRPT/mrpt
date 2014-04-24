/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

// Eigen forward declarations:
#include <mrpt/config.h>

//#include <Eigen/Dense>
#include <Eigen/src/Core/util/DisableStupidWarnings.h>
#include <Eigen/src/Core/util/Macros.h>
#include <complex> // Needed by hdrs below

#define STAGE99_NO_EIGEN2_SUPPORT           99
#define EIGEN2_SUPPORT_STAGE STAGE99_NO_EIGEN2_SUPPORT

#if !EIGEN_VERSION_AT_LEAST(3,1,0)
namespace Eigen {
#endif
// These headers were assumed to lie inside namespace Eigen{} in Eigen <=3.1.0
#	include <Eigen/src/Core/util/Constants.h>
#	include <Eigen/src/Core/util/ForwardDeclarations.h>

#if !EIGEN_VERSION_AT_LEAST(3,1,0)
}
#endif

#undef STAGE99_NO_EIGEN2_SUPPORT
#undef EIGEN2_SUPPORT_STAGE

namespace mrpt
{
    namespace math
    {
        // Dynamic size:
        template <class T> class CMatrixTemplateNumeric;
		typedef CMatrixTemplateNumeric<float> CMatrixFloat;
        typedef CMatrixTemplateNumeric<double> CMatrixDouble;
		template <typename T> class dynamic_vector;
		typedef dynamic_vector<float>  CVectorFloat;
		typedef dynamic_vector<double> CVectorDouble;


        // Fixed size:
        template <typename T,size_t NROWS,size_t NCOLS> class CMatrixFixedNumeric;

		/** @name Typedefs for common sizes
			@{ */
		typedef CMatrixFixedNumeric<double,2,2> CMatrixDouble22;
		typedef CMatrixFixedNumeric<double,2,3> CMatrixDouble23;
		typedef CMatrixFixedNumeric<double,3,2> CMatrixDouble32;
		typedef CMatrixFixedNumeric<double,3,3> CMatrixDouble33;
		typedef CMatrixFixedNumeric<double,4,4> CMatrixDouble44;
		typedef CMatrixFixedNumeric<double,6,6> CMatrixDouble66;
		typedef CMatrixFixedNumeric<double,7,7> CMatrixDouble77;
		typedef CMatrixFixedNumeric<double,1,3> CMatrixDouble13;
		typedef CMatrixFixedNumeric<double,3,1> CMatrixDouble31;
		typedef CMatrixFixedNumeric<double,1,2> CMatrixDouble12;
		typedef CMatrixFixedNumeric<double,2,1> CMatrixDouble21;
		typedef CMatrixFixedNumeric<double,6,1> CMatrixDouble61;
		typedef CMatrixFixedNumeric<double,1,6> CMatrixDouble16;
		typedef CMatrixFixedNumeric<double,7,1> CMatrixDouble71;
		typedef CMatrixFixedNumeric<double,1,7> CMatrixDouble17;
		typedef CMatrixFixedNumeric<double,5,1> CMatrixDouble51;
		typedef CMatrixFixedNumeric<double,1,5> CMatrixDouble15;
		typedef CMatrixFixedNumeric<double,4,1> CMatrixDouble41;

		typedef CMatrixFixedNumeric<float,2,2> CMatrixFloat22;
		typedef CMatrixFixedNumeric<float,2,3> CMatrixFloat23;
		typedef CMatrixFixedNumeric<float,3,2> CMatrixFloat32;
		typedef CMatrixFixedNumeric<float,3,3> CMatrixFloat33;
		typedef CMatrixFixedNumeric<float,4,4> CMatrixFloat44;
		typedef CMatrixFixedNumeric<float,6,6> CMatrixFloat66;
		typedef CMatrixFixedNumeric<float,7,7> CMatrixFloat77;
		typedef CMatrixFixedNumeric<float,1,3> CMatrixFloat13;
		typedef CMatrixFixedNumeric<float,3,1> CMatrixFloat31;
		typedef CMatrixFixedNumeric<float,1,2> CMatrixFloat12;
		typedef CMatrixFixedNumeric<float,2,1> CMatrixFloat21;
		typedef CMatrixFixedNumeric<float,6,1> CMatrixFloat61;
		typedef CMatrixFixedNumeric<float,1,6> CMatrixFloat16;
		typedef CMatrixFixedNumeric<float,7,1> CMatrixFloat71;
		typedef CMatrixFixedNumeric<float,1,7> CMatrixFloat17;
		typedef CMatrixFixedNumeric<float,5,1> CMatrixFloat51;
		typedef CMatrixFixedNumeric<float,1,5> CMatrixFloat15;
		/**  @} */

    }
}
