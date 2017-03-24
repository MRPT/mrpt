/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

// Eigen forward declarations:
#include <mrpt/config.h>

// Minimum Eigen fwrd-decls:
namespace Eigen {
	template<typename Derived> struct EigenBase;
}

namespace mrpt
{
	namespace math
	{
		/** ContainerType<T>::element_t exposes the value of any STL or Eigen container */
		template <typename CONTAINER> struct ContainerType;
		/** Specialization for Eigen containers */
		template <typename Derived>
		struct ContainerType<Eigen::EigenBase<Derived> > {
			typedef typename Derived::Scalar element_t;
		};

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
