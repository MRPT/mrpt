/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

// Eigen forward declarations:
#include <mrpt/config.h>

// Minimum Eigen fwrd-decls:
namespace Eigen
{
template <typename Derived>
struct EigenBase;
template <typename Derived>
class MatrixBase;
}  // namespace Eigen

namespace mrpt
{
namespace math
{
/** ContainerType<T>::element_t exposes the value of any STL or Eigen container
 */
template <typename CONTAINER>
struct ContainerType;
/** Specialization for Eigen containers */
template <typename Derived>
struct ContainerType<Eigen::EigenBase<Derived>>
{
	using element_t = typename Derived::Scalar;
};

// Dynamic size:
template <class T>
class CMatrixTemplateNumeric;
using CMatrixFloat = CMatrixTemplateNumeric<float>;
using CMatrixDouble = CMatrixTemplateNumeric<double>;
template <typename T>
class dynamic_vector;
using CVectorFloat = dynamic_vector<float>;
using CVectorDouble = dynamic_vector<double>;

// Fixed size:
template <typename T, size_t NROWS, size_t NCOLS>
class CMatrixFixedNumeric;

/** @name Typedefs for common sizes
	@{ */
using CMatrixDouble22 = CMatrixFixedNumeric<double, 2, 2>;
using CMatrixDouble23 = CMatrixFixedNumeric<double, 2, 3>;
using CMatrixDouble32 = CMatrixFixedNumeric<double, 3, 2>;
using CMatrixDouble33 = CMatrixFixedNumeric<double, 3, 3>;
using CMatrixDouble44 = CMatrixFixedNumeric<double, 4, 4>;
using CMatrixDouble66 = CMatrixFixedNumeric<double, 6, 6>;
using CMatrixDouble77 = CMatrixFixedNumeric<double, 7, 7>;
using CMatrixDouble13 = CMatrixFixedNumeric<double, 1, 3>;
using CMatrixDouble31 = CMatrixFixedNumeric<double, 3, 1>;
using CMatrixDouble12 = CMatrixFixedNumeric<double, 1, 2>;
using CMatrixDouble21 = CMatrixFixedNumeric<double, 2, 1>;
using CMatrixDouble61 = CMatrixFixedNumeric<double, 6, 1>;
using CMatrixDouble16 = CMatrixFixedNumeric<double, 1, 6>;
using CMatrixDouble71 = CMatrixFixedNumeric<double, 7, 1>;
using CMatrixDouble17 = CMatrixFixedNumeric<double, 1, 7>;
using CMatrixDouble51 = CMatrixFixedNumeric<double, 5, 1>;
using CMatrixDouble15 = CMatrixFixedNumeric<double, 1, 5>;
using CMatrixDouble41 = CMatrixFixedNumeric<double, 4, 1>;

using CMatrixFloat22 = CMatrixFixedNumeric<float, 2, 2>;
using CMatrixFloat23 = CMatrixFixedNumeric<float, 2, 3>;
using CMatrixFloat32 = CMatrixFixedNumeric<float, 3, 2>;
using CMatrixFloat33 = CMatrixFixedNumeric<float, 3, 3>;
using CMatrixFloat44 = CMatrixFixedNumeric<float, 4, 4>;
using CMatrixFloat66 = CMatrixFixedNumeric<float, 6, 6>;
using CMatrixFloat77 = CMatrixFixedNumeric<float, 7, 7>;
using CMatrixFloat13 = CMatrixFixedNumeric<float, 1, 3>;
using CMatrixFloat31 = CMatrixFixedNumeric<float, 3, 1>;
using CMatrixFloat12 = CMatrixFixedNumeric<float, 1, 2>;
using CMatrixFloat21 = CMatrixFixedNumeric<float, 2, 1>;
using CMatrixFloat61 = CMatrixFixedNumeric<float, 6, 1>;
using CMatrixFloat16 = CMatrixFixedNumeric<float, 1, 6>;
using CMatrixFloat71 = CMatrixFixedNumeric<float, 7, 1>;
using CMatrixFloat17 = CMatrixFixedNumeric<float, 1, 7>;
using CMatrixFloat51 = CMatrixFixedNumeric<float, 5, 1>;
using CMatrixFloat15 = CMatrixFixedNumeric<float, 1, 5>;
/**  @} */
}  // namespace math
}  // namespace mrpt
