/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstddef>  // std:size_t

/*! \file math_frwds.h
 * Forward declarations of all mrpt::math classes related to vectors, arrays
 * and matrices.
 * Many of the function implementations are in ops_matrices.h, others in
 * ops_containers.h
 */

// Minimum Eigen forward declarations for use in MRPT templates:
namespace Eigen
{
template <typename PlainObjectType, int MapOptions, typename StrideType>
class Map;
template <int Value>
class InnerStride;
template <int Outter, int Inner>
class Stride;
template <
	typename _Scalar, int _Rows, int _Cols, int _Options,
	int _MaxRows /*= _Rows*/, int _MaxCols /* = _Cols*/>
class Matrix;
// For reference: _Options =
// /*AutoAlign*/ 0 | ((_Rows == 1 && _Cols != 1) ? /*Eigen::RowMajor*/ 1
//  : (_Cols == 1 && _Rows != 1) ? /*Eigen::ColMajor*/ 0 : /*default: Col*/ 0)
// ==> That is: _Options=1 for RowMajor as it's the default in MRPT matrices.

template <typename Derived>
class MatrixBase;
template <typename Derived>
struct EigenBase;
template <typename VectorType, int Size>
class VectorBlock;
template <typename _Lhs, typename _Rhs, int Option>
class Product;
template <typename BinaryOp, typename LhsType, typename RhsType>
class CwiseBinaryOp;
}  // namespace Eigen

namespace mrpt::math
{
/** For usage in one of the constructors of CMatrixFixed or
   CMatrixDynamic (and derived classes), if it's not required
	 to fill it with zeros at the constructor to save time. */
enum TConstructorFlags_Matrices
{
	UNINITIALIZED_MATRIX = 0
};

template <class T>
class CMatrixDynamic;
template <typename T, std::size_t ROWS, std::size_t COLS>
class CMatrixFixed;

}  // namespace mrpt::math
