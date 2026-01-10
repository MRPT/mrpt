/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::math
{
/**  This class is a "CSerializable" wrapper for "CMatrixBool".
 * \note For a complete introduction to Matrices and vectors in MRPT, see:
 * https://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
 * \ingroup mrpt_math_grp
 */
class CMatrixB : public mrpt::serialization::CSerializable, public CMatrixBool
{
  DEFINE_SERIALIZABLE(CMatrixB, mrpt::math)
 public:
  /** Constructor */
  CMatrixB(size_t row = 1, size_t col = 1) : CMatrixBool(row, col) {}
  /** Copy constructor */
  CMatrixB(const CMatrixBool& m) : CMatrixBool(m) {}
  /** Assignment operator for float matrixes */
  CMatrixB& operator=(const CMatrixBool& m)
  {
    CMatrixBool::operator=(m);
    return *this;
  }
};  // end of class definition

}  // namespace mrpt::math
