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
#include <mrpt/math/point_poses2vectors.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::math
{
/**  This class is a "CSerializable" wrapper for
 * "CMatrixDynamic<double>".
 * \note For a complete introduction to Matrices and vectors in MRPT, see:
 * https://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
 * \ingroup mrpt_math_grp
 */
class CMatrixD : public mrpt::serialization::CSerializable, public CMatrixDynamic<double>
{
  DEFINE_SERIALIZABLE(CMatrixD, mrpt::math)
  DEFINE_SCHEMA_SERIALIZABLE()
 public:
  using Base = CMatrixDynamic<double>;

  /** Constructor */
  CMatrixD() : Base(1, 1) {}
  /** Constructor */
  CMatrixD(Base::size_type row, Base::size_type col) : Base(row, col) {}

  /** Copy constructor */
  explicit CMatrixD(const Base& m) : Base(m) {}

  /** Copy constructor  */
  explicit CMatrixD(const CMatrixFloat& m) : Base(0, 0) { *this = m; }

  /*! Assignment operator from any other Eigen class */
  template <typename Other>
  inline CMatrixD& operator=(const Other& other)
  {
    Base::operator=(other);
    return *this;
  }
  /*! Constructor from any other Eigen class */
  template <class Other>
  explicit CMatrixD(const Other& other)
  {
    Base::operator=(other);
  }

  MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CMatrixD)

};  // end of class definition
mrpt::serialization::CArchive& operator>>(mrpt::serialization::CArchive& in, CMatrixD::Ptr& pObj);

}  // namespace mrpt::math
