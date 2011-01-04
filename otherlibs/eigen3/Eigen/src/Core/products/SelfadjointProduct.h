// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

#ifndef EIGEN_SELFADJOINT_PRODUCT_H
#define EIGEN_SELFADJOINT_PRODUCT_H

/**********************************************************************
* This file implements a self adjoint product: C += A A^T updating only
* half of the selfadjoint matrix C.
* It corresponds to the level 3 SYRK Blas routine.
**********************************************************************/

// high level API

template<typename MatrixType, unsigned int UpLo>
template<typename DerivedU>
SelfAdjointView<MatrixType,UpLo>& SelfAdjointView<MatrixType,UpLo>
::rankUpdate(const MatrixBase<DerivedU>& u, Scalar alpha)
{
  typedef internal::blas_traits<DerivedU> UBlasTraits;
  typedef typename UBlasTraits::DirectLinearAccessType ActualUType;
  typedef typename internal::remove_all<ActualUType>::type _ActualUType;
  const ActualUType actualU = UBlasTraits::extract(u.derived());

  Scalar actualAlpha = alpha * UBlasTraits::extractScalarFactor(u.derived());

  enum { IsRowMajor = (internal::traits<MatrixType>::Flags&RowMajorBit) ? 1 : 0 };
  
  internal::general_matrix_matrix_triangular_product<Index,
    Scalar, _ActualUType::Flags&RowMajorBit ? RowMajor : ColMajor,   UBlasTraits::NeedToConjugate  && NumTraits<Scalar>::IsComplex,
    Scalar, _ActualUType::Flags&RowMajorBit ? ColMajor : RowMajor, (!UBlasTraits::NeedToConjugate) && NumTraits<Scalar>::IsComplex,
    MatrixType::Flags&RowMajorBit ? RowMajor : ColMajor, UpLo>
    ::run(_expression().cols(), actualU.cols(),
          &actualU.coeffRef(0,0), actualU.outerStride(), &actualU.coeffRef(0,0), actualU.outerStride(),
          _expression().const_cast_derived().data(), _expression().outerStride(), actualAlpha);

  return *this;
}

#endif // EIGEN_SELFADJOINT_PRODUCT_H
