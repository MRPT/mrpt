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

#ifndef EIGEN_TRIANGULARMATRIXVECTOR_H
#define EIGEN_TRIANGULARMATRIXVECTOR_H

namespace internal {

template<typename Index, int Mode, typename LhsScalar, bool ConjLhs, typename RhsScalar, bool ConjRhs, int StorageOrder>
struct product_triangular_matrix_vector;

template<typename Index, int Mode, typename LhsScalar, bool ConjLhs, typename RhsScalar, bool ConjRhs>
struct product_triangular_matrix_vector<Index,Mode,LhsScalar,ConjLhs,RhsScalar,ConjRhs,ColMajor>
{
  typedef typename scalar_product_traits<LhsScalar, RhsScalar>::ReturnType ResScalar;
  enum {
    IsLower = ((Mode&Lower)==Lower),
    HasUnitDiag = (Mode & UnitDiag)==UnitDiag
  };
  static EIGEN_DONT_INLINE  void run(Index rows, Index cols, const LhsScalar* _lhs, Index lhsStride,
                                     const RhsScalar* _rhs, Index rhsIncr, ResScalar* _res, Index resIncr, ResScalar alpha)
  {
    EIGEN_UNUSED_VARIABLE(resIncr);
    eigen_assert(resIncr==1);
    
    static const Index PanelWidth = EIGEN_TUNE_TRIANGULAR_PANEL_WIDTH;

    typedef Map<Matrix<LhsScalar,Dynamic,Dynamic,ColMajor>, 0, OuterStride<> > LhsMap;
    const LhsMap lhs(_lhs,rows,cols,OuterStride<>(lhsStride));
    typename conj_expr_if<ConjLhs,LhsMap>::type cjLhs(lhs);
    
    typedef Map<Matrix<RhsScalar,Dynamic,1>, 0, InnerStride<> > RhsMap;
    const RhsMap rhs(_rhs,cols,InnerStride<>(rhsIncr));
    typename conj_expr_if<ConjRhs,RhsMap>::type cjRhs(rhs);

    typedef Map<Matrix<ResScalar,Dynamic,1> > ResMap;
    ResMap res(_res,rows);

    for (Index pi=0; pi<cols; pi+=PanelWidth)
    {
      Index actualPanelWidth = std::min(PanelWidth, cols-pi);
      for (Index k=0; k<actualPanelWidth; ++k)
      {
        Index i = pi + k;
        Index s = IsLower ? (HasUnitDiag ? i+1 : i ) : pi;
        Index r = IsLower ? actualPanelWidth-k : k+1;
        if ((!HasUnitDiag) || (--r)>0)
          res.segment(s,r) += (alpha * cjRhs.coeff(i)) * cjLhs.col(i).segment(s,r);
        if (HasUnitDiag)
          res.coeffRef(i) += alpha * cjRhs.coeff(i);
      }
      Index r = IsLower ? cols - pi - actualPanelWidth : pi;
      if (r>0)
      {
        Index s = IsLower ? pi+actualPanelWidth : 0;
        general_matrix_vector_product<Index,LhsScalar,ColMajor,ConjLhs,RhsScalar,ConjRhs>::run(
            r, actualPanelWidth,
            &lhs.coeff(s,pi), lhsStride,
            &rhs.coeff(pi), rhsIncr,
            &res.coeffRef(s), resIncr, alpha);
      }
    }
  }
};

template<typename Index, int Mode, typename LhsScalar, bool ConjLhs, typename RhsScalar, bool ConjRhs>
struct product_triangular_matrix_vector<Index,Mode,LhsScalar,ConjLhs,RhsScalar,ConjRhs,RowMajor>
{
  typedef typename scalar_product_traits<LhsScalar, RhsScalar>::ReturnType ResScalar;
  enum {
    IsLower = ((Mode&Lower)==Lower),
    HasUnitDiag = (Mode & UnitDiag)==UnitDiag
  };
  static void run(Index rows, Index cols, const LhsScalar* _lhs, Index lhsStride,
                  const RhsScalar* _rhs, Index rhsIncr, ResScalar* _res, Index resIncr, ResScalar alpha)
  {
    eigen_assert(rhsIncr==1);
    EIGEN_UNUSED_VARIABLE(rhsIncr);
    
    static const Index PanelWidth = EIGEN_TUNE_TRIANGULAR_PANEL_WIDTH;

    typedef Map<Matrix<LhsScalar,Dynamic,Dynamic,RowMajor>, 0, OuterStride<> > LhsMap;
    const LhsMap lhs(_lhs,rows,cols,OuterStride<>(lhsStride));
    typename conj_expr_if<ConjLhs,LhsMap>::type cjLhs(lhs);

    typedef Map<Matrix<RhsScalar,Dynamic,1> > RhsMap;
    const RhsMap rhs(_rhs,cols);
    typename conj_expr_if<ConjRhs,RhsMap>::type cjRhs(rhs);

    typedef Map<Matrix<ResScalar,Dynamic,1>, 0, InnerStride<> > ResMap;
    ResMap res(_res,rows,InnerStride<>(resIncr));
    
    for (Index pi=0; pi<cols; pi+=PanelWidth)
    {
      Index actualPanelWidth = std::min(PanelWidth, cols-pi);
      for (Index k=0; k<actualPanelWidth; ++k)
      {
        Index i = pi + k;
        Index s = IsLower ? pi  : (HasUnitDiag ? i+1 : i);
        Index r = IsLower ? k+1 : actualPanelWidth-k;
        if ((!HasUnitDiag) || (--r)>0)
          res.coeffRef(i) += alpha * (cjLhs.row(i).segment(s,r).cwiseProduct(cjRhs.segment(s,r).transpose())).sum();
        if (HasUnitDiag)
          res.coeffRef(i) += alpha * cjRhs.coeff(i);
      }
      Index r = IsLower ? pi : cols - pi - actualPanelWidth;
      if (r>0)
      {
        Index s = IsLower ? 0 : pi + actualPanelWidth;
        general_matrix_vector_product<Index,LhsScalar,RowMajor,ConjLhs,RhsScalar,ConjRhs>::run(
            actualPanelWidth, r,
            &(lhs.coeff(pi,s)), lhsStride,
            &(rhs.coeff(s)), rhsIncr,
            &res.coeffRef(pi), resIncr, alpha);
      }
    }
  }
};

/***************************************************************************
* Wrapper to product_triangular_vector
***************************************************************************/

template<int Mode, bool LhsIsTriangular, typename Lhs, typename Rhs>
struct traits<TriangularProduct<Mode,LhsIsTriangular,Lhs,false,Rhs,true> >
 : traits<ProductBase<TriangularProduct<Mode,LhsIsTriangular,Lhs,false,Rhs,true>, Lhs, Rhs> >
{};

template<int Mode, bool LhsIsTriangular, typename Lhs, typename Rhs>
struct traits<TriangularProduct<Mode,LhsIsTriangular,Lhs,true,Rhs,false> >
 : traits<ProductBase<TriangularProduct<Mode,LhsIsTriangular,Lhs,true,Rhs,false>, Lhs, Rhs> >
{};

} // end namespace internal

template<int Mode, typename Lhs, typename Rhs>
struct TriangularProduct<Mode,true,Lhs,false,Rhs,true>
  : public ProductBase<TriangularProduct<Mode,true,Lhs,false,Rhs,true>, Lhs, Rhs >
{
  EIGEN_PRODUCT_PUBLIC_INTERFACE(TriangularProduct)

  TriangularProduct(const Lhs& lhs, const Rhs& rhs) : Base(lhs,rhs) {}

  template<typename Dest> void scaleAndAddTo(Dest& dst, Scalar alpha) const
  {
    eigen_assert(dst.rows()==m_lhs.rows() && dst.cols()==m_rhs.cols());

    const ActualLhsType lhs = LhsBlasTraits::extract(m_lhs);
    const ActualRhsType rhs = RhsBlasTraits::extract(m_rhs);

    Scalar actualAlpha = alpha * LhsBlasTraits::extractScalarFactor(m_lhs)
                               * RhsBlasTraits::extractScalarFactor(m_rhs);

    internal::product_triangular_matrix_vector
      <Index,Mode,
       typename _ActualLhsType::Scalar, LhsBlasTraits::NeedToConjugate,
       typename _ActualRhsType::Scalar, RhsBlasTraits::NeedToConjugate,
       (int(internal::traits<Lhs>::Flags)&RowMajorBit) ? RowMajor : ColMajor>
      ::run(lhs.rows(),lhs.cols(),lhs.data(),lhs.outerStride(),rhs.data(),rhs.innerStride(),
                                  dst.data(),dst.innerStride(),actualAlpha);
  }
};

template<int Mode, typename Lhs, typename Rhs>
struct TriangularProduct<Mode,false,Lhs,true,Rhs,false>
  : public ProductBase<TriangularProduct<Mode,false,Lhs,true,Rhs,false>, Lhs, Rhs >
{
  EIGEN_PRODUCT_PUBLIC_INTERFACE(TriangularProduct)

  TriangularProduct(const Lhs& lhs, const Rhs& rhs) : Base(lhs,rhs) {}

  template<typename Dest> void scaleAndAddTo(Dest& dst, Scalar alpha) const
  {

    eigen_assert(dst.rows()==m_lhs.rows() && dst.cols()==m_rhs.cols());

    const ActualLhsType lhs = LhsBlasTraits::extract(m_lhs);
    const ActualRhsType rhs = RhsBlasTraits::extract(m_rhs);

    Scalar actualAlpha = alpha * LhsBlasTraits::extractScalarFactor(m_lhs)
                               * RhsBlasTraits::extractScalarFactor(m_rhs);

    internal::product_triangular_matrix_vector
      <Index,(Mode & UnitDiag) | (Mode & Lower) ? Upper : Lower,
       typename _ActualRhsType::Scalar, RhsBlasTraits::NeedToConjugate,
       typename _ActualLhsType::Scalar, LhsBlasTraits::NeedToConjugate,
       (int(internal::traits<Rhs>::Flags)&RowMajorBit) ? ColMajor : RowMajor>
      ::run(rhs.rows(),rhs.cols(),rhs.data(),rhs.outerStride(),lhs.data(),lhs.innerStride(),
                                  dst.data(),dst.innerStride(),actualAlpha);
  }
};

#endif // EIGEN_TRIANGULARMATRIXVECTOR_H
