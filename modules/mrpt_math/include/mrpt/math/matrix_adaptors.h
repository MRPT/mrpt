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

#include <mrpt/math/math_frwds.h>  // forward declarations

namespace mrpt
{
namespace math
{
/** Internal classes not to be directly used by the user. */
// Forward declarations:
template <typename T, typename U, bool UIsObject>
class CBinaryRelation;
namespace detail
{
/**
 * This template is a trick to switch the type of a variable using a boolean
 * variable in the template. It's easy to extend its functionality to several
 * types, using a unsigned char instead of a bool.
 */
template <typename U, bool B>
class MatrixWrapper;

// partial specializations:
template <typename U>
class MatrixWrapper<U, true>
{
 public:
  using MatrixType = CMatrixTemplateObjects<U>;
};
template <typename U>
class MatrixWrapper<U, false>
{
 public:
  using MatrixType = CMatrixDynamic<U>;
};

template <typename T, typename U, bool UIsObject, typename FunctionType>
void applyFunction(
    CBinaryRelation<T, U, UIsObject>& o,
    FunctionType fun,
    size_t e1,
    size_t e2,
    const T& T1,
    const T& T2);
}  // namespace detail

namespace detail
{
/** Template class for matrix accessor's iterators.
 * \sa CMatrixRowAccessor,CMatrixColumnAccessor
 */
template <typename A, typename T>
class AccessorIterator
{
 protected:
  A* base;
  int pos;

 public:
  // typedefs for iterator_traits:
  using iterator_category = std::random_access_iterator_tag;
  using value_type = T;
  using difference_type = int;
  using pointer = T*;
  using reference = T&;

  AccessorIterator(A& obj, size_t N) : base(&obj), pos(N) {}
  T& operator*() const { return (*base)[pos]; }
  AccessorIterator<A, T>& operator++()
  {
    ++pos;
    return *this;
  }
  AccessorIterator<A, T> operator++(int)
  {
    AccessorIterator<A, T> it = *this;
    ++*this;
    return it;
  }
  AccessorIterator<A, T>& operator--()
  {
    --pos;
    return *this;
  }
  AccessorIterator<A, T> operator--(int)
  {
    AccessorIterator<A, T> it = *this;
    --*this;
    return it;
  }
  AccessorIterator<A, T>& operator+=(int off)
  {
    pos += off;
    return *this;
  }
  AccessorIterator<A, T> operator+(int off) const
  {
    AccessorIterator<A, T> it = *this;
    it += off;
    return it;
  }
  AccessorIterator<A, T>& operator-=(int off)
  {
    pos -= off;
    return *this;
  }
  AccessorIterator<A, T> operator-(int off) const
  {
    AccessorIterator<A, T> it = *this;
    it -= off;
    return it;
  }
  int operator-(const AccessorIterator<A, T>& it) const { return pos - it.pos; }
  T& operator[](int off) const { return (*base)[pos + off]; }
  bool operator==(const AccessorIterator<A, T>& it) const
  {
    return (pos == it.pos) && (base == it.base);
  }
  bool operator!=(const AccessorIterator<A, T>& it) const { return !(operator==(it)); }
};

/** Template class for matrix accessor's iterators.
 * \sa CMatrixRowAccessor,CMatrixColumnAccessor
 */
template <typename A, typename T>
class ReverseAccessorIterator
{
 protected:
  A* base;
  int pos;

 public:
  // typedefs for iterator_traits:
  using iterator_category = std::random_access_iterator_tag;
  using value_type = T;
  using difference_type = int;
  using pointer = T*;
  using reference = T&;

  ReverseAccessorIterator(A& obj, size_t N) : base(&obj), pos(N) {}
  T& operator*() const { return (*base)[pos]; }
  ReverseAccessorIterator<A, T>& operator++()
  {
    --pos;
    return *this;
  }
  ReverseAccessorIterator<A, T> operator++(int)
  {
    ReverseAccessorIterator<A, T> it = *this;
    ++*this;  // Yes, that's right.
    return it;
  }
  ReverseAccessorIterator<A, T>& operator--()
  {
    ++pos;
    return *this;
  }
  ReverseAccessorIterator<A, T> operator--(int)
  {
    ReverseAccessorIterator<A, T> it = *this;
    --*this;  // Yes, that's right.
    return it;
  }
  ReverseAccessorIterator<A, T>& operator+=(int off)
  {
    pos -= off;
    return *this;
  }
  ReverseAccessorIterator<A, T> operator+(int off) const
  {
    ReverseAccessorIterator<A, T> it = *this;
    it += off;  // Yes, that's right.
    return it;
  }
  AccessorIterator<A, T>& operator-=(int off)
  {
    pos += off;
    return *this;
  }
  AccessorIterator<A, T> operator-(int off) const
  {
    ReverseAccessorIterator<A, T> it = *this;
    it -= off;  // Yes, that's right
    return it;
  }
  int operator-(const ReverseAccessorIterator<A, T>& it) const { return it.pos - pos; }
  T& operator[](int off) const { return (*base)[pos - off]; }
  bool operator==(const ReverseAccessorIterator<A, T>& it) const
  {
    return (pos == it.pos) && (&base == &it.base);
  }
  bool operator!=(const ReverseAccessorIterator<A, T>& it) const { return !(operator==(it)); }
};
}  // namespace detail

/** A vector-like wrapper for a Matrix for accessing the elements of a given row
 * with a [] operator.
 *  For usage with MRPT's CMatrixDynamic only (for MRPT numeric matrices, use
 * Eigen methods)
 * \sa
 * CMatrixColumnAccessor,CMatrixRowAccessorExtended,CConstMatrixRowAccessor,CConstMatrixRowAccessorExtended
 */
template <typename MAT>
class CMatrixRowAccessor
{
 protected:
  MAT* m_mat;
  size_t m_rowInd;

 public:
  using value_type = typename MAT::Scalar;
  using mrpt_autotype = CMatrixRowAccessor<MAT>;
  CMatrixRowAccessor(MAT& mat, size_t rowIdx) : m_mat(&mat), m_rowInd(rowIdx)
  {
    ASSERT_(rowIdx < mat.rows();
  }
  CMatrixRowAccessor() {}
  value_type& operator[](const size_t i) { return (*m_mat)(m_rowInd, i); }
  value_type operator[](const size_t i) const { return (*m_mat)(m_rowInd, i); }
  using iterator = detail::AccessorIterator<CMatrixRowAccessor<MAT>, value_type>;
  using const_iterator = detail::AccessorIterator<const CMatrixRowAccessor<MAT>, const value_type>;
  using reverse_iterator = detail::ReverseAccessorIterator<CMatrixRowAccessor<MAT>, value_type>;
  using const_reverse_iterator =
      detail::ReverseAccessorIterator<const CMatrixRowAccessor<MAT>, const value_type>;
  iterator begin() { return iterator(*this, 0); }
  const_iterator begin() const { return const_iterator(*this, 0); }
  iterator end() { return iterator(*this, m_mat->cols()); }
  const_iterator end() const { return const_iterator(*this, m_mat->cols()); }
  reverse_iterator rbegin() { return reverse_iterator(*this, m_mat->cols() - 1); }
  const_reverse_iterator rbegin() const { return const_reverse_iterator(*this, m_mat->cols() - 1); }
  reverse_iterator rend() { return reverse_iterator(*this, -1); }
  const_reverse_iterator rend() const { return const_reverse_iterator(*this, -1); }
  size_t size() const { return m_mat->cols(); }
  void resize(size_t N)
  {
    if (N != size()) throw std::logic_error("Tried to resize a fixed-size vector");
  }
};
template <typename MAT>
CMatrixRowAccessor<MAT> getRowAccessor(MAT& m, size_t rowIdx)
{
  return CMatrixRowAccessor<MAT>(m, rowIdx);
}

/** A vector-like wrapper for a Matrix for accessing the elements of a given row
 * with a [] operator, with offset and custom spacing.
 *  For usage with MRPT's CMatrixDynamic only (for MRPT numeric matrices, use
 * Eigen methods)
 * \sa
 * CMatrixColumnAccessorExtended,CMatrixRowAccessor,CConstMatrixRowAccessor,CConstMatrixRowAccessorExtended
 */
template <class MAT>
class CMatrixRowAccessorExtended
{
 protected:
  MAT* m_mat;
  size_t m_rowInd;
  size_t m_colOffset;
  size_t m_elementsSpace;
  size_t howMany;

 public:
  using value_type = typename MAT::Scalar;
  using mrpt_autotype = CMatrixRowAccessorExtended<MAT>;
  CMatrixRowAccessorExtended(MAT& mat, size_t row, size_t offset, size_t space) :
      m_mat(&mat), m_rowInd(row), m_colOffset(offset), m_elementsSpace(space)
  {
    ASSERT_(row < mat.rows());
    howMany = (mat.cols() - m_colOffset) / m_elementsSpace;
  }
  CMatrixRowAccessorExtended() {}
  value_type& operator[](size_t i)
  {
    return (*m_mat)(m_rowInd, m_colOffset + (i * m_elementsSpace));
  }
  value_type operator[](size_t i) const
  {
    return (*m_mat)(m_rowInd, m_colOffset + (i * m_elementsSpace));
  }
  using iterator = detail::AccessorIterator<CMatrixRowAccessorExtended<MAT>, value_type>;
  using const_iterator =
      detail::AccessorIterator<const CMatrixRowAccessorExtended<MAT>, const value_type>;
  using reverse_iterator =
      detail::ReverseAccessorIterator<CMatrixRowAccessorExtended<MAT>, value_type>;
  using const_reverse_iterator =
      detail::ReverseAccessorIterator<const CMatrixRowAccessorExtended<MAT>, const value_type>;
  iterator begin() { return iterator(*this, 0); }
  const_iterator begin() const { return const_iterator(*this, 0); }
  iterator end() { return iterator(*this, howMany); }
  const_iterator end() const { return const_iterator(*this, howMany); }
  reverse_iterator rbegin() { return reverse_iterator(*this, howMany - 1); }
  const_reverse_iterator rbegin() const { return const_reverse_iterator(*this, howMany - 1); }
  reverse_iterator rend() { return reverse_iterator(*this, -1); }
  const_reverse_iterator rend() const { return const_reverse_iterator(*this, -1); }
  size_t size() const { return howMany; }
  void resize(size_t N)
  {
    if (N != size()) throw std::logic_error("Tried to resize a fixed-size vector");
  }
};
template <typename MAT>
CMatrixRowAccessorExtended<MAT> getRowAccessor(
    MAT& m, size_t rowIdx, size_t offset, size_t space = 1)
{
  return CMatrixRowAccessor<MAT>(m, rowIdx, offset, space);
}

/** A vector-like wrapper for a const Matrix for accessing the elements of a
 * given row with a [] operator.
 *  For usage with MRPT's CMatrixDynamic only (for MRPT numeric matrices, use
 * Eigen methods)
 * \sa
 * CConstMatrixColumnAccessor,CMatrixRowAccessorExtended,CMatrixRowAccessor,CConstMatrixRowAccessorExtended
 */
template <class MAT>
class CConstMatrixRowAccessor
{
 protected:
  const MAT* m_mat;
  size_t m_rowInd;

 public:
  using value_type = typename MAT::Scalar;
  using mrpt_autotype = CConstMatrixRowAccessor<MAT>;
  CConstMatrixRowAccessor(const MAT& mat, size_t row) : m_mat(&mat), m_rowInd(row)
  {
    ASSERT_(row < mat.rows());
  }
  CConstMatrixRowAccessor() {}
  value_type operator[](size_t i) const { return (*m_mat)(m_rowInd, i); }
  using const_iterator =
      detail::AccessorIterator<const CConstMatrixRowAccessor<MAT>, const value_type>;
  using const_reverse_iterator =
      detail::ReverseAccessorIterator<const CConstMatrixRowAccessor<MAT>, const value_type>;
  const_iterator begin() const { return const_iterator(*this, 0); }
  const_iterator end() const { return const_iterator(*this, m_mat->cols()); }
  const_reverse_iterator rbegin() const { return const_reverse_iterator(*this, m_mat->cols() - 1); }
  const_reverse_iterator rend() const { return const_reverse_iterator(*this, -1); }
  size_t size() const { return m_mat->cols(); }
  void resize(size_t N)
  {
    if (N != size()) throw std::logic_error("Tried to resize a fixed-size vector");
  }
};
template <typename MAT>
CConstMatrixRowAccessor<MAT> getRowAccessor(const MAT& m, size_t rowIdx)
{
  return CMatrixRowAccessor<MAT>(m, rowIdx);
}

/** A vector-like wrapper for a const Matrix for accessing the elements of a
 * given row with a [] operator, with offset and custom spacing.
 *  For usage with MRPT's CMatrixDynamic only (for MRPT numeric matrices, use
 * Eigen methods)
 * \sa
 * CConstMatrixColumnAccessorExtended,CMatrixRowAccessor,CConstMatrixRowAccessor,CMatrixRowAccessorExtended
 */
template <class MAT>
class CConstMatrixRowAccessorExtended
{
 protected:
  const MAT* m_mat;
  size_t m_rowInd;
  size_t m_colOffset;
  size_t m_elementsSpace;
  size_t howMany;

 public:
  using value_type = typename MAT::Scalar;
  using mrpt_autotype = CConstMatrixRowAccessorExtended<MAT>;
  CConstMatrixRowAccessorExtended(const MAT& mat, size_t row, size_t offset, size_t space) :
      m_mat(&mat), m_rowInd(row), m_colOffset(offset), m_elementsSpace(space)
  {
    ASSERT_(row < mat.rows());
    howMany = (mat.cols() - m_colOffset) / m_elementsSpace;
  }
  CConstMatrixRowAccessorExtended() {}
  value_type operator[](size_t i) const
  {
    return (*m_mat)(m_rowInd, m_colOffset + (i * m_elementsSpace));
  }
  using const_iterator =
      detail::AccessorIterator<const CConstMatrixRowAccessorExtended<MAT>, const value_type>;
  using const_reverse_iterator =
      detail::ReverseAccessorIterator<const CConstMatrixRowAccessorExtended<MAT>, const value_type>;
  const_iterator begin() const { return const_iterator(*this, 0); }
  const_iterator end() const { return const_iterator(*this, howMany); }
  const_reverse_iterator rbegin() const { return const_reverse_iterator(*this, howMany - 1); }
  const_reverse_iterator rend() const { return const_reverse_iterator(*this, -1); }
  size_t size() const { return howMany; }
  void resize(size_t N)
  {
    if (N != size()) throw std::logic_error("Tried to resize a fixed-size vector");
  }
};
template <typename MAT>
CConstMatrixRowAccessorExtended<MAT> getRowAccessor(
    const MAT& m, size_t rowIdx, size_t offset, size_t space = 1)
{
  return CConstMatrixRowAccessorExtended<MAT>(m, rowIdx, offset, space);
}

/** A vector-like wrapper for a Matrix for accessing the elements of a given
 * column with a [] operator.
 * \sa
 * CMatrixRowAccessor,CMatrixColumnAccessorExtended,CConstMatrixColumnAccessor,CConstMatrixColumnAccessorExtended
 */
template <typename MAT>
class CMatrixColumnAccessor
{
 protected:
  MAT* m_mat;
  size_t m_colInd;

 public:
  using value_type = typename MAT::Scalar;
  using mrpt_autotype = CMatrixColumnAccessor<MAT>;
  CMatrixColumnAccessor(MAT& mat, size_t colIdx) : m_mat(&mat), m_colInd(colIdx)
  {
    ASSERT_(colIdx < mat.cols();
  }
  CMatrixColumnAccessor() {}
  value_type& operator[](const size_t i) { return (*m_mat)(i, m_colInd); }
  value_type operator[](const size_t i) const { return (*m_mat)(i, m_colInd); }
  using iterator = detail::AccessorIterator<CMatrixColumnAccessor<MAT>, value_type>;
  using const_iterator =
      detail::AccessorIterator<const CMatrixColumnAccessor<MAT>, const value_type>;
  using reverse_iterator = detail::ReverseAccessorIterator<CMatrixColumnAccessor<MAT>, value_type>;
  using const_reverse_iterator =
      detail::ReverseAccessorIterator<const CMatrixColumnAccessor<MAT>, const value_type>;
  iterator begin() { return iterator(*this, 0); }
  const_iterator begin() const { return const_iterator(*this, 0); }
  iterator end() { return iterator(*this, m_mat->rows()); }
  const_iterator end() const { return const_iterator(*this, m_mat->rows()); }
  reverse_iterator rbegin() { return reverse_iterator(*this, m_mat->rows() - 1); }
  const_reverse_iterator rbegin() const { return const_reverse_iterator(*this, m_mat->rows() - 1); }
  reverse_iterator rend() { return reverse_iterator(*this, -1); }
  const_reverse_iterator rend() const { return const_reverse_iterator(*this, -1); }
  size_t size() const { return m_mat->rows(); }
  void resize(size_t N)
  {
    if (N != size()) throw std::logic_error("Tried to resize a fixed-size vector");
  }
};
template <typename MAT>
CMatrixColumnAccessor<MAT> getColumnAccessor(MAT& m, size_t colIdx)
{
  return CMatrixColumnAccessor<MAT>(m, colIdx);
}

/** A vector-like wrapper for a Matrix for accessing the elements of a given
 * column with a [] operator, with offset and custom spacing.
 * \sa
 * CMatrixRowAccessorExtended,CMatrixColumnAccessor,CConstMatrixColumnAccessor,CConstMatrixColumnAccessorExtended
 */
template <typename MAT>
class CMatrixColumnAccessorExtended
{
 protected:
  MAT* m_mat;
  size_t m_colInd;
  size_t m_rowOffset;
  size_t m_elementsSpace;
  size_t howMany;

 public:
  using value_type = typename MAT::Scalar;
  using mrpt_autotype = CMatrixColumnAccessorExtended<MAT>;
  CMatrixColumnAccessorExtended(MAT& mat, size_t col, size_t offset, size_t space) :
      m_mat(&mat), m_colInd(col), m_rowOffset(offset), m_elementsSpace(space)
  {
    ASSERT_(col < mat.cols());
    howMany = (mat.rows() - m_rowOffset) / m_elementsSpace;
  }
  CMatrixColumnAccessorExtended() {}
  value_type& operator[](size_t i)
  {
    return (*m_mat)(m_rowOffset + (i * m_elementsSpace), m_colInd);
  }
  value_type operator[](size_t i) const
  {
    return (*m_mat)(m_rowOffset + (i * m_elementsSpace), m_colInd);
  }
  using iterator = detail::AccessorIterator<CMatrixColumnAccessorExtended<MAT>, value_type>;
  using const_iterator =
      detail::AccessorIterator<const CMatrixColumnAccessorExtended<MAT>, const value_type>;
  using reverse_iterator =
      detail::ReverseAccessorIterator<CMatrixColumnAccessorExtended<MAT>, value_type>;
  using const_reverse_iterator =
      detail::ReverseAccessorIterator<const CMatrixColumnAccessorExtended<MAT>, const value_type>;
  iterator begin() { return iterator(*this, 0); }
  const_iterator begin() const { return const_iterator(*this, 0); }
  iterator end() { return iterator(*this, howMany); }
  const_iterator end() const { return const_iterator(*this, howMany); }
  reverse_iterator rbegin() { return reverse_iterator(*this, howMany - 1); }
  const_reverse_iterator rbegin() const { return const_reverse_iterator(*this, howMany - 1); }
  reverse_iterator rend() { return reverse_iterator(*this, -1); }
  const_reverse_iterator rend() const { return const_reverse_iterator(*this, -1); }
  size_t size() const { return howMany; }
  void resize(size_t N)
  {
    if (N != size()) throw std::logic_error("Tried to resize a fixed-size vector");
  }
};
template <typename MAT>
CMatrixColumnAccessorExtended<MAT> getColumnAccessor(
    MAT& m, size_t colIdx, size_t offset, size_t space = 1)
{
  return CMatrixColumnAccessorExtended<MAT>(m, colIdx, offset, space);
}

/** A vector-like wrapper for a const Matrix for accessing the elements of a
 * given column with a [] operator.
 * \sa
 * CConstMatrixRowAccessor,CMatrixColumnAccessorExtended,CMatrixColumnAccessor,CConstMatrixColumnAccessorExtended
 */
template <class MAT>
class CConstMatrixColumnAccessor
{
 protected:
  const MAT* m_mat;
  size_t m_colInd;

 public:
  using value_type = typename MAT::Scalar;
  using mrpt_autotype = CConstMatrixColumnAccessor<MAT>;
  CConstMatrixColumnAccessor(const MAT& mat, size_t colIdx) : m_mat(&mat), m_colInd(colIdx)
  {
    ASSERT_(colIdx < mat.cols());
  }
  CConstMatrixColumnAccessor() {}
  value_type operator[](size_t i) const { return (*m_mat)(i, m_colInd); }
  using const_iterator =
      detail::AccessorIterator<const CConstMatrixColumnAccessor<MAT>, const value_type>;
  using const_reverse_iterator =
      detail::ReverseAccessorIterator<const CConstMatrixColumnAccessor<MAT>, const value_type>;
  const_iterator begin() const { return const_iterator(*this, 0); }
  const_iterator end() const { return const_iterator(*this, m_mat->rows()); }
  const_reverse_iterator rbegin() const { return const_reverse_iterator(*this, m_mat->rows() - 1); }
  const_reverse_iterator rend() const { return const_reverse_iterator(*this, -1); }
  size_t size() const { return m_mat->rows(); }
  void resize(size_t N)
  {
    if (N != size()) throw std::logic_error("Tried to resize a fixed-size vector");
  }
};
template <typename MAT>
CConstMatrixColumnAccessor<MAT> getColumnAccessor(const MAT& m, size_t colIdx)
{
  return CConstMatrixColumnAccessor<MAT>(m, colIdx);
}

/** A vector-like wrapper for a const Matrix for accessing the elements of a
 * given column with a [] operator, with offset and custom spacing.
 * \sa
 * CConstMatrixRowAccessorExtended,CMatrixColumnAccessor,CConstMatrixColumnAccessor,CMatrixColumnAccessorExtended
 */
template <typename MAT>
class CConstMatrixColumnAccessorExtended
{
 protected:
  const MAT* m_mat;
  size_t m_colInd;
  size_t m_rowOffset;
  size_t m_elementsSpace;
  size_t howMany;

 public:
  using value_type = typename MAT::Scalar;
  using mrpt_autotype = CMatrixColumnAccessorExtended<MAT>;
  CConstMatrixColumnAccessorExtended(const MAT& mat, size_t col, size_t offset, size_t space) :
      m_mat(&mat), m_colInd(col), m_rowOffset(offset), m_elementsSpace(space)
  {
    ASSERT_(col < mat.cols());
    howMany = (mat.rows() - m_rowOffset) / m_elementsSpace;
  }
  CConstMatrixColumnAccessorExtended() {}
  value_type operator[](size_t i) const
  {
    return (*m_mat)(m_rowOffset + (i * m_elementsSpace), m_colInd);
  }
  using const_iterator =
      detail::AccessorIterator<const CConstMatrixColumnAccessorExtended<MAT>, const value_type>;
  using const_reverse_iterator = detail::
      ReverseAccessorIterator<const CConstMatrixColumnAccessorExtended<MAT>, const value_type>;
  const_iterator begin() const { return const_iterator(*this, 0); }
  const_iterator end() const { return const_iterator(*this, howMany); }
  const_reverse_iterator rbegin() const { return const_reverse_iterator(*this, howMany - 1); }
  const_reverse_iterator rend() const { return const_reverse_iterator(*this, -1); }
  size_t size() const { return howMany; }
  void resize(size_t N)
  {
    if (N != size()) throw std::logic_error("Tried to resize a fixed-size vector");
  }
};
template <typename MAT>
CConstMatrixColumnAccessorExtended<MAT> getColumnAccessor(
    const MAT& m, size_t colIdx, size_t offset, size_t space = 1)
{
  return CConstMatrixColumnAccessorExtended<MAT>(m, colIdx, offset, space);
}

}  // namespace math
}  // namespace mrpt
