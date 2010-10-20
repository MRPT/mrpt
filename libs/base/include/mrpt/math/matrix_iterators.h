/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_matrix_iterators_H
#define  mrpt_matrix_iterators_H

#include <mrpt/math/math_frwds.h>  // Fordward declarations

#include <algorithm>
#include <iterator>

// Macro for defining iterator/const_iterator/.... in any dynamic-like matrix.
#define DECLARE_MRPT_MATRIX_ITERATORS \
	/*! @name Iterator stuff @{ */ \
	typedef mrpt::math::detail::CGenericMatrixIterator<mrpt_autotype>  iterator; \
	typedef mrpt::math::detail::CGenericMatrixConstIterator<mrpt_autotype>  const_iterator; \
	typedef std::reverse_iterator<iterator> 		reverse_iterator;  \
	typedef std::reverse_iterator<const_iterator> 	const_reverse_iterator; \
	inline iterator 		begin()   { return iterator(*this,0,0); } \
	inline iterator 		end()     { return iterator(*this,mrpt_autotype::getRowCount(),0); } \
	inline const_iterator 	begin() const	{ return const_iterator(*this,0,0); } \
	inline const_iterator 	end() const		{ return const_iterator(*this,mrpt_autotype::getRowCount(),0); } \
	inline reverse_iterator 		rbegin() 		{ return reverse_iterator(end()); } \
	inline const_reverse_iterator 	rbegin() const 	{ return const_reverse_iterator(end()); } \
	inline reverse_iterator 		rend() 			{ return reverse_iterator(begin()); } \
	inline const_reverse_iterator 	rend() const 	{ return const_reverse_iterator(begin()); } \
	/*! @} */ \


namespace mrpt {
namespace math {
namespace detail {

	/** @name Iterator-related stuff
		@{ */

	/** A random-access iterator for trasversing all the elements of a matrix, left to right, top to bottom.
	  * Only in DEBUG mode, the iterator checks for the limits of the matrix.
	  * \note The size of a matrix being accessed through an iterator should NOT be modifed or the iterator will become invalid.
	  * \note The end() iterator is represented by column=0, row=N_ROWS.
	  * \note An excelent reference for iterators theory: http://www.cs.helsinki.fi/u/tpkarkka/alglib/k06/lectures/iterators.html
	  */
	template <class MATRIXTYPE>
	struct CGenericMatrixIterator : public std::iterator<std::random_access_iterator_tag,typename MATRIXTYPE::value_type>
	{
	private:
		typedef std::iterator<std::random_access_iterator_tag,typename MATRIXTYPE::value_type> iterator_base;
		MATRIXTYPE 	*m_matrix; 				//!< A reference to the source of this iterator
		size_t		m_cur_row, m_cur_col;   //!< The iterator points to this element.
		typedef typename MATRIXTYPE::value_type T; //!< The type of the matrix elements

		inline void check_limits(bool allow_end = false) const
		{
#ifdef _DEBUG
			ASSERTMSG_(m_matrix!=NULL,"non initialized iterator");
			if (m_cur_col>=m_matrix->getColCount()) THROW_EXCEPTION("Column index out of range in iterator.")
			if (!allow_end) {
				if (m_cur_row>=m_matrix->getRowCount()) { THROW_EXCEPTION("Row index out of range in iterator.") }
			} else { 
				if (m_cur_row>m_matrix->getRowCount())  THROW_EXCEPTION("Row index out of range in iterator.") }
#endif
		}

	public:
		inline bool operator <(const CGenericMatrixIterator<MATRIXTYPE> &it2) const
		{
			if (m_cur_row<m_cur_col) return true;
			else if (m_cur_row==it2.m_cur_row)
				 return m_cur_col<it2.m_cur_col;
			else return false;
		}
		inline bool operator >(const CGenericMatrixIterator<MATRIXTYPE> &it2) const
		{
			if (m_cur_row<m_cur_col) return false;
			else if (m_cur_row==it2.m_cur_row)
				 return m_cur_col>it2.m_cur_col;
			else return true;
		}

		inline size_t getCol() const { return m_cur_col; }
		inline size_t getRow() const { return m_cur_row; }

		inline CGenericMatrixIterator() : m_matrix(NULL),m_cur_row(0),m_cur_col(0) { }
		inline CGenericMatrixIterator(MATRIXTYPE &obj, size_t start_row, size_t start_col)
			: m_matrix(&obj),m_cur_row(start_row),m_cur_col(start_col)
		{
			check_limits(true); // Dont report as error an iterator to end().
		}
		inline typename MATRIXTYPE::reference operator*() const	{
			check_limits();
			return m_matrix->get_unsafe(m_cur_row,m_cur_col);
		}
		inline CGenericMatrixIterator<MATRIXTYPE> &operator++() {
			check_limits();
			if (++m_cur_col>=m_matrix->getColCount()) { m_cur_col=0; ++m_cur_row; }
			return *this;
		}
		inline CGenericMatrixIterator<MATRIXTYPE> operator++(int)	{
			CGenericMatrixIterator<MATRIXTYPE> it=*this;
			++*this;
			return it;
		}
		inline CGenericMatrixIterator<MATRIXTYPE> &operator--()	{
			if (!m_cur_col) {
				m_cur_col = m_matrix->getColCount()-1;
				--m_cur_row;
			}
			else --m_cur_col;
			check_limits();
			return *this;
		}
		inline CGenericMatrixIterator<MATRIXTYPE> operator--(int)	{
			CGenericMatrixIterator<MATRIXTYPE> it=*this;
			--*this;
			return it;
		}
		CGenericMatrixIterator<MATRIXTYPE> &operator+=(typename iterator_base::difference_type off)	{
			const size_t N = m_matrix->getColCount();
			if (off>=0)
			{  // Advance all as columns, then count how many rows should that be in fact:
				m_cur_col+=off;
				m_cur_row+=m_cur_col/N;
				m_cur_col=m_cur_col%N;
			}
			else
			{  // Going backwards is more complicated since m_cur_* are not signed:
				const size_t idx=(m_cur_col+(m_cur_row*N)) + off;
				m_cur_row = idx/N;
				m_cur_col = idx%N;
			}
			check_limits(true);
			return *this;
		}
		inline CGenericMatrixIterator<MATRIXTYPE> operator+(typename iterator_base::difference_type off) const	{
			CGenericMatrixIterator<MATRIXTYPE> it=*this;
			it+=off;
			return it;
		}
		inline CGenericMatrixIterator<MATRIXTYPE> &operator-=(typename iterator_base::difference_type off)	{
			return (*this)+=(-off);
		}
		inline CGenericMatrixIterator<MATRIXTYPE> operator-(typename iterator_base::difference_type off) const	{
			CGenericMatrixIterator<MATRIXTYPE> it=*this;
			it-=off;
			return it;
		}
		typename iterator_base::difference_type operator-(const CGenericMatrixIterator<MATRIXTYPE> &it) const	{
			return m_matrix->getColCount()*( m_cur_row-it.m_cur_row ) + ( m_cur_col-it.m_cur_col);
		}
		inline typename MATRIXTYPE::reference operator[](typename iterator_base::difference_type off) const	{
			const size_t incr_row = off / m_matrix->getColCount();
			const size_t incr_col = off % m_matrix->getColCount();
			return (*m_matrix)(m_cur_row+incr_row,m_cur_col+incr_col);
		}
		inline bool operator==(const CGenericMatrixIterator<MATRIXTYPE> &it) const {
			return (m_cur_col==it.m_cur_col)&&(m_cur_row==it.m_cur_row)&&(m_matrix==it.m_matrix);
		}
		inline bool operator!=(const CGenericMatrixIterator<MATRIXTYPE> &it) const {
			return !(operator==(it));
		}
	}; // end CGenericMatrixIterator<MATRIXTYPE>

	/** A random-access const iterator for trasversing all the elements of a matrix, left to right, top to bottom.
	  * Only in DEBUG mode, the iterator checks for the limits of the matrix.
	  * \note The size of a matrix being accessed through an iterator should NOT be modifed or the iterator will become invalid.
	  * \note The end() iterator is represented by column=0, row=N_ROWS.
	  * \note An excelent reference for iterators theory: http://www.cs.helsinki.fi/u/tpkarkka/alglib/k06/lectures/iterators.html
	  */
	template <class MATRIXTYPE>
	struct CGenericMatrixConstIterator : public std::iterator<std::random_access_iterator_tag,typename MATRIXTYPE::value_type>
	{
	private:
		typedef std::iterator<std::random_access_iterator_tag,typename MATRIXTYPE::value_type> iterator_base;
		const MATRIXTYPE *m_matrix; 				//!< A reference to the source of this iterator
		size_t	m_cur_row, m_cur_col;   //!< The iterator points to this element.
		typedef typename MATRIXTYPE::value_type T; //!< The type of the matrix elements

		inline void check_limits(bool allow_end = false) const
		{
#ifdef _DEBUG
			ASSERTMSG_(m_matrix!=NULL,"non initialized iterator");
			if (m_cur_col>=m_matrix->getColCount()) THROW_EXCEPTION("Column index out of range in iterator.")
			if (!allow_end)
			{
				if (m_cur_row>=m_matrix->getRowCount()) { THROW_EXCEPTION("Row index out of range in iterator.") }
				else if (m_cur_row>m_matrix->getRowCount())  THROW_EXCEPTION("Row index out of range in iterator.")
			}
#endif
		}

	public:
		inline bool operator <(const CGenericMatrixConstIterator<MATRIXTYPE> &it2) const
		{
			if (m_cur_row<m_cur_col) return true;
			else if (m_cur_row==it2.m_cur_row)
				 return m_cur_col<it2.m_cur_col;
			else return false;
		}
		inline bool operator >(const CGenericMatrixConstIterator<MATRIXTYPE> &it2) const
		{
			if (m_cur_row<m_cur_col) return false;
			else if (m_cur_row==it2.m_cur_row)
				 return m_cur_col>it2.m_cur_col;
			else return true;
		}

		inline size_t getCol() const { return m_cur_col; }
		inline size_t getRow() const { return m_cur_row; }

		inline CGenericMatrixConstIterator() : m_matrix(NULL),m_cur_row(0),m_cur_col(0) { }
		inline CGenericMatrixConstIterator(const MATRIXTYPE &obj,size_t start_row, size_t start_col)
			: m_matrix(&obj),m_cur_row(start_row),m_cur_col(start_col)
		{
			check_limits(true); // Dont report as error an iterator to end().
		}
		inline typename MATRIXTYPE::const_reference operator*() const	{
			check_limits();
			return m_matrix->get_unsafe(m_cur_row,m_cur_col);
		}
		inline CGenericMatrixConstIterator<MATRIXTYPE> &operator++() {
			check_limits();
			if (++m_cur_col>=m_matrix->getColCount()) { m_cur_col=0; ++m_cur_row; }
			return *this;
		}
		inline CGenericMatrixConstIterator<MATRIXTYPE> operator++(int)	{
			CGenericMatrixConstIterator<MATRIXTYPE> it=*this;
			++*this;
			return it;
		}
		inline CGenericMatrixConstIterator<MATRIXTYPE> &operator--()	{
			if (!m_cur_col) {
				m_cur_col = m_matrix->getColCount()-1;
				--m_cur_row;
			}
			else --m_cur_col;
			check_limits();
			return *this;
		}
		inline CGenericMatrixConstIterator<MATRIXTYPE> operator--(int)	{
			CGenericMatrixConstIterator<MATRIXTYPE> it=*this;
			--*this;
			return it;
		}
		CGenericMatrixConstIterator<MATRIXTYPE> &operator+=(typename iterator_base::difference_type off)	{
			const size_t N = m_matrix->getColCount();
			if (off>=0)
			{  // Advance all as columns, then count how many rows should that be in fact:
				m_cur_col+=off;
				m_cur_row+=m_cur_col/N;
				m_cur_col=m_cur_col%N;
			}
			else
			{  // Going backwards is more complicated since m_cur_* are not signed:
				const size_t idx=(m_cur_col+m_cur_row*N) + off;
				m_cur_row = idx/N;
				m_cur_col = idx%N;
			}
			check_limits(true);
			return *this;
		}
		inline CGenericMatrixConstIterator<MATRIXTYPE> operator+(typename iterator_base::difference_type off) const	{
			CGenericMatrixConstIterator<MATRIXTYPE> it=*this;
			it+=off;
			return it;
		}
		CGenericMatrixConstIterator<MATRIXTYPE> &operator-=(typename iterator_base::difference_type off)	{
			return (*this)+=(-off);
		}
		inline CGenericMatrixConstIterator<MATRIXTYPE> operator-(typename iterator_base::difference_type off) const	{
			CGenericMatrixConstIterator<MATRIXTYPE> it=*this;
			it-=off;
			return it;
		}
		typename iterator_base::difference_type operator-(const CGenericMatrixConstIterator<MATRIXTYPE> &it) const	{
			return m_matrix->getColCount()*( m_cur_row-it.m_cur_row ) + ( m_cur_col-it.m_cur_col);
		}
		inline typename MATRIXTYPE::const_reference operator[](typename iterator_base::difference_type off) const	{
			const size_t incr_row = off / m_matrix->getColCount();
			const size_t incr_col = off % m_matrix->getColCount();
			return (*m_matrix)(m_cur_row+incr_row,m_cur_col+incr_col);
		}
		inline bool operator==(const CGenericMatrixConstIterator<MATRIXTYPE> &it) const {
			return (m_cur_col==it.m_cur_col)&&(m_cur_row==it.m_cur_row)&&(m_matrix==it.m_matrix);
		}
		inline bool operator!=(const CGenericMatrixConstIterator<MATRIXTYPE> &it) const {
			return !(operator==(it));
		}
	}; // end CGenericMatrixConstIterator<MATRIXTYPE>

	/** @} */


} // end namespace
} // end namespace
} // end namespace


#endif
