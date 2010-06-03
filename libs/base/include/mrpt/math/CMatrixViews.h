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
#ifndef CMatrixView_H
#define CMatrixView_H

#include <mrpt/math/math_frwds.h>  // Fordward declarations
#include <mrpt/math/matrix_iterators.h>  // Fordward declarations

/** \file CMatrixViews.h
  *  Matrix "views", or wrappers around existing matrices that change the way we see them, eg transpose, submatrix, etc...
  */

namespace mrpt
{
	namespace math
	{
		/** The base for all matrix views. Used just for grouping purposes
		  * It is very important to realize that CMatrixView's are just wrappers over other existing matrix objects,
		  *  so the original object MUST exist during the whole scope of these wrappers,
		  *  i.e. you can't delete the original object before its matrix views.
		  */
		class CMatrixView
		{

		};

		// The very few methods not implemented in DECLARE_COMMON_MATRIX_MEMBERS() for efficiency reasons
#define DECLARE_COMMON_MATRIX_VIEWS_MEMBERS(NUMTYPE)  \
		/*! Set all the elements to zero */ \
		void zeros() { for (size_t i=0;i<this->getRowCount();i++) for (size_t j=0;j<this->getColCount();j++) this->get_unsafe(i,j)=0; } \


#define DECLARE_MATRIX_VIEW_TYPES(_M) \
			typedef typename _M::value_type		value_type; \
			typedef typename _M::reference 		reference; \
			typedef typename _M::const_reference const_reference; \
			typedef typename _M::size_type		size_type; \
			typedef typename _M::difference_type difference_type; \


		/** A wrapper around an existing matrix (of any kind) that allows operating on the transposed matrix.
		  *  Example of usage:
		  * \code
		  *  CMatrixDouble  C(4,2);
		  *  CMatrixViewTranspose<CMatrixDouble>  Ct(C);  // Transpose view of C
		  * \endcode
		  */
		template <class MATRIXTYPE>
		class CMatrixViewTranspose : public CMatrixView
		{
		protected:
			MATRIXTYPE  &base;

		public:
			typedef CMatrixViewTranspose<MATRIXTYPE> mrpt_autotype;  //!< See ops_containers.h
			DECLARE_MATRIX_VIEW_TYPES(MATRIXTYPE)
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX
			DECLARE_MRPT_MATRIX_ITERATORS
			DECLARE_COMMON_MATRIX_MEMBERS(value_type)
			DECLARE_COMMON_MATRIX_VIEWS_MEMBERS(value_type)

			inline CMatrixViewTranspose(MATRIXTYPE &m):base(m)	{}
			inline size_t getRowCount() const { return base.getColCount(); }
			inline size_t getColCount() const { return base.getRowCount(); }
			inline value_type &get_unsafe(size_t r,size_t c)	{ return base.get_unsafe(c,r); }
			inline value_type get_unsafe(size_t r,size_t c) const	{ return base.get_unsafe(c,r); }
			inline void set_unsafe(size_t r,size_t c,value_type v)	{ base.set_unsafe(c,r,v); }
			inline value_type &operator()(const size_t r,const size_t c) { return base(c,r); }
			inline value_type operator()(const size_t r,const size_t c) const	{ return base(c,r); }
			template<typename OTHERMATRIX> inline mrpt_autotype &operator=(const OTHERMATRIX &m)	{
				const size_t NR = this->getRowCount();
				const size_t NC = this->getColCount();
				ASSERT_(NR==m.getRowCount());
				ASSERT_(NC==m.getColCount());
				for (size_t i=0;i<NR;++i) for (size_t j=0;j<NC;++j) this->get_unsafe(i,j)=m.get_unsafe(i,j);
				return *this;
			}
			inline CMatrixTemplateSize size()	{
				size_t dimsA[]={this->getRowCount(),this->getColCount()};
				CMatrixTemplateSize dims(dimsA);
				return dims;
			}
			inline void setSize(size_t r,size_t c)	{
				if ((this->getRowCount()!=r)||(this->getColCount()!=c)) throw std::logic_error("Tried to change size of a matrix view.");
			}
			inline void resize(size_t rtc)	{
				if (this->getRowCount()*this->getColCount()!=rtc) throw std::logic_error("Tried to change size of a matrix view.");
			}
		};
		template<typename MAT> CMatrixViewTranspose<MAT> getTransposed(MAT &m)	{
			return CMatrixViewTranspose<MAT>(m);
		}

		/** A wrapper around an existing const matrix (of any kind) that allows operating on the transposed matrix.
		  *  Example of usage:
		  * \code
		  *  const CMatrixDouble  C(4,2);
		  *  CConstMatrixViewTranspose<CMatrixDouble>  Ct(C);  // Transpose view of C
		  * \endcode
		  */
		template <class MAT> class CConstMatrixViewTranspose:public CMatrixView	{
		protected:
			const MAT &base;
		public:
			typedef CConstMatrixViewTranspose<MAT> mrpt_autotype;
			DECLARE_MATRIX_VIEW_TYPES(MAT)
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX
			DECLARE_MRPT_MATRIX_ITERATORS
			DECLARE_COMMON_MATRIX_MEMBERS(value_type)
			DECLARE_COMMON_MATRIX_VIEWS_MEMBERS(value_type)
			inline CConstMatrixViewTranspose(const MAT &m):base(m)	{}
			inline size_t getRowCount() const	{
				return base.getColCount();
			}
			inline size_t getColCount() const	{
				return base.getRowCount();
			}
			inline value_type get_unsafe(size_t r,size_t c) const	{
				return base.get_unsafe(c,r);
			}
			inline value_type operator()(size_t r,size_t c) const	{
				return base(c,r);
			}
			inline CMatrixTemplateSize size() const	{
				CMatrixTemplateSize dims();
				dims[0]=getRowCount();
				dims[1]=getColCount();
				return dims;
			}
			inline void setSize(size_t r,size_t c)	{
				if ((getRowCount()!=r)||(getColCount()!=c)) throw std::logic_error("Tried to change the size of a matrix view.");
			}
			inline void resize(size_t rc)	{
				if (getRowCount()*getColCount()!=rc) throw std::logic_error("Tried to chgange the size of a matrix view.");
			}
		};
		template<typename MAT> CConstMatrixViewTranspose<MAT> getTransposed(const MAT &m)	{
			return CConstMatrixViewTranspose<MAT>(m);
		}

		/** A wrapper around an existing matrix (of any kind) that allows operating on a subrange of the elements.
		  *  Example of usage:
		  * \code
		  *  CMatrixDouble  C(10,10);
		  *  CSubmatrixView<CMatrixDouble,3,2>  Csub(C,5,6);  // Csub is C([5,6,7],[6,7])
		  * \endcode
		  * \sa CArbitrarySubmatrixView
		  */
		template<typename MATRIXTYPE,size_t NR,size_t NC> class CSubmatrixView:public CMatrixView	{
		protected:
			MATRIXTYPE &base;
			size_t firstRow;
			size_t firstCol;
		public:
			typedef CSubmatrixView<MATRIXTYPE,NR,NC> mrpt_autotype;
			DECLARE_MATRIX_VIEW_TYPES(MATRIXTYPE)
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX
			DECLARE_MRPT_MATRIX_ITERATORS
			DECLARE_COMMON_MATRIX_MEMBERS(value_type)
			DECLARE_COMMON_MATRIX_VIEWS_MEMBERS(value_type)
			inline CSubmatrixView(MATRIXTYPE &m,size_t r1,size_t c1):base(m),firstRow(r1),firstCol(c1)	{}
			inline static size_t getRowCount()	{
				return NR;
			}
			inline static size_t getColCount()	{
				return NC;
			}
			inline value_type &get_unsafe(size_t r,size_t c)	{
				return base.get_unsafe(firstRow+r,firstCol+c);
			}
			inline value_type get_unsafe(size_t r,size_t c) const	{
				return base.get_unsafe(firstRow+r,firstCol+c);
			}
			inline void set_unsafe(size_t r,size_t c,value_type v)	{
				base.set_unsafe(firstRow+r,firstCol+c,v);
			}
			inline value_type &operator()(const size_t r,const size_t c)	{
				return base(firstRow+r,firstCol+c);	//base's operator() will check ;).
			}
			inline value_type operator()(const size_t r,const size_t c) const	{
				return base(firstRow+r,firstCol+c);
			}
			template<typename OTHERMATRIX> inline mrpt_autotype &operator=(const OTHERMATRIX &m)	{
				ASSERT_(NR==m.getRowCount());
				ASSERT_(NC==m.getColCount());
				for (size_t i=0;i<NR;++i) for (size_t j=0;j<NC;++j) base.get_unsafe(firstRow+i,firstCol+j)=m.get_unsafe(i,j);
				return *this;
			}
			inline static CMatrixTemplateSize size()	{
				static size_t dimsA[]={NR,NC};
				static CMatrixTemplateSize dims(dimsA);
				return dims;
			}
			static inline void setSize(size_t r,size_t c)	{
				if ((NR!=r)||(NC!=c)) throw std::logic_error("Tried to change size on a fixed-size matrix.");
			}
			static inline void resize(size_t rtc)	{
				if (NR*NC!=rtc) throw std::logic_error("Tried to change size on a fixed-size matrix.");
			}
		};

		/** A const wrapper around an existing matrix (of any kind) that allows operating on a subrange of the elements.
		  *  Example of usage:
		  * \code
		  *  const CMatrixDouble  C(10,10);
		  *  CConstSubmatrixView<CMatrixDouble,3,2>  Csub(C,5,6);  // Csub is C([5,6,7],[6,7])
		  * \endcode
		  * \sa CConstArbitrarySubmatrixView, CSubmatrixView
		  */
		template<typename MATRIXTYPE,size_t NR,size_t NC> class CConstSubmatrixView:public CMatrixView	{
		protected:
			const MATRIXTYPE &base;
			size_t firstRow;
			size_t firstCol;
		public:
			typedef CConstSubmatrixView<MATRIXTYPE,NR,NC> mrpt_autotype;
			DECLARE_MATRIX_VIEW_TYPES(MATRIXTYPE)
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX
			DECLARE_MRPT_MATRIX_ITERATORS
			DECLARE_COMMON_MATRIX_MEMBERS(value_type)
			DECLARE_COMMON_MATRIX_VIEWS_MEMBERS(value_type)
			inline CConstSubmatrixView(const MATRIXTYPE &m,size_t r1,size_t c1):base(m),firstRow(r1),firstCol(c1)	{}
			inline static size_t getRowCount()	{
				return NR;
			}
			inline static size_t getColCount()	{
				return NC;
			}
			inline value_type get_unsafe(size_t r,size_t c) const	{
				return base.get_unsafe(firstRow+r,firstCol+c);
			}
			inline value_type operator()(const size_t r,const size_t c) const	{
				return base(firstRow+r,firstCol+c);
			}
			inline static CMatrixTemplateSize size()	{
				static size_t dimsA[]={NR,NC};
				static CMatrixTemplateSize dims(dimsA);
				return dims;
			}
			static inline void setSize(size_t r,size_t c)	{
				if ((NR!=r)||(NC!=c)) throw std::logic_error("Tried to change size on a fixed-size matrix.");
			}
			static inline void resize(size_t rtc)	{
				if (NR*NC!=rtc) throw std::logic_error("Tried to change size on a fixed-size matrix.");
			}
		};

		/** A wrapper around an existing matrix (of any kind) that allows operating on a subrange of the elements.
		  *  Example of usage:
		  * \code
		  *  CMatrixDouble  C(10,10);
		  *  vector_size_t  idxs = make_vector<4,size_t>(0,1,6,7);
		  *  CArbitrarySubmatrixView<CMatrixDouble>  Csub(C,idxs);  // Csub is C(idxs,idxs)
		  * \endcode
		  * \sa CSubmatrixView
		  */
		template<typename MATRIXTYPE> class CArbitrarySubmatrixView:public CMatrixView	{
		protected:
			MATRIXTYPE &base;
			std::vector<size_t> rows;
			std::vector<size_t> cols;
		public:
			typedef CArbitrarySubmatrixView<MATRIXTYPE> mrpt_autotype;
			DECLARE_MATRIX_VIEW_TYPES(MATRIXTYPE)
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX
			DECLARE_MRPT_MATRIX_ITERATORS
			DECLARE_COMMON_MATRIX_MEMBERS(value_type)
			DECLARE_COMMON_MATRIX_VIEWS_MEMBERS(value_type)
			inline CArbitrarySubmatrixView(MATRIXTYPE &m,const std::vector<size_t> &rs,const std::vector<size_t> &cs):base(m),rows(rs),cols(cs)	{}
			inline CArbitrarySubmatrixView(MATRIXTYPE &m,const std::vector<size_t> &rows_and_cols):base(m),rows(rows_and_cols),cols(rows_and_cols)	{}
			inline CArbitrarySubmatrixView(MATRIXTYPE &m,size_t firstRow,size_t numRows,size_t firstCol,size_t numCols):base(m),rows(numRows),cols(numCols)	{
				for (size_t i=0;i<numRows;++i) rows[i]=firstRow+i;
				for (size_t i=0;i<numCols;++i) cols[i]=firstCol+i;
			}
			inline size_t getRowCount() const	{
				return rows.size();
			}
			inline size_t getColCount() const	{
				return cols.size();
			}
			inline value_type &get_unsafe(size_t r,size_t c)	{
				return base.get_unsafe(rows[r],cols[r]);
			}
			inline value_type get_unsafe(size_t r,size_t c) const	{
				return base.get_unsafe(rows[r],cols[r]);
			}
			inline void set_unsafe(size_t r,size_t c,value_type v)	{
				base.set_unsafe(rows[r],cols[c],v);
			}
			inline value_type &operator()(size_t r,size_t c)	{
				return base(rows.at(r),cols.at(c));
			}
			inline value_type operator()(size_t r,size_t c) const	{
				return base(rows.at(r),cols.at(c));
			}
			template<typename OTHERMATRIX> inline mrpt_autotype &operator=(const OTHERMATRIX &m)	{
				ASSERT_(rows.size()==m.getRowCount());
				ASSERT_(cols.size()==m.getColCount());
				for (size_t i=0;i<rows.size();++i) for (size_t j=0;j<cols.size();++j) base.get_unsafe(rows[i],cols[j])=m.get_unsafe(i,j);
				return *this;
			}
			inline CMatrixTemplateSize size() const	{
				size_t dimsA[2];
				dimsA[0]=rows.size();
				dimsA[1]=cols.size();
				return CMatrixTemplateSize(dimsA);
			}
			inline void setSize(size_t r,size_t c)	{
				if ((r!=rows.size())||(c!=cols.size())) throw std::logic_error("Tried to change size on a fixed-size matrix.");
			}
			inline void resize(size_t rtc)	{
				if (rows.size()*cols.size()!=rtc) throw std::logic_error("Tried to change size on a fixed-size matrix.");
			}
			inline void getRealRowIndices(std::vector<size_t> &vec) const	{
				vec=rows;
			}
			inline void getRealColumnIndices(std::vector<size_t> &vec) const	{
				vec=cols;
			}
			inline void deleteRow(size_t r)	{
				ASSERT_(r<rows.size());
				rows.erase(rows.begin()+r);
			}
			inline void deleteColumn(size_t c)	{
				ASSERT_(c<cols.size());
				cols.erase(cols.begin()+c);
			}
			inline size_t getProxyRow(size_t r) const	{
				ASSERT_(r<rows.size());
				return rows[r];
			}
			inline size_t getProxyCol(size_t c) const	{
				ASSERT_(c<cols.size());
				return cols[c];
			}
			inline value_type &getWithRowProxied(size_t proxyRow,size_t c)	{
				return base.get_unsafe(proxyRow,cols[c]);
			}
			inline value_type getWithRowProxied(size_t proxyRow,size_t c) const	{
				return base.get_unsafe(proxyRow,cols[c]);
			}
			inline value_type &getWithColProxied(size_t r,size_t proxyCol)	{
				return base.get_unsafe(rows[r],proxyCol);
			}
			inline value_type getWithColProxied(size_t r,size_t proxyCol) const	{
				return base.get_unsafe(rows[r],proxyCol);
			}
		};

		template<typename MATRIXTYPE> class CConstArbitrarySubmatrixView:public CMatrixView	{
		protected:
			const MATRIXTYPE &base;
			std::vector<size_t> rows;
			std::vector<size_t> cols;
		public:
			typedef CConstArbitrarySubmatrixView<MATRIXTYPE> mrpt_autotype;
			DECLARE_MATRIX_VIEW_TYPES(MATRIXTYPE)
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX
			DECLARE_MRPT_MATRIX_ITERATORS
			DECLARE_COMMON_MATRIX_MEMBERS(value_type)
			DECLARE_COMMON_MATRIX_VIEWS_MEMBERS(value_type)
			inline CConstArbitrarySubmatrixView(const MATRIXTYPE &m,const std::vector<size_t> &rs,const std::vector<size_t> &cs):base(m),rows(rs),cols(cs)	{}
			inline CConstArbitrarySubmatrixView(const MATRIXTYPE &m,const std::vector<size_t> &rows_and_cols):base(m),rows(rows_and_cols),cols(rows_and_cols)	{}
			inline CConstArbitrarySubmatrixView(const MATRIXTYPE &m,size_t firstRow,size_t numRows,size_t firstCol,size_t numCols):base(m),rows(numRows),cols(numCols)	{
				for (size_t i=0;i<numRows;++i) rows[i]=firstRow+i;
				for (size_t i=0;i<numCols;++i) cols[i]=firstCol+i;
			}
			inline size_t getRowCount() const	{
				return rows.size();
			}
			inline size_t getColCount() const	{
				return cols.size();
			}
			inline value_type get_unsafe(size_t r,size_t c) const	{
				return base.get_unsafe(rows[r],cols[r]);
			}
			inline value_type operator()(size_t r,size_t c) const	{
				return base(rows.at(r),cols.at(c));
			}
			inline CMatrixTemplateSize size() const	{
				size_t dimsA[2];
				dimsA[0]=rows.size();
				dimsA[1]=cols.size();
				return CMatrixTemplateSize(dimsA);
			}
			inline void setSize(size_t r,size_t c)	{
				if ((r!=rows.size())||(c!=cols.size())) throw std::logic_error("Tried to change size on a fixed-size matrix.");
			}
			inline void resize(size_t rtc)	{
				if (rows.size()*cols.size()!=rtc) throw std::logic_error("Tried to change size on a fixed-size matrix.");
			}
			inline void getRealRowIndices(std::vector<size_t> &vec) const	{
				vec=rows;
			}
			inline void getRealColumnIndices(std::vector<size_t> &vec) const	{
				vec=cols;
			}
			inline void deleteRow(size_t r)	{
				ASSERT_(r<rows.size());
				rows.erase(rows.begin()+r);
			}
			inline void deleteColumn(size_t c)	{
				ASSERT_(c<cols.size());
				cols.erase(cols.begin()+c);
			}
			inline size_t getProxyRow(size_t r) const	{
				ASSERT_(r<rows.size());
				return rows[r];
			}
			inline size_t getProxyCol(size_t c) const	{
				ASSERT_(c<cols.size());
				return cols[c];
			}
			inline value_type getWithRowProxied(size_t proxyRow,size_t c) const	{
				return base.get_unsafe(proxyRow,cols[c]);
			}
			inline value_type getWithColProxied(size_t r,size_t proxyCol) const	{
				return base.get_unsafe(rows[r],proxyCol);
			}
		};


		/** View the diagonal of an existing NxN matrix as a 1xN matrix (or equivalently for many MRPT methods, an N-vector).
		  *  The original matrix should be square, but any matrix can be used as long as only valid elements are accessed.
		  * Apart from the matrix-like interface, you can use "[index]" to access individual diagonal elements like if it were a vector.
		  */
		template<typename MATRIXTYPE> class CDiagonalMatrixView :public CMatrixView	{
		protected:
			MATRIXTYPE &base;
		public:
			typedef CDiagonalMatrixView<MATRIXTYPE> mrpt_autotype;
			DECLARE_MATRIX_VIEW_TYPES(MATRIXTYPE)
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX
			DECLARE_MRPT_MATRIX_ITERATORS
			DECLARE_COMMON_MATRIX_MEMBERS(value_type)
			DECLARE_COMMON_MATRIX_VIEWS_MEMBERS(value_type)
			inline CDiagonalMatrixView(MATRIXTYPE &m):base(m) {}
			inline size_t getRowCount() const	{ return 1; }
			inline size_t getColCount() const	{ return base.getRowCount(); }
			inline value_type &get_unsafe(size_t r,size_t c)	{
				if (r!=0) throw std::runtime_error("CDiagonalMatrixView is a 1xN matrix: access out of range");
				else return base.get_unsafe(c,c);
			}
			inline value_type get_unsafe(size_t r,size_t c) const	{
				if (r!=0) throw std::runtime_error("CDiagonalMatrixView is a 1xN matrix: access out of range");
				else return base.get_unsafe(c,c);
			}
			inline void set_unsafe(size_t r,size_t c,value_type v)	{
				if (r!=0) throw std::runtime_error("CDiagonalMatrixView is a 1xN matrix: access out of range");
				else return base.get_unsafe(c,c)=v;
			}
			inline value_type &operator()(size_t r,size_t c)	{
				if (r!=0) throw std::runtime_error("CDiagonalMatrixView is a 1xN matrix: access out of range");
				else return base(c,c);
			}
			inline value_type operator()(size_t r,size_t c) const	{
				if (r!=0) throw std::runtime_error("CDiagonalMatrixView is a 1xN matrix: access out of range");
				else return base(c,c);
			}
			inline value_type operator[](size_t i) const /*!< Vector-like access */	{
				return base.get_unsafe(i,i);
			}
			inline value_type &operator[](size_t i) /*!< Vector-like access */	{
				return base.get_unsafe(i,i);
			}
			/** Assigns the contents of another MATRIX or VECTOR to this diagonal. */
			template<typename CONTAINER> inline mrpt_autotype &operator=(const CONTAINER &m)	{
				const size_t nElements = m.size();
				ASSERT_(nElements==base.getColCount())
				typename CONTAINER::const_iterator itSrc;
				size_t c;
				for (c=0,itSrc=m.begin(); c<nElements; ++c, ++itSrc)
					base.get_unsafe(c,c) = *itSrc;
				return *this;
			}
			inline CMatrixTemplateSize size() const	{
				size_t dimsA[2];
				dimsA[0]=1;
				dimsA[1]=base.getColCount();
				return CMatrixTemplateSize(dimsA);
			}
			inline void setSize(size_t r,size_t c)	{
				if ((r!=1)||(c!=base.getColCount())) throw std::logic_error("Tried to change size of a diagonal matrix with another size.");
			}
			inline void resize(size_t rtc)	{
				if (rtc!=base.getColCount()) throw std::logic_error("Tried to change size of a diagonal matrix with another size.");
			}
		};

		namespace detail	{
			/** Template class for matrix accessor's iterators.
			  * \sa CMatrixRowAccessor,CMatrixColumnAccessor
			  */
			template<typename A,typename T> class AccessorIterator	{
			protected:
				A *base;
				int pos;
			public:
				//typedefs for iterator_traits:
				typedef std::random_access_iterator_tag iterator_category;
				typedef T value_type;
				typedef int difference_type;
				typedef T *pointer;
				typedef T &reference;

				inline AccessorIterator(A &obj,size_t N):base(&obj),pos(N)	{}
				inline T &operator*() const	{
					return (*base)[pos];
				}
				inline AccessorIterator<A,T> &operator++()	{
					++pos;
					return *this;
				}
				inline AccessorIterator<A,T> operator++(int)	{
					AccessorIterator<A,T> it=*this;
					++*this;
					return it;
				}
				inline AccessorIterator<A,T> &operator--()	{
					--pos;
					return *this;
				}
				inline AccessorIterator<A,T> operator--(int)	{
					AccessorIterator<A,T> it=*this;
					--*this;
					return it;
				}
				inline AccessorIterator<A,T> &operator+=(int off)	{
					pos+=off;
					return *this;
				}
				inline AccessorIterator<A,T> operator+(int off) const	{
					AccessorIterator<A,T> it=*this;
					it+=off;
					return it;
				}
				inline AccessorIterator<A,T> &operator-=(int off)	{
					pos-=off;
					return *this;
				}
				inline AccessorIterator<A,T> operator-(int off) const	{
					AccessorIterator<A,T> it=*this;
					it-=off;
					return it;
				}
				inline int operator-(const AccessorIterator<A,T> &it) const	{
					return pos-it.pos;
				}
				inline T &operator[](int off) const	{
					return (*base)[pos+off];
				}
				inline bool operator==(const AccessorIterator<A,T> &it) const	{
					return (pos==it.pos)&&(base==it.base);
				}
				inline bool operator!=(const AccessorIterator<A,T> &it) const	{
					return !(operator==(it));
				}
			};

			/** Template class for matrix accessor's iterators.
			  * \sa CMatrixRowAccessor,CMatrixColumnAccessor
			  */
			template<typename A,typename T> class ReverseAccessorIterator	{
			protected:
				A *base;
				int pos;
			public:
				//typedefs for iterator_traits:
				typedef std::random_access_iterator_tag iterator_category;
				typedef T value_type;
				typedef int difference_type;
				typedef T *pointer;
				typedef T &reference;

				inline ReverseAccessorIterator(A &obj,size_t N):base(&obj),pos(N)	{}
				inline T &operator*() const	{
					return (*base)[pos];
				}
				inline ReverseAccessorIterator<A,T> &operator++()	{
					--pos;
					return *this;
				}
				inline ReverseAccessorIterator<A,T> operator++(int)	{
					ReverseAccessorIterator<A,T> it=*this;
					++*this;	//Yes, that's right.
					return it;
				}
				inline ReverseAccessorIterator<A,T> &operator--()	{
					++pos;
					return *this;
				}
				inline ReverseAccessorIterator<A,T> operator--(int)	{
					ReverseAccessorIterator<A,T> it=*this;
					--*this;	//Yes, that's right.
					return it;
				}
				inline ReverseAccessorIterator<A,T> &operator+=(int off)	{
					pos-=off;
					return *this;
				}
				inline ReverseAccessorIterator<A,T> operator+(int off) const	{
					ReverseAccessorIterator<A,T> it=*this;
					it+=off;	//Yes, that's right.
					return it;
				}
				inline AccessorIterator<A,T> &operator-=(int off)	{
					pos+=off;
					return *this;
				}
				inline AccessorIterator<A,T> operator-(int off) const	{
					ReverseAccessorIterator<A,T> it=*this;
					it-=off;	//Yes, that's right
					return it;
				}
				inline int operator-(const ReverseAccessorIterator<A,T> &it) const	{
					return it.pos-pos;
				}
				inline T &operator[](int off) const	{
					return (*base)[pos-off];
				}
				inline bool operator==(const ReverseAccessorIterator<A,T> &it) const	{
					return (pos==it.pos)&&(&base==&it.base);
				}
				inline bool operator!=(const ReverseAccessorIterator<A,T> &it) const	{
					return !(operator==(it));
				}
			};
		}	//End of detail namespace

		/** A vector-like wrapper for a Matrix for accessing the elements of a given column with a [] operator.
		  * \sa CMatrixRowAccessor,CMatrixColumnAccessorExtended,CConstMatrixColumnAccessor,CConstMatrixColumnAccessorExtended
		  */
		template <typename MAT> class CMatrixColumnAccessor	{
		protected:
			MAT *m_mat;
			size_t	m_colInd;
		public:
			typedef typename MAT::value_type value_type;
			typedef CMatrixColumnAccessor<MAT> mrpt_autotype;
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_VECTOR
			DECLARE_COMMON_CONTAINERS_MEMBERS(value_type)
			inline CMatrixColumnAccessor(MAT &mat, size_t colIdx) : m_mat(&mat), m_colInd(colIdx) { ASSERT_(colIdx<mat.getColCount()) }
			inline CMatrixColumnAccessor()	{}
			inline value_type &operator[](const size_t i) { return (*m_mat)(i,m_colInd); }
			inline value_type operator[](const size_t i) const { return (*m_mat)(i,m_colInd); }
			typedef detail::AccessorIterator<CMatrixColumnAccessor<MAT>,value_type> iterator;
			typedef detail::AccessorIterator<const CMatrixColumnAccessor<MAT>,const value_type> const_iterator;
			typedef detail::ReverseAccessorIterator<CMatrixColumnAccessor<MAT>,value_type> reverse_iterator;
			typedef detail::ReverseAccessorIterator<const CMatrixColumnAccessor<MAT>,const value_type> const_reverse_iterator;
			inline iterator begin()	{
				return iterator(*this,0);
			}
			inline const_iterator begin() const	{
				return const_iterator(*this,0);
			}
			inline iterator end()	{
				return iterator(*this,m_mat->getRowCount());
			}
			inline const_iterator end() const	{
				return const_iterator(*this,m_mat->getRowCount());
			}
			inline reverse_iterator rbegin()	{
				return reverse_iterator(*this,m_mat->getRowCount()-1);
			}
			inline const_reverse_iterator rbegin() const	{
				return const_reverse_iterator(*this,m_mat->getRowCount()-1);
			}
			inline reverse_iterator rend()	{
				return reverse_iterator(*this,-1);
			}
			inline const_reverse_iterator rend() const	{
				return const_reverse_iterator(*this,-1);
			}
			inline size_t size() const	{
				return m_mat->getRowCount();
			}
			inline void resize(size_t N)	{
				if (N!=size()) throw std::logic_error("Tried to resize a fixed-size vector");
			}
		};
		template<typename MAT> inline CMatrixColumnAccessor<MAT> getColumnAccessor(MAT &m,size_t colIdx)	{
			return CMatrixColumnAccessor<MAT>(m,colIdx);
		}

		/** A vector-like wrapper for a Matrix for accessing the elements of a given column with a [] operator, with offset and custom spacing.
		  * \sa CMatrixRowAccessorExtended,CMatrixColumnAccessor,CConstMatrixColumnAccessor,CConstMatrixColumnAccessorExtended
		  */
		template<typename MAT>
		class CMatrixColumnAccessorExtended	{
		protected:
			MAT *m_mat;
			size_t m_colInd;
			size_t m_rowOffset;
			size_t m_elementsSpace;
			size_t howMany;
		public:
			typedef typename MAT::value_type value_type;
			typedef CMatrixColumnAccessorExtended<MAT> mrpt_autotype;
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_VECTOR
			DECLARE_COMMON_CONTAINERS_MEMBERS(mrpt_autotype)
			inline CMatrixColumnAccessorExtended(MAT &mat,size_t col,size_t offset,size_t space):m_mat(&mat),m_colInd(col),m_rowOffset(offset),m_elementsSpace(space)	{
				ASSERT_(col<mat.getColCount());
				howMany=(mat.getRowCount()-m_rowOffset)/m_elementsSpace;
			}
			inline CMatrixColumnAccessorExtended()	{}
			inline value_type &operator[](size_t i)	{
				return (*m_mat)(m_rowOffset+(i*m_elementsSpace),m_colInd);
			}
			inline value_type operator[](size_t i) const	{
				return (*m_mat)(m_rowOffset+(i*m_elementsSpace),m_colInd);
			}
			typedef detail::AccessorIterator<CMatrixColumnAccessorExtended<MAT>,value_type> iterator;
			typedef detail::AccessorIterator<const CMatrixColumnAccessorExtended<MAT>,const value_type> const_iterator;
			typedef detail::ReverseAccessorIterator<CMatrixColumnAccessorExtended<MAT>,value_type> reverse_iterator;
			typedef detail::ReverseAccessorIterator<const CMatrixColumnAccessorExtended<MAT>,const value_type> const_reverse_iterator;
			inline iterator begin()	{
				return iterator(*this,0);
			}
			inline const_iterator begin() const	{
				return const_iterator(*this,0);
			}
			inline iterator end()	{
				return iterator(*this,howMany);
			}
			inline const_iterator end() const	{
				return const_iterator(*this,howMany);
			}
			inline reverse_iterator rbegin()	{
				return reverse_iterator(*this,howMany-1);
			}
			inline const_reverse_iterator rbegin() const	{
				return const_reverse_iterator(*this,howMany-1);
			}
			inline reverse_iterator rend()	{
				return reverse_iterator(*this,-1);
			}
			inline const_reverse_iterator rend() const	{
				return const_reverse_iterator(*this,-1);
			}
			inline size_t size() const	{
				return howMany();
			}
			inline void resize(size_t N)	{
				if (N!=size()) throw std::logic_error("Tried to resize a fixed-size vector");
			}
		};
		template<typename MAT> inline CMatrixColumnAccessorExtended<MAT> getColumnAccessor(MAT &m,size_t colIdx,size_t offset,size_t space=1)	{
			return CMatrixColumnAccessorExtended<MAT>(m,colIdx,offset,space);
		}

		/** A vector-like wrapper for a const Matrix for accessing the elements of a given column with a [] operator.
		  * \sa CConstMatrixRowAccessor,CMatrixColumnAccessorExtended,CMatrixColumnAccessor,CConstMatrixColumnAccessorExtended
		  */
		template<class MAT>
		class CConstMatrixColumnAccessor	{
		protected:
			const MAT *m_mat;
			size_t m_colInd;
		public:
			typedef typename MAT::value_type value_type;
			typedef CConstMatrixColumnAccessor<MAT> mrpt_autotype;
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_VECTOR
			DECLARE_COMMON_CONTAINERS_MEMBERS(value_type)
			inline CConstMatrixColumnAccessor(const MAT &mat,size_t colIdx):m_mat(&mat),m_colInd(colIdx)	{
				ASSERT_(colIdx<mat.getColCount());
			}
			inline CConstMatrixColumnAccessor()	{}
			inline value_type operator[](size_t i) const	{
				return (*m_mat)(i,m_colInd);
			}
			typedef detail::AccessorIterator<const CConstMatrixColumnAccessor<MAT>,const value_type> const_iterator;
			typedef detail::ReverseAccessorIterator<const CConstMatrixColumnAccessor<MAT>,const value_type> const_reverse_iterator;
			inline const_iterator begin() const	{
				return const_iterator(*this,0);
			}
			inline const_iterator end() const	{
				return const_iterator(*this,m_mat->getRowCount());
			}
			inline const_reverse_iterator rbegin() const	{
				return const_reverse_iterator(*this,m_mat->getRowCount()-1);
			}
			inline const_reverse_iterator rend() const	{
				return const_reverse_iterator(*this,-1);
			}
			inline size_t size() const	{
				return m_mat->getRowCount();
			}
			inline void resize(size_t N)	{
				if (N!=size()) throw std::logic_error("Tried to resize a fixed-size vector");
			}
		};
		template<typename MAT> inline CConstMatrixColumnAccessor<MAT> getColumnAccessor(const MAT &m,size_t colIdx)	{
			return CConstMatrixColumnAccessor<MAT>(m,colIdx);
		}

		/** A vector-like wrapper for a const Matrix for accessing the elements of a given column with a [] operator, with offset and custom spacing.
		  * \sa CConstMatrixRowAccessorExtended,CMatrixColumnAccessor,CConstMatrixColumnAccessor,CMatrixColumnAccessorExtended
		  */
		template<typename MAT>
		class CConstMatrixColumnAccessorExtended	{
		protected:
			const MAT *m_mat;
			size_t m_colInd;
			size_t m_rowOffset;
			size_t m_elementsSpace;
			size_t howMany;
		public:
			typedef typename MAT::value_type value_type;
			typedef CMatrixColumnAccessorExtended<MAT> mrpt_autotype;
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_VECTOR
			DECLARE_COMMON_CONTAINERS_MEMBERS(value_type)
			inline CConstMatrixColumnAccessorExtended(const MAT &mat,size_t col,size_t offset,size_t space):m_mat(&mat),m_colInd(col),m_rowOffset(offset),m_elementsSpace(space)	{
				ASSERT_(col<mat.getColCount());
				howMany=(mat.getRowCount()-m_rowOffset)/m_elementsSpace;
			}
			inline CConstMatrixColumnAccessorExtended()	{}
			inline value_type operator[](size_t i) const	{
				return (*m_mat)(m_rowOffset+(i*m_elementsSpace),m_colInd);
			}
			typedef detail::AccessorIterator<const CConstMatrixColumnAccessorExtended<MAT>,const value_type> const_iterator;
			typedef detail::ReverseAccessorIterator<const CConstMatrixColumnAccessorExtended<MAT>,const value_type> const_reverse_iterator;
			inline const_iterator begin() const	{
				return const_iterator(*this,0);
			}
			inline const_iterator end() const	{
				return const_iterator(*this,howMany);
			}
			inline const_reverse_iterator rbegin() const	{
				return const_reverse_iterator(*this,howMany-1);
			}
			inline const_reverse_iterator rend() const	{
				return const_reverse_iterator(*this,-1);
			}
			inline size_t size() const	{
				return howMany;
			}
			inline void resize(size_t N)	{
				if (N!=size()) throw std::logic_error("Tried to resize a fixed-size vector");
			}
		};
		template<typename MAT> inline CConstMatrixColumnAccessorExtended<MAT> getColumnAccessor(const MAT &m,size_t colIdx,size_t offset,size_t space=1)	{
			return CConstMatrixColumnAccessorExtended<MAT>(m,colIdx,offset,space);
		}

		/** A vector-like wrapper for a Matrix for accessing the elements of a given row with a [] operator.
		  * \sa CMatrixColumnAccessor,CMatrixRowAccessorExtended,CConstMatrixRowAccessor,CConstMatrixRowAccessorExtended
		  */
		template <typename MAT>
		class CMatrixRowAccessor
		{
		protected:
			MAT *m_mat;
			size_t	m_rowInd;
		public:
			typedef typename MAT::value_type value_type;
			typedef CMatrixRowAccessor<MAT> mrpt_autotype;
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_VECTOR
			DECLARE_COMMON_CONTAINERS_MEMBERS(value_type)
			inline CMatrixRowAccessor(MAT &mat, size_t rowIdx) : m_mat(&mat), m_rowInd(rowIdx) { ASSERT_(rowIdx<mat.getRowCount()) }
			inline CMatrixRowAccessor()	{}
			inline value_type &operator[](const size_t i) { return (*m_mat)(m_rowInd,i); }
			inline value_type operator[](const size_t i) const { return (*m_mat)(m_rowInd,i); }
			typedef detail::AccessorIterator<CMatrixRowAccessor<MAT>,value_type> iterator;
			typedef detail::AccessorIterator<const CMatrixRowAccessor<MAT>,const value_type> const_iterator;
			typedef detail::ReverseAccessorIterator<CMatrixRowAccessor<MAT>,value_type> reverse_iterator;
			typedef detail::ReverseAccessorIterator<const CMatrixRowAccessor<MAT>,const value_type> const_reverse_iterator;
			inline iterator begin()	{
				return iterator(*this,0);
			}
			inline const_iterator begin() const	{
				return const_iterator(*this,0);
			}
			inline iterator end()	{
				return iterator(*this,m_mat->getColCount());
			}
			inline const_iterator end() const	{
				return const_iterator(*this,m_mat->getColCount());
			}
			inline reverse_iterator rbegin()	{
				return reverse_iterator(*this,m_mat->getColCount()-1);
			}
			inline const_reverse_iterator rbegin() const	{
				return const_reverse_iterator(*this,m_mat.getColCount()-1);
			}
			inline reverse_iterator rend()	{
				return reverse_iterator(*this,-1);
			}
			inline const_reverse_iterator rend() const	{
				return const_reverse_iterator(*this,-1);
			}
			inline size_t size() const	{
				return m_mat->getColCount();
			}
			inline void resize(size_t N)	{
				if (N!=size()) throw std::logic_error("Tried to resize a fixed-size vector");
			}
		};
		template<typename MAT> inline CMatrixRowAccessor<MAT> getRowAccessor(MAT &m,size_t rowIdx)	{
			return CMatrixRowAccessor<MAT>(m,rowIdx);
		}

		/** A vector-like wrapper for a Matrix for accessing the elements of a given row with a [] operator, with offset and custom spacing.
		  * \sa CMatrixColumnAccessorExtended,CMatrixRowAccessor,CConstMatrixRowAccessor,CConstMatrixRowAccessorExtended
		  */
		template<class MAT>
		class CMatrixRowAccessorExtended	{
		protected:
			MAT *m_mat;
			size_t m_rowInd;
			size_t m_colOffset;
			size_t m_elementsSpace;
			size_t howMany;
		public:
			typedef typename MAT::value_type value_type;
			typedef CMatrixRowAccessorExtended<MAT> mrpt_autotype;
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_VECTOR
			DECLARE_COMMON_CONTAINERS_MEMBERS(value_type)
			inline CMatrixRowAccessorExtended(MAT &mat,size_t row,size_t offset,size_t space):m_mat(&mat),m_rowInd(row),m_colOffset(offset),m_elementsSpace(space)	{
				ASSERT_(row<mat.getRowCount());
				howMany=(mat.getColCount()-m_colOffset)/m_elementsSpace;
			}
			inline CMatrixRowAccessorExtended()	{}
			inline value_type &operator[](size_t i)	{
				return (*m_mat)(m_rowInd,m_colOffset+(i*m_elementsSpace));
			}
			inline value_type operator[](size_t i) const {
				return (*m_mat)(m_rowInd,m_colOffset+(i*m_elementsSpace));
			}
			typedef detail::AccessorIterator<CMatrixRowAccessorExtended<MAT>,value_type> iterator;
			typedef detail::AccessorIterator<const CMatrixRowAccessorExtended<MAT>,const value_type> const_iterator;
			typedef detail::ReverseAccessorIterator<CMatrixRowAccessorExtended<MAT>,value_type> reverse_iterator;
			typedef detail::ReverseAccessorIterator<const CMatrixRowAccessorExtended<MAT>,const value_type> const_reverse_iterator;
			inline iterator begin()	{
				return iterator(*this,0);
			}
			inline const_iterator begin() const	{
				return const_iterator(*this,0);
			}
			inline iterator end()	{
				return iterator(*this,howMany);
			}
			inline const_iterator end() const	{
				return const_iterator(*this,howMany);
			}
			inline reverse_iterator rbegin()	{
				return reverse_iterator(*this,howMany-1);
			}
			inline const_reverse_iterator rbegin() const	{
				return const_reverse_iterator(*this,howMany-1);
			}
			inline reverse_iterator rend()	{
				return reverse_iterator(*this,-1);
			}
			inline const_reverse_iterator rend() const	{
				return const_reverse_iterator(*this,-1);
			}
			inline size_t size() const	{
				return howMany;
			}
			inline void resize(size_t N)	{
				if (N!=size()) throw std::logic_error("Tried to resize a fixed-size vector");
			}
		};
		template<typename MAT> inline CMatrixRowAccessorExtended<MAT> getRowAccessor(MAT &m,size_t rowIdx,size_t offset,size_t space=1)	{
			return CMatrixRowAccessor<MAT>(m,rowIdx,offset,space);
		}

		/** A vector-like wrapper for a const Matrix for accessing the elements of a given row with a [] operator.
		  * \sa CConstMatrixColumnAccessor,CMatrixRowAccessorExtended,CMatrixRowAccessor,CConstMatrixRowAccessorExtended
		  */
		template<class MAT>
		class CConstMatrixRowAccessor	{
		protected:
			const MAT *m_mat;
			size_t m_rowInd;
		public:
			typedef typename MAT::value_type value_type;
			typedef CConstMatrixRowAccessor<MAT> mrpt_autotype;
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_VECTOR
			DECLARE_COMMON_CONTAINERS_MEMBERS(value_type)
			inline CConstMatrixRowAccessor(const MAT &mat,size_t row):m_mat(&mat),m_rowInd(row)	{
				ASSERT_(row<mat.getRowCount());
			}
			inline CConstMatrixRowAccessor()	{}
			inline value_type operator[](size_t i) const	{
				return (*m_mat)(m_rowInd,i);
			}
			typedef detail::AccessorIterator<const CConstMatrixRowAccessor<MAT>,const value_type> const_iterator;
			typedef detail::ReverseAccessorIterator<const CConstMatrixRowAccessor<MAT>,const value_type> const_reverse_iterator;
			inline const_iterator begin() const	{
				return const_iterator(*this,0);
			}
			inline const_iterator end() const	{
				return const_iterator(*this,m_mat->getColCount());
			}
			inline const_reverse_iterator rbegin() const	{
				return const_reverse_iterator(*this,m_mat->getColCount()-1);
			}
			inline const_reverse_iterator rend() const	{
				return const_reverse_iterator(*this,-1);
			}
			inline size_t size() const	{
				return m_mat->getColCount();
			}
			inline void resize(size_t N)	{
				if (N!=size()) throw std::logic_error("Tried to resize a fixed-size vector");
			}
		};
		template<typename MAT> inline CConstMatrixRowAccessor<MAT> getRowAccessor(const MAT &m,size_t rowIdx)	{
			return CMatrixRowAccessor<MAT>(m,rowIdx);
		}

		/** A vector-like wrapper for a const Matrix for accessing the elements of a given row with a [] operator, with offset and custom spacing.
		  * \sa CConstMatrixColumnAccessorExtended,CMatrixRowAccessor,CConstMatrixRowAccessor,CMatrixRowAccessorExtended
		  */
		template<class MAT>
		class CConstMatrixRowAccessorExtended	{
		protected:
			const MAT *m_mat;
			size_t m_rowInd;
			size_t m_colOffset;
			size_t m_elementsSpace;
			size_t howMany;
		public:
			typedef typename MAT::value_type value_type;
			typedef CConstMatrixRowAccessorExtended<MAT> mrpt_autotype;
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_VECTOR
			DECLARE_COMMON_CONTAINERS_MEMBERS(value_type)
			inline CConstMatrixRowAccessorExtended(const MAT &mat,size_t row,size_t offset,size_t space):m_mat(&mat),m_rowInd(row),m_colOffset(offset),m_elementsSpace(space)	{
				ASSERT_(row<mat.getRowCount());
				howMany=(mat.getColCount()-m_colOffset)/m_elementsSpace;
			}
			inline CConstMatrixRowAccessorExtended()	{}
			inline value_type operator[](size_t i) const	{
				return (*m_mat)(m_rowInd,m_colOffset+(i*m_elementsSpace));
			}
			typedef detail::AccessorIterator<const CConstMatrixRowAccessorExtended<MAT>,const value_type> const_iterator;
			typedef detail::ReverseAccessorIterator<const CConstMatrixRowAccessorExtended<MAT>,const value_type> const_reverse_iterator;
			inline const_iterator begin() const	{
				return const_iterator(*this,0);
			}
			inline const_iterator end() const	{
				return const_iterator(*this,howMany);
			}
			inline const_reverse_iterator rbegin() const	{
				return const_reverse_iterator(*this,howMany-1);
			}
			inline const_reverse_iterator rend() const	{
				return const_reverse_iterator(*this,-1);
			}
			inline size_t size() const	{
				return howMany;
			}
			inline void resize(size_t N)	{
				if (N!=size()) throw std::logic_error("Tried to resize a fixed-size vector");
			}
		};
		template<typename MAT> inline CConstMatrixRowAccessorExtended<MAT> getRowAccessor(const MAT &m,size_t rowIdx,size_t offset,size_t space=1)	{
			return CConstMatrixRowAccessorExtended<MAT>(m,rowIdx,offset,space);
		}

		template<typename VEC> class CVectorRowWrapper:public CMatrixView	{
		private:
			VEC &vec;
		public:
			typedef CVectorRowWrapper<VEC> mrpt_autotype;
			DECLARE_MATRIX_VIEW_TYPES(VEC)
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX
			DECLARE_MRPT_MATRIX_ITERATORS
			DECLARE_COMMON_MATRIX_MEMBERS(value_type)
			DECLARE_COMMON_MATRIX_VIEWS_MEMBERS(value_type)
			inline CVectorRowWrapper(VEC &v):vec(v)	{}
			inline static size_t getRowCount()	{
				return 1;
			}
			inline size_t getColCount() const	{
				return vec.size();
			}
			inline value_type &get_unsafe(size_t i,size_t j)	{
				return vec[j];
			}
			inline value_type get_unsafe(size_t i,size_t j) const	{
				return vec[j];
			}
			inline void set_unsafe(size_t i,size_t j,value_type v)	{
				vec[j]=v;
			}
			inline value_type &operator()(size_t i,size_t j)	{
				ASSERT_(i==0);
				return vec[j];
			}
			inline value_type operator()(size_t i,size_t j) const	{
				ASSERT_(i==0);
				return vec[j];
			}
			inline void setSize(size_t r,size_t c)	{
				ASSERT_(r==1);
				vec.resize(c);
			}
			inline void resize(size_t rc)	{
				vec.resize(rc);
			}
		};
		template<typename VEC> inline CVectorRowWrapper<VEC> getAsRow(VEC &v)	{
			return CVectorRowWrapper<VEC>(v);
		}

		template<typename VEC> class CConstVectorRowWrapper:public CMatrixView	{
		private:
			const VEC &vec;
		public:
			typedef CConstVectorRowWrapper<VEC> mrpt_autotype;
			DECLARE_MATRIX_VIEW_TYPES(VEC)
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX
			DECLARE_MRPT_MATRIX_ITERATORS
			DECLARE_COMMON_MATRIX_MEMBERS(value_type)
			DECLARE_COMMON_MATRIX_VIEWS_MEMBERS(value_type)
			inline CConstVectorRowWrapper(const VEC &v):vec(v)	{}
			inline static size_t getRowCount()	{
				return 1;
			}
			inline size_t getColCount() const	{
				return vec.size();
			}
			inline value_type get_unsafe(size_t i,size_t j) const	{
				return vec[j];
			}
			inline value_type operator()(size_t i,size_t j) const	{
				ASSERT_(i==0);
				return vec[j];
			}
			inline void setSize(size_t r,size_t c)	{
				ASSERT_(r==0);
				if (c!=vec.size()) throw std::logic_error("Tried to resize a const object.");
			}
			inline void resize(size_t rc)	{
				if (rc!=vec.size()) throw std::logic_error("Tried to resize a const object.");
			}
		};
		template<typename VEC> inline CConstVectorRowWrapper<VEC> getAsRow(const VEC &v)	{
			return CConstVectorRowWrapper<VEC>(v);
		}

		template<typename VEC> class CVectorColumnWrapper:public CMatrixView	{
		private:
			VEC &vec;
		public:
			typedef CVectorColumnWrapper<VEC> mrpt_autotype;
			DECLARE_MATRIX_VIEW_TYPES(VEC)
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX
			DECLARE_MRPT_MATRIX_ITERATORS
			DECLARE_COMMON_MATRIX_MEMBERS(value_type)
			DECLARE_COMMON_MATRIX_VIEWS_MEMBERS(value_type)
			inline CVectorColumnWrapper(VEC &v):vec(v)	{}
			inline size_t getRowCount() const	{
				return vec.size();
			}
			inline static size_t getColCount()	{
				return 1;
			}
			inline value_type &get_unsafe(size_t i,size_t j)	{
				return vec[i];
			}
			inline value_type get_unsafe(size_t i,size_t j) const	{
				return vec[i];
			}
			inline void set_unsafe(size_t i,size_t j,value_type v)	{
				vec[i]=v;
			}
			inline value_type &operator()(size_t i,size_t j)	{
				ASSERT_(j==0);
				return vec[i];
			}
			inline value_type operator()(size_t i,size_t j) const	{
				ASSERT_(j==0);
				return vec[i];
			}
			inline void setSize(size_t r,size_t c)	{
				ASSERT_(c==1);
				vec.resize(r);
			}
			inline void resize(size_t rc)	{
				vec.resize(rc);
			}
		};
		template<typename VEC> inline CVectorColumnWrapper<VEC> getAsRow(VEC &v)	{
			return CVectorColumnWrapper<VEC>(v);
		}

		template<typename VEC> class CConstVectorColumnWrapper:public CMatrixView	{
		private:
			const VEC &vec;
		public:
			typedef CConstVectorColumnWrapper<VEC> mrpt_autotype;
			DECLARE_MATRIX_VIEW_TYPES(VEC)
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX
			DECLARE_MRPT_MATRIX_ITERATORS
			DECLARE_COMMON_MATRIX_MEMBERS(value_type)
			DECLARE_COMMON_MATRIX_VIEWS_MEMBERS(value_type)
			inline CConstVectorColumnWrapper(const VEC &v):vec(v)	{}
			inline size_t getRowCount() const	{
				return vec.size();
			}
			inline size_t static getColCount()	{
				return 1;
			}
			inline value_type get_unsafe(size_t i,size_t j) const	{
				return vec[i];
			}
			inline value_type operator()(size_t i,size_t j) const	{
				ASSERT_(j==0);
				return vec[i];
			}
			inline void setSize(size_t r,size_t c)	{
				ASSERT_(c==1);
				if (r!=vec.size()) throw std::logic_error("Tried to resize a const object.");
			}
			inline void resize(size_t rc)	{
				if (rc!=vec.size()) throw std::logic_error("Tried to resize a const object.");
			}
		};
		template<typename VEC> inline CConstVectorColumnWrapper<VEC> getAsRow(VEC &v)	{
			return CConstVectorColumnWrapper<VEC>(v);
		}



		/** Access to two matrices joint horizontally [A|B]  \sa JointAccessor
		  */
		template<typename M1,typename M2> class JointHorizontalAccessor	{
		private:
			M1 &first;
			M2 &second;
		public:
			typedef typename M1::value_type value_type;
			const size_t C1,C2;
			const size_t R;
			inline JointHorizontalAccessor(M1 &f,M2 &s):first(f),second(s),C1(f.getColCount()),C2(s.getColCount()),R(f.getRowCount())	{
				ASSERT_(s.getRowCount()==R);
				//TODO: ¿assert M1::value_type==M2::value_type? ¿Se podría hacer con typeid?
			}
			inline value_type &getFirst(size_t dim1,size_t dim2)	{
				return first.get_unsafe(dim1,dim2);
			}
			inline value_type &getSecond(size_t dim1,size_t dim2)	{
				return second.get_unsafe(dim1,dim2);
			}
			inline size_t getRowCount() const	{
				return R;
			}
			inline size_t getColCount() const	{
				return C1+C2;
			}
			inline value_type &operator()(size_t i,size_t j)	{
				return (j<C1)?first(i,j):second(i,j-C1);
			}
			inline value_type operator()(size_t i,size_t j) const	{
				return (j<C1)?first(i,j):second(i,j-C1);
			}
			inline value_type &get_unsafe(size_t i,size_t j)	{
				return (j<C1)?first.get_unsafe(i,j):second.get_unsafe(i,j-C1);
			}
			inline value_type get_unsafe(size_t i,size_t j) const	{
				return (j<C1)?first.get_unsafe(i,j):second.get_unsafe(i,j-C1);
			}
			inline void set_unsafe(size_t i,size_t j,value_type v)	{
				if (j<C1) first.set_unsafe(i,j,v);
				else second.set_unsafe(i,j-C1,v);
			}
		};
		/** Access to two matrices joint vertically [A;B]  \sa JointAccessor
		  */
		template<typename M1,typename M2> class JointVerticalAccessor	{
		private:
			M1 &first;
			M2 &second;
		public:
			typedef typename M1::value_type value_type;
			const size_t C1,C2;	//Despite their names, these variables refer to the amount of rows!
			const size_t R;	//Despite its name, this variable refers to the amount of columns!
			inline JointVerticalAccessor(M1 &f,M2 &s):first(f),second(s),C1(f.getRowCount()),C2(s.getRowCount()),R(f.getColCount())	{
				ASSERT_(s.getColCount()==R);
				//TODO: ¿assert M1::value_type==M2::value_type? ¿Se podría hacer con typeid?
			}
			//getFirst and getSecond methods act like transposed views. This is done to help the division methods.
			inline value_type &getFirst(size_t dim1,size_t dim2)	{
				return first.get_unsafe(dim2,dim1);
			}
			inline value_type &getSecond(size_t dim1,size_t dim2)	{
				return second.get_unsafe(dim2,dim1);
			}
			inline size_t getRowCount() const	{
				return C1+C2;
			}
			inline size_t getColCount() const	{
				return R;
			}
			//These operators DO NOT act like transposed views.
			inline value_type &operator()(size_t i,size_t j)	{
				return (i<C1)?first(i,j):second(i-C1,j);
			}
			inline value_type operator()(size_t i,size_t j) const	{
				return (i<C1)?first(i,j):second(i-C1,j);
			}
			inline value_type &get_unsafe(size_t i,size_t j)	{
				return (i<C1)?first.get_unsafe(i,j):second.get_unsafe(i-C1,j);
			}
			inline value_type get_unsafe(size_t i,size_t j) const	{
				return (i<C1)?first.get_unsafe(i,j):second.get_unsafe(i-C1,j);
			}
			inline void set_unsafe(size_t i,size_t j,value_type v)	{
				if (i<C1) first.set_unsafe(i,j,v);
				else second.set_unsafe(i-C1,j,v);
			}
		};

		//THIS IS A VERY SPECIALIZED VIEW! Should it really be public? JointVerticalAccessor and JointHorizontalAccessor are already good views, and JointAccessor is not a complete view.
		/** For usage with JointVerticalAccessor and JointHorizontalAccessor
		  */
		template<typename JA> class JointAccessor	{
		public:
			typedef typename JA::value_type value_type;
		private:
			JA &joint;
			value_type eps;
		public:
			inline JointAccessor(JA &j,value_type e=1e-7):joint(j),eps(e)	{}
			inline void sumRowMultiplied(size_t from,size_t to,size_t startIndex,value_type coef)	{
				for (size_t i=startIndex;i<joint.C1;++i) joint.getFirst(to,i)+=coef*joint.getFirst(from,i);
				for (size_t i=0;i<joint.C2;++i) joint.getSecond(to,i)+=coef*joint.getSecond(from,i);
			}
			//The following two methods are highly incorrect if the first matrix is actually
			//intended to end as an identity matrix. However, this is not necessary, since this
			//class is used to calculate a quotient matrix, and the "first" one should just be a
			//temporary copy of the divisor.
			//That is, unnecesary calculations to the first matrix are avoided.
			inline void substractRowAsNeeded(size_t from,size_t to)	{
				value_type coef=joint.getFirst(to,from)/joint.getFirst(from,from);
				if (std::abs(coef)>=eps) sumRowMultiplied(from,to,from+1,-coef);
			}
			inline void substractWhenReduced(size_t from,size_t to)	{
				value_type coef=joint.getFirst(to,from);
				if (std::abs(coef)>=eps) for (size_t i=0;i<joint.C2;++i) joint.getSecond(to,i)-=coef*joint.getSecond(from,i);
			}
			inline size_t size() const	{
				return joint.R;
			}
			void unitarizeReducedRow(size_t pos)	{
				value_type coef=joint.getFirst(pos,pos);
				joint.getFirst(pos,pos)=static_cast<value_type>(1);
				for (size_t i=pos+1;i<joint.C1;++i) joint.getFirst(pos,i)/=coef;
				for (size_t i=0;i<joint.C2;++i) joint.getSecond(pos,i)/=coef;
			}
			void ensureSuitablePos(size_t pos)	{
				if (std::abs(joint.getFirst(pos,pos))>=eps) return;
				for (size_t i=pos+1;i<joint.R;++i) if (std::abs(joint.getFirst(i,pos))>=eps)	{
					for (size_t j=pos;j<joint.C1;++j) joint.getFirst(pos,j)+=joint.getFirst(i,j);
					for (size_t j=0;j<joint.C2;++j) joint.getSecond(pos,j)+=joint.getSecond(i,j);
					return;
				}
				throw std::logic_error("ensureSuitablePos: Divisor is a singular matrix");
			}
			void ensureAndUnitarizeLast()	{
				size_t pos=joint.R-1;
				value_type coef=joint.getFirst(pos,pos);
				if (std::abs(coef)<eps) throw std::logic_error("ensureAndUnitarizeLast: Divisor is a singular matrix");
				joint.getFirst(pos,pos)=value_type(1);
				for (size_t i=0;i<joint.C2;++i) joint.getSecond(pos,i)/=coef;
			}
			void dumpToConsole() const {
				for (size_t i=0;i<joint.R;i++)	{
					for (size_t c=0;c<joint.C1;c++) printf("%15.03f ",joint.getFirst(i,c));
					for (size_t c=0;c<joint.C2;c++) printf("%15.03f ",joint.getSecond(i,c));
					printf("\n");
				}
			}
		};


		template<typename T> class IndirectAccessWrapper	{
		private:
			CMatrixTemplateNumeric<T> mat;	//NOT a reference, but a local copy.
			std::vector<size_t> rowsIndices;
			std::vector<size_t> colsIndices;
			class Generator	{
			private:
				size_t N;
				size_t i;
			public:
				inline Generator(size_t n):N(n),i(0)	{}
				inline size_t operator()()	{
					return i++;
				}
			};
		public:
			inline IndirectAccessWrapper(const CMatrixTemplateNumeric<T> &m):mat(m),rowsIndices(m.getRowCount()),colsIndices(m.getColCount())	{
				std::generate(rowsIndices.begin(),rowsIndices.end(),Generator(m.getRowCount()));
				std::generate(colsIndices.begin(),colsIndices.end(),Generator(m.getColCount()));
			}
			inline T &operator()(size_t r,size_t c)	{
				return mat.get_unsafe(rowsIndices[r],colsIndices[c]);
			}
			inline const T &operator()(size_t r,size_t c) const	{
				return mat.get_unsafe(rowsIndices[r],colsIndices[c]);
			}
			inline void deleteRow(size_t r)	{
				rowsIndices.erase(rowsIndices.begin()+ r);
			}
			inline void deleteColumn(size_t c)	{
				colsIndices.erase(colsIndices.begin()+ c);
			}
			inline size_t getRowCount() const	{
				return rowsIndices.size();
			}
			inline size_t getColCount() const	{
				return colsIndices.size();
			}
			inline size_t getProxyRow(size_t r)	{
				return rowsIndices[r];
			}
			inline T &getProxied(size_t proxiedR,size_t c)	{
				return mat.get_unsafe(proxiedR,colsIndices[c]);
			}
			inline const T &getProxied(size_t proxiedR,size_t c) const	{
				return mat.get_unsafe(proxiedR,colsIndices[c]);
			}
		};


	} // End of namespace
} // End of namespace

#endif

