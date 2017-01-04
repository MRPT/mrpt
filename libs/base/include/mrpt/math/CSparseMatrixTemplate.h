/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CSparseMatrixTemplate_H
#define CSparseMatrixTemplate_H

#include <mrpt/utils/utils_defs.h>
#include <map>

namespace mrpt	{
namespace math	{

    /** A sparse matrix container (with cells of any type), with iterators.
      *  This class stores only those elements created by assigning them a value, for example: "M(2,3)=8;".
      *
      *  This class doesn't implement math operations since it's a generic sparse container, but it can be
      *   used to initialize the contents of a CSparse library-based matrix of type mrpt::math::CSparseMatrix.
      *
      *  Note that reading non-existing cell elements will return the default value (0 for numbers)
      *   and that cell will remain non-created in the matrix.
      *
      *  There is an additional method "exists(i,j)" to check whether a given element exists in the matrix.
      *
      *  \sa mrpt::math::MatrixBlockSparseCols, mrpt::math::CSparseMatrix, CSparseSymmetricalMatrix
      *  \note Methods marked as "Doesn't check bounds" mean that if an access to an element out of the matrix size is tried, an empty element will be assumed, but this will not raise any invalid memory access.
      * \ingroup mrpt_base_grp
      */
	template<class T>
	class CSparseMatrixTemplate	{
		//Public typedefs
	public:
		/**
		  * Internal map type, used to store the actual matrix.
		  */
		typedef typename std::map<std::pair<size_t,size_t>,T> SparseMatrixMap;
		/**
		  * Const iterator to move through the matrix.
		  * \sa CSparseMatrixTemplate::const_reverse_iterator
		  */
		typedef typename SparseMatrixMap::const_iterator const_iterator;
		/**
		  * Const reverse iterator to move through the matrix.
		  * \sa CSparseMatrixTemplate::const_iterator
		  */
		typedef typename SparseMatrixMap::const_reverse_iterator const_reverse_iterator;
	protected:
		/**
		  * Size of the matrix.
		  */
		size_t mRows,mColumns;
		/**
		  * Actual matrix.
		  */
		SparseMatrixMap objectList;
	public:
		/**
		  * Basic constructor with no data. Size is set to (0,0).
		  */
		CSparseMatrixTemplate():mRows(0),mColumns(0)	{}
		/**
		  * Constructor with default size.
		  */
		CSparseMatrixTemplate(size_t nR,size_t nC):mRows(nR),mColumns(nC)	{}
		/**
		  * Element access operator. Doesn't check bounds.
		  */
		inline T operator()(size_t r,size_t c) const	{
			const_iterator it=objectList.find(std::make_pair(r,c));
			if (it==objectList.end()) return T();
			else return it->second;
		}

		/** Element access operator. Checks bounds.
		  */
		inline bool exists(size_t r,size_t c) const	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (r>=mRows||c>=mColumns) throw std::logic_error("Out of range");
#endif
			return (objectList.find(std::make_pair(r,c)) != objectList.end());
		}

		/**
		  * Reference access operator. Checks for bounds.
		  */
		inline T& operator()(size_t r,size_t c)	{ //-V659
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (r>=mRows||c>=mColumns) throw std::logic_error("Out of range");
#endif
			return objectList[std::make_pair(r,c)];
		}
		/**
		  * Returns the amount of rows in this matrix.
		  * \sa getColCount,getRow
		  */
		inline size_t getRowCount() const	{
			return mRows;
		}
		/**
		  * Returns the amount of columns in this matrix.
		  * \sa getRowCount
		  */
		inline size_t getColCount() const	{
			return mColumns;
		}
		/**
		  * Extracts a full row from the matrix.
		  * \sa getRowCount,getColumn,setRow
		  * \throw std::logic_error on out of range.
		  */
		void getRow(size_t nRow,std::vector<T> &vec) const	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (nRow>=mRows) throw std::logic_error("Out of range");
#endif
			vec.resize(mColumns);
			size_t nextIndex=0;
			for (typename SparseMatrixMap::const_iterator it=objectList.begin();it!=objectList.end();++it)	{
				const std::pair<size_t,size_t> &index=it->first;
				if (index.first<nRow) continue;
				else if (index.first==nRow)	{
					for (size_t i=nextIndex;i<index.second;i++) vec[i]=T();
					vec[index.second]=it->second;
					nextIndex=index.second+1;
				}	else	{
					for (size_t i=nextIndex;i<mColumns;i++) vec[i]=T();
					break;
				}
			}
		}
		/**
		  * Extracts a full column from the matrix.
		  * \sa getColCount,getRow,setColumn
		  * \throw std::logic_error on out of range.
		  */
		void getColumn(size_t nCol,std::vector<T> &vec) const	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (nCol>=mColumns) throw std::logic_error("Out of range");
#endif
			vec.resize(mRows);
			size_t nextIndex=0;
			for (typename SparseMatrixMap::const_iterator it=objectList.begin();it!=objectList.end();++it)	{
				const std::pair<size_t,size_t> &index=it->first;
				if (index.second==nCol)	{
					for (size_t i=nextIndex;i<index.first;i++) vec[i]=T();
					vec[index.first]=it->second;
					nextIndex=index.first+1;
				}
			}
			for (size_t i=nextIndex;i<mRows;i++) vec[i]=T();
		}
		/**
		  * Inserts an element into the matrix.
		  * \sa operator()(size_t,size_t)
		  */
		inline void insert(size_t row,size_t column,const T& obj)	{
			operator()(row,column)=obj;
		}

		/** Inserts submatrix at a given location */
		template <class MATRIX_LIKE>
		inline void insertMatrix(size_t row,size_t column,const MATRIX_LIKE& mat)
		{
			for (size_t nr=0;nr<mat.getRowCount();nr++)
				for (size_t nc=0;nc<mat.getColCount();nc++)
					operator()(row+nr,column+nc)=mat(nr,nc);
		}

		//Public interface only supports const iterators. This way, no user of this class will be able to freely modify it contents.
		/**
		  * Returns an iterator which points to the starting point of the matrix. It's a const_iterator, so that the usar isn't able to modify the matrix content into an invalid state.
		  * \sa end,rbegin,rend
		  */
		inline const_iterator begin() const	{
			return objectList.begin();
		}
		/**
		  * Returns an iterator which points to the end of the matrix. It's a const_iterator, so that the usar isn't able to modify the matrix content into an invalid state.
		  * \sa begin,rbegin,rend
		  */
		inline const_iterator end() const	{
			return objectList.end();
		}
		/**
		  * Returns an iterator which points to the end of the matrix, and can be used to move backwards. It's a const_reverse_iterator, so that the usar isn't able to modify the matrix content into an invalid state.
		  * \sa begin,end,rend
		  */
		inline const_reverse_iterator rbegin() const	{
			return objectList.rbegin();
		}
		/**
		  * Returns an iterator which points to the starting point of the matrix, although it's the upper limit of the matrix since it's a reverse iterator. Also, it's a const_reverse_iterator, so that the usar isn't able to modify the matrix content into an invalid state.
		  * \sa begin,end,rbegin
		  */
		inline const_reverse_iterator rend() const	{
			return objectList.rend();
		}
		/**
		  * Inserts a full row into the matrix. The third argument is used to specify a null object (which won't be inserted, since the matrix is sparse).
		  * \sa getRow
		  * \throw std::logic_error on out of range or wrong sized vector.
		  */
		void setRow(size_t nRow,const std::vector<T> &vec,const T& nullObject=T())	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (nRow>=mRows) throw std::logic_error("Out of range");
#endif
			size_t N=vec.size();
			if (N!=mColumns) throw std::logic_error("Wrong-sized vector");
			for (size_t i=0;i<N;i++)	{
				const T &obj=vec[i];
				std::pair<size_t,size_t> index=std::make_pair(nRow,i);
				if (obj==nullObject) objectList.erase(index);
				else objectList[index]=obj;
			}
		}
		/**
		  * Inserts a full column into the matrix. The third argument is used to specify a null object (which won't be inserted, since the matrix is sparse).
		  * \sa getColumn
		  * \throw std::logic_error on out of range or wrong sized vector.
		  */
		void setColumn(size_t nCol,const std::vector<T> &vec,const T& nullObject=T())	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (nCol>=mColumns) throw std::logic_error("Out of range");
#endif
			size_t N=vec.size();
			if (N!=mRows) throw std::logic_error("Wrong-sized vector");
			for (size_t i=0;i<N;i++)	{
				const T &obj=vec[i];
				std::pair<size_t,size_t> index=std::make_pair(i,nCol);
				if (obj==nullObject) objectList.erase(index);
				else objectList[index]=obj;
			}
		}
		/**
		  * Changes the size of the matrix.
		  */
		void resize(size_t nRows,size_t nCols)	{
			// if (mRows<0||mColumns<0) throw std::logic_error("Invalid range"); // This case never happens!
			if (mRows==nRows && mColumns==nCols) return;
			mRows=nRows;
			mColumns=nCols;
			std::vector<std::pair<size_t,size_t> > toErase;
			for (const_iterator it=objectList.begin();it!=objectList.end();++it)	{
				const std::pair<size_t,size_t> &i=it->first;
				if (i.first>=nRows||i.second>=nCols) toErase.push_back(it->first);
			}
			for (std::vector<std::pair<size_t,size_t> >::const_iterator it=toErase.begin();it!=toErase.end();++it) objectList.erase(*it);
		}
		/**
		  * Extracts a submatrix form the matrix.
		  * \sa operator()(size_t,size_t)
		  * \throw std::logic_error on invalid bounds.
		  */
		CSparseMatrixTemplate<T> operator()(size_t firstRow,size_t lastRow,size_t firstColumn,size_t lastColumn) const	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (lastRow>=mRows||lastColumn>=mColumns) throw std::logic_error("Out of range");
			if (firstRow>lastRow||firstColumn>lastColumn) throw std::logic_error("Invalid size");
#endif
			CSparseMatrixTemplate<T> res=CSparseMatrixTemplate<T>(lastRow+1-firstRow,lastColumn+1-firstColumn);
			for (typename SparseMatrixMap::const_iterator it=begin();it!=end();++it)	{
				const std::pair<size_t,size_t> &i=it->first;
				if (i.first>=firstRow&&i.first<=lastRow&&i.second>=firstColumn&&i.second<=lastColumn) res(i.first-firstRow,i.second-firstColumn)=it->second;
			}
			return res;
		}
		/**
		  * Gets a vector containing all the elements of the matrix, ignoring their position.
		  */
		void getAsVector(std::vector<T> &vec) const	{
			size_t N=objectList.size();
			vec.resize(0);
			vec.reserve(N);
			for (const_iterator it=objectList.begin();it!=objectList.end();++it) vec.push_back(it->second);
		}
		/**
		  * Gets the amount of non-null elements inside the matrix.
		  * \sa getNullElements,isNull,isNotNull
		  */
		inline size_t getNonNullElements() const	{
			return objectList.size();
		}
		/** Are there no elements set to !=0 ?
		  * \sa getNullElements,isNull,isNotNull
		  */
		inline bool empty() const	{ return objectList.empty(); }

		/**
		  * Gets the amount of null elements inside the matrix.
		  * \sa getNonNullElements,isNull,isNotNull
		  */
		inline size_t getNullElements() const	{
			return mRows*mColumns-getNonNullElements();
		}
		/**
		  * Checks whether an element of the matrix is the default object.
		  * \sa getNonNullElements,getNullElements,isNotNull
		  * \throw std::logic_error on out of range
		  */
		inline bool isNull(size_t nRow,size_t nCol) const	{
			if (nRow>=mRows||nCol>=mColumns) throw std::logic_error("Out of range");
			return objectList.count(std::make_pair(nRow,nCol))==0;
		}
		/**
		  * Checks whether an element of the matrix is not the default object.
		  * \sa getNonNullElements,getNullElements,isNull
		  */
		inline bool isNotNull(size_t nRow,size_t nCol) const	{
			if (nRow>=mRows||nCol>=mColumns) throw std::logic_error("Out of range");
			return objectList.count(std::make_pair(nRow,nCol))>0;
		}
		/**
		  * Completely removes all elements, although maintaining the matrix's size.
		  */
		inline void clear()	{
			objectList.clear();
		}
		/**
		  * Checks each non-null elements against the basic objects, erasing unnecesary references to it.
		  */
		void purge(T nullObject=T())	{
			std::vector<std::pair<size_t,size_t> > nulls;
			for (const_iterator it=begin();it!=end();++it) if (it->second==nullObject) nulls.push_back(it->first);
			for (std::vector<std::pair<size_t,size_t> >::const_iterator it=nulls.begin();it!=nulls.end();++it) objectList.erase(*it);
		}
	}; // end of sparse matrix

    /** A sparse matrix container for square symmetrical content around the main diagonal.
      *  This class saves half of the space with respect to CSparseMatrixTemplate since only those entries (c,r) such as c>=r are really stored,
      *   but both (c,r) and (r,c) can be retrieved or set and both redirect to the same internal cell container.
      *  \sa CSparseMatrixTemplate
      */
	template<class T>
	class CSparseSymmetricalMatrix : public CSparseMatrixTemplate<T> {
		public:
		CSparseSymmetricalMatrix() : CSparseMatrixTemplate<T>() { }
		explicit CSparseSymmetricalMatrix(const CSparseSymmetricalMatrix &o) : CSparseMatrixTemplate<T>(o) { }
		explicit CSparseSymmetricalMatrix(const CSparseMatrixTemplate<T> &o) : CSparseMatrixTemplate<T>(o) { }
		virtual ~CSparseSymmetricalMatrix() { }

		void resize(size_t matrixSize) {
			CSparseMatrixTemplate<T>::resize(matrixSize,matrixSize);
		}

		inline T operator()(size_t r,size_t c) const	{
			if (c<r) std::swap(r,c); // Symmetrical matrix
			typename CSparseMatrixTemplate<T>::const_iterator it=CSparseMatrixTemplate<T>::objectList.find(std::make_pair(r,c));
			if (it==CSparseMatrixTemplate<T>::objectList.end()) return T();
			else return it->second;
		}
		inline T& operator()(size_t r,size_t c)	{ //-V659
			if (c<r) std::swap(r,c); // Symmetrical matrix
			if (r>=CSparseMatrixTemplate<T>::mRows||c>=CSparseMatrixTemplate<T>::mColumns) throw std::logic_error("Out of range");
			return CSparseMatrixTemplate<T>::objectList[std::make_pair(r,c)];
		}

	}; // end of CSparseSymmetricalMatrix

}
}
#endif
