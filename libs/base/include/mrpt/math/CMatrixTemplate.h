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
#ifndef CMatrixTemplate_H
#define CMatrixTemplate_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/memory.h>
#include <mrpt/system/datetime.h>

#include <mrpt/math/math_frwds.h>  // Fordward declarations
#include <mrpt/math/CArray.h>

namespace mrpt
{
namespace math
{
	// A static cast with a specialization for bool:
	template <typename U> U myStaticCast(double val) { return static_cast<U>(val); }
	template <> bool myStaticCast(double val);


/**  This template class provides the basic functionality for a general 2D any-size, resizable container of numerical or non-numerical elements.
 * NOTES:
 *		- This class is not serializable since it is a template. Use or declare derived classes for using serialization.
 *		- First row or column index is "0".
 *		- This class includes range checks with ASSERT_() if compiling with "_DEBUG" or "MRPT_ALWAYS_CHECKS_DEBUG_MATRICES=1".
 *		- Please DO NOT use as template class type any other class. It can be safely used the following types:
 *			- Elemental types (int,char,float,doble,...)
 *			- Data struct (Not classes!)
 *			- Any kind of pointers (user is responsible for allocating and freeing the memory addressed by pointers).
 *
 * \note Memory blocks for each row are 16-bytes aligned (since MRPT 0.7.0).
 * \sa CMatrixTemplateNumeric
 */
template <class T>
class CMatrixTemplate
{
public:
	// type definitions
	typedef T              value_type;		//!< The type of the matrix elements
	typedef T&             reference;
	typedef const T&       const_reference;
	typedef std::size_t    size_type;
	typedef std::ptrdiff_t difference_type;


protected:
	T				**m_Val;
	size_t			m_Rows, m_Cols;

	/** Internal use only: It reallocs the memory for the 2D matrix, maintaining the previous contents if posible.
	  */
    void realloc(size_t row, size_t col, bool newElementsToZero = false)
	{
		if (row!=m_Rows || col!=m_Cols || m_Val==NULL)
		{
			size_t	r;
			bool    doZeroColumns   = newElementsToZero && (col>m_Cols);
			size_t	sizeZeroColumns = sizeof(T)*(col-m_Cols);

			// If we are reducing rows, free that memory:
            for (r=row;r<m_Rows;r++)
				mrpt::system::os::aligned_free( m_Val[r] );

			// Realloc the vector of pointers:
			if (!row)
					{ mrpt::system::os::aligned_free(m_Val); m_Val=NULL; }
			else	m_Val = static_cast<T**> (mrpt::system::os::aligned_realloc(m_Val, sizeof(T*) * row, 16 ) );

			// How many new rows/cols?
			size_t	row_size = col * sizeof(T);

			// Alloc new ROW pointers & resize previously existing rows, as required:
			for (r=0;r<row;r++)
			{
				if (r<m_Rows)
				{
					// This was an existing row: Resize the memory:
					m_Val[r] = static_cast<T*> (mrpt::system::os::aligned_realloc( m_Val[r], row_size, 16));

					if (doZeroColumns)
					{
						// Fill with zeros:
						::memset(&m_Val[r][m_Cols],0,sizeZeroColumns);
					}
				}
				else
				{
					// This is a new row, alloc the memory for the first time:
					m_Val[r] = static_cast<T*> ( mrpt::system::os::aligned_calloc( row_size, 16 ));
				}
			}

			// Done!
			m_Rows	= row;
			m_Cols	= col;
		}
	}


public:
	/**
	  * Checks whether the rows [r-N,r+N] and the columns [c-N,c+N] are present in the matrix.
	  */
	template<size_t N> inline void ASSERT_ENOUGHROOM(size_t r,size_t c) const	{
		#if defined(_DEBUG)||(MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			ASSERT_((r>=N)&&(r+N<getRowCount())&&(c>=N)&&(c+N<getColCount()));
		#endif
	}
	/*! Fill all the elements with a given value (Note: named "fillAll" since "fill" will be used by child classes) */
	void fillAll(const T &val) {
		for (size_t r=0;r<m_Rows;r++)
			for (size_t c=0;c<m_Cols;c++)
				m_Val[r][c]=val;
	}

	/** Swap with another matrix very efficiently (just swaps a pointer and two integer values). */
	inline void  swap(CMatrixTemplate<T> &o)
	{
		std::swap(m_Val,  o.m_Val  );
		std::swap(m_Rows, o.m_Rows );
		std::swap(m_Cols, o.m_Cols );
	}

	/** Returns a given column to a vector (without modifying the matrix)
		* \exception std::exception On index out of bounds
		*/
	void  extractCol(size_t nCol, std::vector<T> &out, int startingRow = 0) const
	{
		size_t		i,n;
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (nCol>=m_Cols)
			THROW_EXCEPTION("extractCol: Column index out of bounds");
#endif

		n = m_Rows - startingRow;
		out.resize( n );

		for (i=0;i<n;i++)
			out[i] = m_Val[i+startingRow][nCol];
	}

	/** Gets a given column to a vector (without modifying the matrix)
		* \exception std::exception On index out of bounds
		*/
	void  extractCol(size_t nCol, CMatrixTemplate<T> &out, int startingRow = 0) const
	{
		size_t		i,n;
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (nCol>=m_Cols)
			THROW_EXCEPTION("extractCol: Column index out of bounds");
#endif

		n = m_Rows - startingRow;
		out.setSize(n,1);

		for (i=0;i<n;i++)
			out(i,0) = m_Val[i+startingRow][nCol];
	}

	/** Swap two columns
		* \exception std::exception On index out of bounds
		*/
	void  swapCols(size_t nCol1, size_t nCol2)
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (nCol1>=m_Cols || nCol2>=m_Cols)
			THROW_EXCEPTION("swapCols: Column index out of bounds");
#endif
		const size_t n = m_Rows;
		for (size_t i=0;i<n;i++)
			std::swap( m_Val[i][nCol1], m_Val[i][nCol2] );
	}

	/** Swap two rows
		* \exception std::exception On index out of bounds
		*/
	inline void  swapRows(size_t nRow1, size_t nRow2)
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (nRow1>=m_Rows || nRow2>=m_Rows)
			THROW_EXCEPTION("swapCols: Row index out of bounds");
#endif
		std::swap(CMatrixTemplate<T>::m_Val[nRow1], CMatrixTemplate<T>::m_Val[nRow2] );
	}

	/** Gets a given row to a vector (without modifying the matrix)
		* \exception std::exception On index out of bounds
		*/
	template <class F>
	void  extractRow(size_t nRow, std::vector<F> &out, size_t startingCol = 0) const
	{
		size_t		i,n;
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (nRow>=m_Rows)
			THROW_EXCEPTION("extractRow: Row index out of bounds");
#endif
		n = m_Cols - startingCol ;
		out.resize( n );

		for (i=0;i<n;i++)
			out[i] = static_cast<F> ( m_Val[nRow][i+startingCol] );
	}

	/** Gets a given row to a vector (without modifying the matrix)
		* \exception std::exception On index out of bounds
		*/
	template <class F>
	void  extractRow(size_t nRow, CMatrixTemplate<F> &out, size_t startingCol = 0) const
	{
		size_t		i,n;
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (nRow>=m_Rows)
			THROW_EXCEPTION("extractRow: Row index out of bounds");
#endif
		n = m_Cols - startingCol ;
		out.setSize(1,n);

		for (i=0;i<n;i++)
			out(0,i) = static_cast<F>( m_Val[nRow][i+startingCol] );
	}

	/** Constructors
	*/
	CMatrixTemplate (const CMatrixTemplate& m) : m_Val(NULL),m_Rows(0),m_Cols(0)
	{
		(*this) = m;
	}

	CMatrixTemplate (size_t row = 3, size_t col = 3) :  m_Val(NULL),m_Rows(0),m_Cols(0)
	{
		realloc(row,col);
	}

	/** Copy constructor & crop from another matrix
	*/
	CMatrixTemplate (const CMatrixTemplate& m, const size_t cropRowCount, const size_t cropColCount) : m_Val(NULL),m_Rows(0),m_Cols(0)
	{
		ASSERT_(m.m_Rows>=cropRowCount)
		ASSERT_(m.m_Cols>=cropColCount)

		realloc( cropRowCount, cropColCount );

		for (size_t i=0; i < m_Rows; i++)
			for (size_t j=0; j < m_Cols; j++)
				m_Val[i][j] = m.m_Val[i][j];
	}

	/** Constructor from a given size and a C array. The array length must match cols x row.
	  * \code
	  *  const double numbers[] = {
	  *    1,2,3,
	  *    4,5,6 };
	  *	 CMatrixDouble   M(3,2, numbers);
	  * \endcode
	  */
	template <typename V, size_t N>
	CMatrixTemplate (size_t row, size_t col, V (&theArray)[N] ) :  m_Val(NULL),m_Rows(0),m_Cols(0)
	{
		MRPT_COMPILE_TIME_ASSERT(N!=0)
		realloc(row,col);
		if (m_Rows*m_Cols != N) THROW_EXCEPTION(format("Mismatch between matrix size %lu x %lu and array of length %lu",static_cast<long unsigned>(m_Rows),static_cast<long unsigned>(m_Cols),static_cast<long unsigned>(N)))
		size_t	idx=0;
		for (size_t i=0; i < m_Rows; i++)
			for (size_t j=0; j < m_Cols; j++)
				m_Val[i][j] = static_cast<T>(theArray[idx++]);

	}

	/** Constructor from a given size and a STL container (std::vector, std::list,...) with the initial values. The vector length must match cols x row.
	*/
	template <typename V>
	CMatrixTemplate(size_t row, size_t col, const V &theVector ) :  m_Val(NULL),m_Rows(0),m_Cols(0)
	{
		const size_t N = theVector.size();
		realloc(row,col);
		if (m_Rows*m_Cols != N)  THROW_EXCEPTION(format("Mismatch between matrix size %lu x %lu and array of length %lu",static_cast<long unsigned>(m_Rows),static_cast<long unsigned>(m_Cols),static_cast<long unsigned>(N)))

		typename V::const_iterator it = theVector.begin();
		for (size_t i=0; i < m_Rows; i++)
			for (size_t j=0; j < m_Cols; j++)
				m_Val[i][j] = static_cast<T>( *(it++) );

	}

	/** Destructor
	*/
	virtual ~CMatrixTemplate()
	{
		realloc(0,0);
	}

	/** Assignment operator from another matrix
	  */
	CMatrixTemplate& operator = (const CMatrixTemplate& m)
	{
		realloc( m.m_Rows, m.m_Cols );

		for (size_t i=0; i < m_Rows; i++)
			for (size_t j=0; j < m_Cols; j++)
				m_Val[i][j] = m.m_Val[i][j];

		return *this;
	}

	/** Assignment operator for initializing from a C array (The matrix must be set to the correct size before invoking this asignament)
	  * \code
	  *	 CMatrixDouble   M(3,2);
	  *  const double numbers[] = {
	  *    1,2,3,
	  *    4,5,6 };
	  *  M = numbers;
	  * \endcode
	  *  Refer also to the constructor with initialization data CMatrixTemplate::CMatrixTemplate
	  */
	template <typename V, size_t N>
	CMatrixTemplate& operator = (V (&theArray)[N] )
	{
		MRPT_COMPILE_TIME_ASSERT(N!=0)

		if (m_Rows*m_Cols != N)
		{
			THROW_EXCEPTION(format("Mismatch between matrix size %lu x %lu and array of length %lu",m_Rows,m_Cols,N))
		}
		size_t	idx=0;
		for (size_t i=0; i < m_Rows; i++)
			for (size_t j=0; j < m_Cols; j++)
				m_Val[i][j] = static_cast<T>(theArray[idx++]);

		return *this;
	}


	/** Number of rows in the matrix
	  * \sa getRowCount, getColCount, nr, nc
	  */
	inline size_t getRowCount() const { return m_Rows; }

	/** Number of columns in the matrix
	  * \sa getRowCount, getColCount, nr, nc
	 */
	inline size_t getColCount() const { return m_Cols; }

	/** Get a 2-vector with [NROWS NCOLS] (as in MATLAB command size(x)) */
	inline CMatrixTemplateSize size() const
	{
		CMatrixTemplateSize dims;
		dims[0]=m_Rows;
		dims[1]=m_Cols;
		return dims;
	}

	/** Changes the size of matrix, maintaining the previous contents. */
	void setSize(size_t row, size_t col,bool zeroNewElements=false)
	{
		realloc(row,col,zeroNewElements);
	}

	/** This method just checks has no effects in this class, but raises an exception if the expected size does not match */
	inline void resize(const CMatrixTemplateSize &siz,bool zeroNewElements=false)
	{
		setSize(siz[0],siz[1],zeroNewElements);
	}

	/** Subscript operator to get/set individual elements
		*/
	inline T& operator () (size_t row, size_t col)
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (row >= m_Rows || col >= m_Cols)
			THROW_EXCEPTION( format("Indexes (%lu,%lu) out of range. Matrix is %lux%lu",static_cast<unsigned long>(row),static_cast<unsigned long>(col),static_cast<unsigned long>(m_Rows),static_cast<unsigned long>(m_Cols)) );
#endif
		return m_Val[row][col];
	}

	/** Subscript operator to get individual elements
		*/
	inline const T &operator () (size_t row, size_t col) const
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (row >= m_Rows || col >= m_Cols)
			THROW_EXCEPTION( format("Indexes (%lu,%lu) out of range. Matrix is %lux%lu",static_cast<unsigned long>(row),static_cast<unsigned long>(col),static_cast<unsigned long>(m_Rows),static_cast<unsigned long>(m_Cols)) );
#endif
		return m_Val[row][col];
	}

	/** Subscript operator to get/set an individual element from a row or column matrix.
	  * \exception std::exception If the object is not a column or row matrix.
	  */
	inline T& operator () (size_t ith)
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		ASSERT_(m_Rows==1 || m_Cols==1);
#endif
		if (m_Rows==1)
		{
			// A row matrix:
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (ith >= m_Cols)
				THROW_EXCEPTION_CUSTOM_MSG1( "Index %u out of range!",static_cast<unsigned>(ith) );
#endif
			return m_Val[0][ith];
		}
		else
		{
			// A columns matrix:
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (ith >= m_Rows)
				THROW_EXCEPTION_CUSTOM_MSG1( "Index %u out of range!",static_cast<unsigned>(ith) );
#endif
			return m_Val[ith][0];
		}
	}

	/** Subscript operator to get/set an individual element from a row or column matrix.
	  * \exception std::exception If the object is not a column or row matrix.
	  */
	inline T operator () (size_t ith) const
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		ASSERT_(m_Rows==1 || m_Cols==1);
#endif
		if (m_Rows==1)
		{
			// A row matrix:
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (ith >= m_Cols)
				THROW_EXCEPTION_CUSTOM_MSG1( "Index %u out of range!",static_cast<unsigned>(ith) );
#endif
			return m_Val[0][ith];
		}
		else
		{
			// A columns matrix:
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (ith >= m_Rows)
				THROW_EXCEPTION_CUSTOM_MSG1( "Index %u out of range!",static_cast<unsigned>(ith) );
#endif
			return m_Val[ith][0];
		}
	}

	/** Fast but unsafe method to write a value in the matrix
		*/
	inline void set_unsafe(size_t row, size_t col,const T &v)
	{
#ifdef _DEBUG
		if (row >= m_Rows || col >= m_Cols)
			THROW_EXCEPTION( format("Indexes (%lu,%lu) out of range. Matrix is %lux%lu",static_cast<unsigned long>(row),static_cast<unsigned long>(col),static_cast<unsigned long>(m_Rows),static_cast<unsigned long>(m_Cols)) );
#endif
		m_Val[row][col] = v;
	}

	/** Fast but unsafe method to read a value from the matrix
		*/
	inline const T &get_unsafe(size_t row, size_t col) const
	{
#ifdef _DEBUG
		if (row >= m_Rows || col >= m_Cols)
			THROW_EXCEPTION( format("Indexes (%lu,%lu) out of range. Matrix is %lux%lu",static_cast<unsigned long>(row),static_cast<unsigned long>(col),static_cast<unsigned long>(m_Rows),static_cast<unsigned long>(m_Cols)) );
#endif
		return m_Val[row][col];
	}

	/** Fast but unsafe method to get a reference from the matrix
		*/
	inline T &get_unsafe(size_t row,size_t col)
	{
#ifdef _DEBUG
		if (row >= m_Rows || col >= m_Cols)
			THROW_EXCEPTION( format("Indexes (%lu,%lu) out of range. Matrix is %lux%lu",static_cast<unsigned long>(row),static_cast<unsigned long>(col),static_cast<unsigned long>(m_Rows),static_cast<unsigned long>(m_Cols)) );
#endif
		return m_Val[row][col];
	}

	/** Fast but unsafe method to obtain a pointer to a given row of the matrix (Use only in time critical applications)
		*/
	inline T* get_unsafe_row(size_t row)
	{
#ifdef _DEBUG
		if (row >= m_Rows)
			THROW_EXCEPTION( format("Row index %"PRIuPTR" out of range. Matrix is %"PRIuPTR"x%"PRIuPTR,static_cast<unsigned long>(row),static_cast<unsigned long>(m_Rows),static_cast<unsigned long>(m_Cols)) );
#endif
		return m_Val[row];
	}

	/** Fast but unsafe method to obtain a pointer to a given row of the matrix (Use only in critical applications)
		*/
	inline const T* get_unsafe_row(size_t row) const	{
		return m_Val[row];
	}

	/** Gets a series of contiguous rows.
		* \exception std::logic_error On index out of bounds
		* \sa extractRow
		* \sa extractColumns
		*/
	inline void extractRows(size_t firstRow,size_t lastRow,CMatrixTemplate<T> &out) const	{
		out.setSize(lastRow-firstRow+1,m_Cols);
		detail::extractMatrix(*this,firstRow,0,out);
	}

	/** Gets a series of contiguous columns.
		* \exception std::logic_error On index out of bounds
		* \sa extractColumn
		* \sa extractRows
		*/
	inline void extractColumns(size_t firstCol,size_t lastCol,CMatrixTemplate<T> &out) const	{
		out.setSize(m_Rows,lastCol-firstCol+1);
		detail::extractMatrix(*this,0,firstCol,out);
	}

	/** Inserts a row from a vector, replacing the current contents of that row.
		* \exception std::exception On index out of bounds
		* \sa extractRow
		*/
	void  insertRow(size_t nRow, const std::vector<T> &in)
	{
		if (nRow>=m_Rows) THROW_EXCEPTION("insertRow: Row index out of bounds");

		size_t n = in.size();
		ASSERT_(m_Cols>=in.size());

		for (size_t i=0;i<n;i++)
			m_Val[nRow][i] = in[i];
	}

	/** Appends a new row to the MxN matrix from a 1xN vector.
	    *  The lenght of the vector must match the width of the matrix, unless it's empty: in that case the matrix is resized to 1xN.
	    *  \code
	    *    CMatrixDouble  M(0,0);
	    *    vector_double  v(7),w(7);
	    *    // ...
	    *    M.appendRow(v);
	    *    M.appendRow(w);
	    *  \endcode
		* \exception std::exception On incorrect vector length.
		* \sa extractRow
		* \sa appendCol
		*/
	void  appendRow(const std::vector<T> &in)
	{
		size_t		i,n, row;

		n = m_Cols;
		row = m_Rows;

		if (m_Cols==0 || m_Rows==0)
		{
			ASSERT_(!in.empty());
			n=m_Cols=in.size();
		}
		else
		{
			ASSERT_(in.size()==m_Cols);
		}

		realloc( row+1,n );

		for (i=0;i<n;i++)
			m_Val[row][i] = in[i];
	}

	/** Appends a new column to the matrix from a vector.
		* The length of the vector must match the number of rows of the matrix, unless it is (0,0).
		* \exception std::exception On size mismatch.
		* \sa extractCol
		* \sa appendRow
		*/
	void appendCol(const std::vector<T> &in)	{
		size_t r=m_Rows,c=m_Cols;
		if (m_Cols==0||m_Rows==0)	{
			ASSERT_(!in.empty());
			r=in.size();
			c=0;
		}	else ASSERT_(in.size()==m_Rows);
		realloc(r,c+1);
		for (size_t i=0;i<m_Rows;i++) m_Val[i][m_Cols-1]=in[i];
	}

	/** Inserts a column from a vector, replacing the current contents of that column.
		* \exception std::exception On index out of bounds
		* \sa extractCol
		*/
	void  insertCol(size_t nCol, const std::vector<T> &in)
	{
		if (nCol>=m_Cols) THROW_EXCEPTION("insertCol: Row index out of bounds");

		size_t n = in.size();
		ASSERT_( m_Rows >= in.size() );

		for (size_t i=0;i<n;i++)
			m_Val[i][nCol] = in[i];
	}

	/** Inserts a matrix into this matrix
	    *  Notice that the matrix must "fit" into the existing size of this matrix.
	    * \param in The submatrix to insert into 'this' matrix.
	    * \param nRow The row in 'this' matrix where the submatrix will be inserted (0:first).
	    * \param nCol The column in 'this' matrix where the submatrix will be inserted (0:first).
		* \exception std::exception On index out of bounds
		* \sa extractCol
		*/
	template <class MAT_R>
	void  insertMatrix(const size_t nRow,const  size_t nCol, const MAT_R &in)
	{
		const size_t nrows = in.getRowCount();
		const size_t ncols = in.getColCount();
		if ( (nRow+nrows > m_Rows) || (nCol+ncols >m_Cols) )
			THROW_EXCEPTION("insertMatrix: Row or Col index out of bounds");
		for (size_t i=nRow;i<nRow+nrows;i++)
			for(size_t j=nCol;j<nCol+ncols;j++)
				set_unsafe(i,j, static_cast<typename MAT_R::value_type> (in.get_unsafe(i-nRow,j-nCol) ) );
	}

	/** Inserts the transpose of a given matrix into this matrix
	    *  Notice that the matrix must "fit" into the existing size of this matrix.
	    * \param in The submatrix to insert into 'this' matrix.
	    * \param nRow The row in 'this' matrix where the submatrix will be inserted (0:first).
	    * \param nCol The column in 'this' matrix where the submatrix will be inserted (0:first).
		* \exception std::exception On index out of bounds
		* \sa extractCol
		*/
	template <class MAT_R>
	void  insertMatrixTranspose(const size_t nRow,const  size_t nCol, const MAT_R &in)
	{
		const size_t ncols = in.getRowCount(); // Transpose!
		const size_t nrows = in.getColCount();
		if ( (nRow+nrows > m_Rows) || (nCol+ncols >m_Cols) )
			THROW_EXCEPTION("insertMatrix: Row or Col index out of bounds");
		for (size_t i=nRow;i<nRow+nrows;i++)
			for(size_t j=nCol;j<nCol+ncols;j++)
				set_unsafe(i,j, static_cast<typename MAT_R::value_type> ( in.get_unsafe(j-nCol,i-nRow) ) );
	}

	/** Inserts a matrix line (vector) into this matrix
	    *  Notice that the matrix must "fit" into the existing size of this matrix.
		* \exception std::exception On index out of bounds
		* \sa extractCol
		* By AJOGD @ MAR-2007
		*/
	void  insertMatrix(size_t nRow, size_t nCol, const std::vector<T> &in)
	{
		size_t	j,ncols;

		ncols = in.size();
		if ( (nRow+1 > m_Rows) || (nCol+ncols >m_Cols) )
			THROW_EXCEPTION("insertMatrix: Row or Col index out of bounds");

		for(j = nCol ; j < nCol + ncols ; j++)
			set_unsafe(nRow,j, in[j-nCol] );
	}

	/** Inserts 4 matrixes corresponding to the "four corners" into this matrix.
		* \exception std::exception On index out of bounds
		* \sa insertMatrix
		*/
	void  joinMatrix(const CMatrixTemplate<T> &left_up,		const CMatrixTemplate<T> &right_up,
								const CMatrixTemplate<T> &left_down,	const CMatrixTemplate<T> &right_down)
	{
		if ((left_up.getRowCount()!= right_up.getRowCount())||(left_up.getColCount()!=left_down.getColCount())||
			(left_down.getRowCount()!=right_down.getRowCount())||(right_up.getColCount()!=right_down.getColCount()))
			THROW_EXCEPTION("join_Matrix: Row or Col index out of bounds");
		setSize(left_up.getRowCount()+left_down.getRowCount(),left_up.getColCount()+right_up.getColCount());
		insertMatrix(0,0,left_up);
        insertMatrix(0,left_up.getColCount(),right_up);
        insertMatrix(left_up.getRowCount(),0,left_down);
        insertMatrix(left_up.getRowCount(),left_up.getColCount(),right_down);
	}

	/** Get a submatrix, given its bounds
	  * \sa extractSubmatrixSymmetricalBlocks
	  */
	void extractSubmatrix(const size_t row1,const size_t row2,const size_t col1,const size_t col2,CMatrixTemplate<T> &out) const	{
		size_t nrows=row2-row1+1;
		size_t ncols=col2-col1+1;
		if (nrows<=0||ncols<=0)	{
			out.realloc(0,0);
			return;
		}
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (row1<0||row2>=m_Rows||col1<0||col2>=m_Cols) THROW_EXCEPTION("Indices out of range!");
#endif
		out.realloc(nrows,ncols);
		for (size_t i=0;i<nrows;i++) for (size_t j=0;j<ncols;j++) out.m_Val[i][j]=m_Val[i+row1][j+col1];
	}

	/** Subscript operator to get a submatrix
	  */
	inline CMatrixTemplate<T> operator() (const size_t row1,const size_t row2,const size_t col1,const size_t col2) const	{
		CMatrixTemplate<T> val(0,0);
		extractSubmatrix(row1,row2,col1,col2,val);
		return val;
	}

	/** Get a submatrix from a square matrix, by collecting the elements M(idxs,idxs), where idxs is a sequence {block_indices(i):block_indices(i)+block_size-1} for all "i" up to the size of block_indices.
	  *  A perfect application of this method is in extracting covariance matrices of a subset of variables from the full covariance matrix.
	  * \sa extractSubmatrix, extractSubmatrixSymmetrical
	  */
	void extractSubmatrixSymmetricalBlocks(
		const size_t 			block_size,
		const vector_size_t  	&block_indices,
		CMatrixTemplate<T> 		&out) const
	{
		ASSERT_(block_size>=1);
		ASSERT_(m_Cols==m_Rows);

		const size_t N = block_indices.size();
		size_t nrows_out=N*block_size;
		out.realloc(nrows_out,nrows_out);
		if (!N) return; // Done

		for (size_t dst_row_blk=0;dst_row_blk<N; ++dst_row_blk )
		{
#if defined(_DEBUG)||(MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (block_indices[dst_row_blk]*block_size + block_size-1>=m_Cols) THROW_EXCEPTION("Indices out of range!");
#endif

			for (size_t r=0;r<block_size;r++)
			{
				const T* src_row = this->m_Val[ block_indices[dst_row_blk] * block_size + r ];
				T* dst = &out.m_Val[ block_size*dst_row_blk + r ][0];
				for (size_t dst_col_blk=0;dst_col_blk<N; ++dst_col_blk ) // Copy one complete row "src_row"
				{
					const T* src = src_row +  block_indices[dst_col_blk] * block_size;
					for (size_t c=0;c<block_size;c++)
						*dst++ = *src++;
				}
			}
		}
	}

	/** Get a submatrix from a square matrix, by collecting the elements M(idxs,idxs), where idxs is the sequence of indices passed as argument.
	  *  A perfect application of this method is in extracting covariance matrices of a subset of variables from the full covariance matrix.
	  * \sa extractSubmatrix, extractSubmatrixSymmetricalBlocks
	  */
	void extractSubmatrixSymmetrical(
		const vector_size_t  	&indices,
		CMatrixTemplate<T> 		&out) const
	{
		ASSERT_(m_Cols==m_Rows);

		const size_t N = indices.size();
		const size_t nrows_out=N;
		out.realloc(nrows_out,nrows_out);
		if (!N) return; // Done

		for (size_t dst_row_blk=0;dst_row_blk<N; ++dst_row_blk )
		{
#if defined(_DEBUG)||(MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (indices[dst_row_blk]>=m_Cols) THROW_EXCEPTION("Indices out of range!");
#endif
			const T* src_row = this->m_Val[ indices[dst_row_blk] ];
			T* dst = &out.m_Val[ dst_row_blk ][0];
			for (size_t dst_col_blk=0;dst_col_blk<N; ++dst_col_blk ) // Copy one complete row "src_row"
			{
				const T* src = src_row +  indices[dst_col_blk];
				*dst++ = *src++;
			}
		}
	}


	/** Interchanges two columns of the matrix.
	  */
	void exchangeColumns(size_t col1,size_t col2)	{
#if defined(_DEBUG)||(MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (col1<0||col2<0||col1>=m_Cols||col2>=m_Cols) THROW_EXCEPTION("Indices out of range!");
#endif
		T tmp;
		for (size_t i=0;i<m_Rows;i++)	{
			tmp=m_Val[i][col1];
			m_Val[i][col1]=m_Val[i][col2];
			m_Val[i][col2]=tmp;
		}
	}

	/** Interchanges two rows of the matrix. */
	inline void exchangeRows(size_t row1,size_t row2) { swapRows(row1,row2); }

	/** Deletes a row of the matrix.
	  */
	void deleteRow(size_t row)	{
#if defined(_DEBUG)||(MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (row<0||row>=m_Rows) THROW_EXCEPTION("Index out of range!");
#endif
		for (size_t i=row;i<m_Rows-1;i++) for (size_t j=0;j<m_Cols;j++) m_Val[i][j]=m_Val[i+1][j];
		realloc(m_Rows-1,m_Cols);
	}

	/** Deletes a column of the matrix.
	  */
	void deleteColumn(size_t col)	{
#if defined(_DEBUG)||(MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (col<0||col>=m_Cols) THROW_EXCEPTION("Index out of range!");
#endif
		for (size_t i=0;i<m_Rows;i++) for (size_t j=col;j<m_Cols-1;j++) m_Val[i][j]=m_Val[i][j+1];
		realloc(m_Rows,m_Cols-1);
	}

	/** Extract a sub matrix from a matrix:
		* \exception std::exception On index out of bounds
		* The output matrix must have been previously set to the desired size.
		* (by AJOGD @ JAN-2007)
		*/
	template <class R>
	void  extractMatrix(size_t nRow, size_t nCol, CMatrixTemplate<R> &in) const
	{
		size_t		i,j,ncols,nrows;

		nrows = in.getRowCount();
		ncols = in.getColCount();
		if ( (nRow+nrows > CMatrixTemplate<T>::getRowCount()) || (nCol+ncols >CMatrixTemplate<T>::getColCount()) )
			THROW_EXCEPTION("extractMatrix: Row or Col index out of bounds");

		for (i=nRow;i<nRow+nrows;i++)
			for(j=nCol;j<nCol+ncols;j++)
				in.set_unsafe( i-nRow,j-nCol, static_cast<R> ( CMatrixTemplate<T>::m_Val[i][j] ) );
	}

	/** Extract a sub matrix (vector) from a matrix:
		* \exception std::exception On index out of bounds
		* The output matrix must have been previously set to the desired size.
		* (by AJOGD @ JAN-2007)
		*/
	void  extractMatrix(size_t nRow, size_t nCol, std::vector<T> &in) const
	{
		size_t		j,ncols,nrows;

		ncols = in.size();
		nrows = 1;
		if ( (nRow+nrows > CMatrixTemplate<T>::getRowCount()) || (nCol+ncols >CMatrixTemplate<T>::getColCount()) )
			THROW_EXCEPTION("extractMatrix: Row or Col index out of bounds");

		for(j=nCol;j<nCol+ncols;j++)
				in[j-nCol] = CMatrixTemplate<T>::m_Val[nRow][j];
	}

	/** Extract a sub matrix from this matrix (the size of the output matrix upon call determines the size of the submatrix to extract).
		* \exception std::exception On index out of bounds
		* The output matrix settles the size of the submatrix to extract, and (nRow,nCol) are the
		* indices of the first element in the submatrix: e.g. (0,0) means extract a submatrix from the top-left corner
		*/
	template <size_t NROWS,size_t NCOLS>
	inline void extractMatrix(const size_t nRow, const size_t nCol, CMatrixFixedNumeric<T,NROWS,NCOLS> &outMat) const {
		detail::extractMatrix(*this,nRow,nCol,outMat);
	}

	/** Extract a sub matrix (vector) from a matrix:
		* \exception std::exception On index out of bounds
		* The output matrix must have been previously set to the desired size.
		* (by AJOGD @ JAN-2007)
		*/
	std::vector<T>  extractMatrix(size_t nRow, size_t nCol, size_t ncols) const
	{
		size_t		j;
		std::vector<T> out;
		out.resize(ncols);

		if ( (nRow+1 > CMatrixTemplate<T>::getRowCount()) || (nCol+ncols >CMatrixTemplate<T>::getColCount()) )
			THROW_EXCEPTION("extractMatrix: Row or Col index out of bounds");

		for(j=nCol;j<nCol+ncols;j++)
				out[j-nCol] = CMatrixTemplate<T>::m_Val[nRow][j];
		return out;
	}


	/** Dump matrix in matlab format.
	  *  This template method can be instantiated for matrices of the types: int, long, unsinged int, unsigned long, float, double, long double
	  * \sa fromMatlabStringFormat
	  */
	std::string  inMatlabFormat(const size_t decimal_digits = 6) const
	{
		std::stringstream  s;

		s << "[";
		s << std::scientific;
		s.precision(decimal_digits);
		for (size_t i=0;i<m_Rows;i++)
		{
			for (size_t j=0;j<m_Cols;j++)
				s << m_Val[i][j] << " ";

			if (i<m_Rows-1)	s << ";";
		}
		s << "]";

		return s.str();
	}


	/** Read a matrix from a string in Matlab-like format, for example "[1 0 2; 0 4 -1]"
	  *  The string must start with '[' and end with ']'. Rows are separated by semicolons ';' and
	  *  columns in each row by one or more whitespaces ' ' or tabs.
	  *
	  * This format is also used for CConfigFile::read_matrix.
	  *
	  *  This template method can be instantiated for matrices of the types: int, long, unsinged int, unsigned long, float, double, long double
	  *
	  * \return true on success. false if the string is malformed, and then the matrix will be resized to 0x0.
	  * \sa inMatlabFormat, CConfigFile::read_matrix
	  */
	bool fromMatlabStringFormat(const std::string &s)
	{
		this->setSize(0,0);

		// Look for starting "[".
		size_t  ini = s.find_first_not_of(" \t\r\n");
		if (ini==std::string::npos || s[ini]!='[') return false;

		size_t  end = s.find_last_not_of(" \t\r\n");
		if (end==std::string::npos || s[end]!=']') return false;

		if (ini>end) return false;

		std::vector<T> lstElements;

		size_t i = ini+1;
		size_t nRow = 0;

		while (i<end)
		{
			// Extract one row:
			size_t end_row = s.find_first_of(";]",i);
			if (end_row==std::string::npos) { setSize(0,0); return false; }

			// We have one row in s[ i : (end_row-1) ]
			std::stringstream  ss (s.substr(i, end_row-i ));
			lstElements.clear();
			try
			{
				while (!ss.eof())
				{
					T val;
					ss >> val;
					if (ss.bad() || ss.fail()) break;
					lstElements.push_back(val);
				}
			} catch (...) { }  // end of line

			// Empty row? Only for the first row, then this is an empty matrix:
			if (lstElements.empty())
			{
				if (nRow>0) {  setSize(0,0); return false; }
				// Else, this may be an empty matrix... if there is no next row, we'll return with a (0,0) matrix
			}
			else
			{
				const size_t N = lstElements.size();

				// Check valid width:
				if (nRow>0 && m_Cols!=N)  {  setSize(0,0); return false; }  // All rows must have the same width

				// Append to the matrix:
				realloc( nRow+1,N );

				for (size_t q=0;q<N;q++)
					m_Val[nRow][q] = lstElements[q];

				// Go for the next row:
				nRow++;
			}

			i = end_row+1;
		}
		return true; // Ok
	}

	/** @name Import/export as text
		@{ */

	/** Save matrix to a text file, compatible with MATLAB text format.
		* \param file The target filename.
		* \param fileFormat See TMatrixTextFileFormat. The format of the numbers in the text file.
		* \param appendMRPTHeader Insert this header to the file "% File generated by MRPT. Load with MATLAB with: VAR=load(FILENAME);"
		* \param userHeader Additional text to be written at the head of the file. Typically MALAB comments "% This file blah blah". Final end-of-line is not needed.
		* \sa loadFromTextFile, CMatrixTemplate::inMatlabFormat, SAVE_MATRIX
		*/
	inline void  saveToTextFile(
		const std::string &file,
		TMatrixTextFileFormat fileFormat = MATRIX_FORMAT_ENG,
		bool    appendMRPTHeader = false,
		const std::string &userHeader = std::string("")
		) const
	{
		detail::saveMatrixToTextFile(*this, file,fileFormat,appendMRPTHeader,userHeader);
	}

	/** Load matrix from a text file, compatible with MATLAB text format.
	  *  Lines starting with '%' or '#' are interpreted as comments and ignored.
	  * \sa saveToTextFile, CMatrixTemplate::fromMatlabStringFormat
	  */
	void  loadFromTextFile(const std::string &file)
	{
		std::ifstream	f(file.c_str());
		if (f.fail()) THROW_EXCEPTION_CUSTOM_MSG1("loadFromTextFile: can't open file:'%s'",file.c_str());

		std::string		str;
		std::vector<double>	fil(512);

		const char	*ptr;
		char		*ptrEnd;
		size_t	i,j;
		size_t	nCols = std::numeric_limits<size_t>::max();
		size_t	nRows = 0;

		CMatrixTemplate<T>::realloc(0,0);

		while ( !f.eof() )
		{
			std::getline(f,str);

			if (str.size() && str[0]!='#' && str[0]!='%')
			{
				// Parse row to floats:
				ptr = str.c_str();

				ptrEnd = NULL;
				i=0;

				// Process each number in this row:
				while ( ptr[0] && ptr!=ptrEnd )
				{
					// Find next number: (non white-space character):
					while (ptr[0] && (ptr[0]==' ' || ptr[0]=='\t' || ptr[0]=='\r' || ptr[0]=='\n'))
						ptr++;

					if (fil.size()<=i)	fil.resize(fil.size()+512);

					// Convert to "double":
					fil[i] = strtod(ptr,&ptrEnd);

					// A valid conversion has been done?
					if (ptr!=ptrEnd)
					{
						i++;	// Yes
						ptr = ptrEnd;
						ptrEnd = NULL;
					}
				}; // end while procesing this row

				if (nCols==std::numeric_limits<size_t>::max())
				{
					// First row:
					nCols = i;
					CMatrixTemplate<T>::realloc(nCols,nCols);
				}
				else
				{
					// Same elements count in each row?
					if (CMatrixTemplate<T>::getColCount()!=i )
						THROW_EXCEPTION("The matrix in the text file must have the same number of elements in each row!");
				}

				// Copy row to matrix:
				for (j=0;j<nCols;j++)
					CMatrixTemplate<T>::m_Val[nRows][j] = myStaticCast<T>(fil[j]); // static_cast<T>(fil[j]);

				nRows++;
				if (nRows >= CMatrixTemplate<T>::getRowCount() )
					CMatrixTemplate<T>::realloc( nRows+10, nCols );

			} // end if fgets

		} // end while not feof

		if (nRows && nCols)
			CMatrixTemplate<T>::realloc( nRows, nCols );

		// Report error as exception
		if ( !CMatrixTemplate<T>::getRowCount() || !CMatrixTemplate<T>::getColCount() )
			THROW_EXCEPTION("loadFromTextFile: Error loading from text file");
	}

	/** @} */

	/** Removes a set of columns by their indexes (0:first), given by the vector "idxsToRemove".
	  *  \param vectorIsAlreadySorted Can be set to true only when it can be assured that the vector of indices is sorted in ascending order. Otherwise, the method will sort it internally.
	  * \note All the indices MUST BE UNIQUE. Behavior on duplicated indices is undefined.
	  * \exception std::exception On index out of bounds
	  */
	void removeColumns( const mrpt::vector_size_t &idxsToRemove, bool vectorIsAlreadySorted = false)
	{
		MRPT_START

		if (idxsToRemove.empty()) return;
		ASSERT_(idxsToRemove.size()<=m_Cols);
		if (idxsToRemove.size()==m_Cols)
		{
			this->setSize(m_Rows,0);
			return;
		}

		const vector_size_t *idxs = &idxsToRemove; // The real, sorted indexes to use.
		vector_size_t auxSortedIdxs;

		if (!vectorIsAlreadySorted)
		{
			auxSortedIdxs = idxsToRemove;
			std::sort(auxSortedIdxs.begin(),auxSortedIdxs.end());
			idxs = &idxsToRemove;
		}

		size_t newColCount = m_Cols;

		for (vector_size_t::const_reverse_iterator i=idxs->rbegin();i!=idxs->rend();++i)
		{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (*i>=m_Cols) THROW_EXCEPTION("removeColumns: Column index out of bounds");
#endif
			for (size_t row=0;row<m_Rows;row++)
			{
				// Move elements [*i+1,newColCount-1] -> [*i,newColCount-2]
				for (size_t c=*i;c<newColCount;c++)
					m_Val[row][c] = m_Val[row][c+1];
			}
			newColCount--;
		} // end for

		m_Cols = newColCount;	// Set new size:

		MRPT_END
	}

	/** Returns a vector containing the matrix's values.
	  */
	void getAsVector(std::vector<T> &out) const	{
		out.clear();
		out.reserve(m_Rows*m_Cols);
		for (size_t i=0;i<m_Rows;i++) out.insert(out.end(),&(m_Val[i][0]),&(m_Val[i][m_Cols]));
	}

	/** Returns a submatrix with random indices to the matrix, à la Matlab.
	  */
	CMatrixTemplate<T> operator()(const std::vector<size_t> &rows,const std::vector<size_t> &cols) const	{
		struct gtN	{
			size_t N;
			gtN(size_t el):N(el)	{}
			bool operator()(size_t el)	{
				return el>=N;
			}
		};
		ASSERT_((find_if(rows.begin(),rows.end(),gtN(m_Rows))!=rows.end())||(find_if(cols.begin(),cols.end(),gtN(m_Cols))!=cols.end()));
		size_t nR=rows.size(),nC=cols.size();
		CMatrixTemplate<T> res(nR,nC);
		for (size_t i=0;i<nR;++i) for (size_t j=0;j<nC;++j) res(i,j)=m_Val[rows[i]][cols[i]];
		return res;
	}

	/** Removes arbitrary rows and columns from the matrix.
	  */
	void removeRowsAndCols(const std::set<size_t> &rows,const std::set<size_t> &cols)	{
		struct gtN	{
			size_t N;
			gtN(size_t el):N(el)	{}
			bool operator()(size_t el)	{
				return el>=N;
			}
		};
		ASSERT_((find_if(rows.begin(),rows.end(),gtN(m_Rows))==rows.end())||(find_if(cols.begin(),cols.end(),gtN(m_Cols))==cols.end()));
		if (rows.empty()&&cols.empty()) return;
		//Determine the new amount of rows and columns
		size_t newRows=m_Rows-rows.size();
		size_t newCols=m_Cols-cols.size();
		#ifdef _DEBUG
			ASSERT_(newRows<=m_Rows);
			ASSERT_(newCols<=m_Cols);
		#endif
		if (newRows==0||newCols==0)	{
			realloc(newRows,newCols);
			return;
		}
		//For both rows and columns, create a vector containing the new offsets to which the copy operation must reference.
		//That is, if a matrix has 7 rows and 4 columns, and this method is asked to remove rows (0,2,3) and column (3),
		//the following code creates two vectors: [1,4,5,6] for the rows and [0,1,2] for the cols.
		std::vector<size_t> rowsOffset;
		rowsOffset.reserve(newRows);
		std::vector<size_t> colsOffset;
		colsOffset.reserve(newCols);
		std::set<size_t>::const_iterator it1=rows.begin();
		size_t next=0;
		while (it1!=rows.end())	{
			if (next<*it1) rowsOffset.push_back(next);
			else ++it1;
			++next;
		}
		for (;next<m_Rows;++next) rowsOffset.push_back(next);
		it1=cols.begin();
		next=0;
		while (it1!=cols.end())	{
			if (next<*it1) colsOffset.push_back(next);
			else ++it1;
			++next;
		}
		for (;next<m_Cols;++next) colsOffset.push_back(next);
		//Once both vectors are created, copy the new matrix (done so that values are not wrongly overwrited). Self-assignment is avoided for efficiency.
		if (cols.empty()) for (size_t i=*rows.begin();i<newRows;++i) for (size_t j=0;j<newCols;++j) m_Val[i][j]=m_Val[rowsOffset[i]][colsOffset[j]];
		else if (rows.empty()) for (size_t i=0;i<newRows;++i) for (size_t j=*cols.begin();j<newCols;++j) m_Val[i][j]=m_Val[rowsOffset[i]][colsOffset[j]];
		else	{
			for (size_t j=*cols.begin();j<newCols;++j) m_Val[0][j]=m_Val[rowsOffset[0]][colsOffset[j]];
			for (size_t i=1;i<newRows;++i) for (size_t j=0;j<newCols;++j) m_Val[i][j]=m_Val[rowsOffset[i]][colsOffset[j]];
		}
		//Once the new block has been correctly assigned, realloc memory so that the actual size is correctly reflected.
		realloc(newRows,newCols);
	}

	/** Inserts rows and columns as needed. For example, if the row 3 exists twice in the multiset, two rows will be inserted _before_ the third row.
	  * Any index greater than the size will cause the column or row to be inserted right after the last element.
	  * THIS FUNCTION IS NOT SECURE TO USE WITH CMatrixTemplateObjects.
	  */
	void insertRowsAndCols(const std::multiset<size_t> &rows,const std::multiset<size_t> &cols,const T &defaultValue=T())	{
		size_t newRows=m_Rows+rows.size();
		size_t newCols=m_Cols+cols.size();
		std::vector<size_t> rowsOffset(m_Rows);
		std::vector<size_t> insertedRows(rows.size());
		std::vector<size_t> colsOffset(m_Cols);
		std::vector<size_t> insertedCols(cols.size());
		//Calculate the new distribution of the data (i.e. check where does the existing data fit).
		size_t iOffset=0;
		size_t iInserted=0;
		std::multiset<size_t>::const_iterator itInserted=rows.begin();
		for (size_t i=0;i<newRows;++i)	{
			if (itInserted==rows.end()) rowsOffset[iOffset++]=i;
			else if (*itInserted<=iOffset)	{
				insertedRows[iInserted++]=i;
				itInserted++;
			}	else rowsOffset[iOffset++]=i;
		}
		iOffset=0;
		iInserted=0;
		itInserted=cols.begin();
		for (size_t i=0;i<newCols;++i)	{
			if (itInserted==cols.end()) colsOffset[iOffset++]=i;
			else if (*itInserted<=iOffset)	{
				insertedCols[iInserted++]=i;
				itInserted++;
			}	else colsOffset[iOffset++]=i;
		}
		//Realloc memory as needed.
		T **newVal=static_cast<T **>(mrpt::system::os::aligned_calloc(sizeof(T *)*newRows,16));
		size_t rowSize=sizeof(T)*newCols;
		for (size_t i=0;i<newRows;++i) newVal[i]=static_cast<T *>(mrpt::system::os::aligned_calloc(rowSize,16));
		//Copy previous data
		for (size_t i=0;i<m_Rows;++i) for (size_t j=0;j<m_Cols;++j) newVal[rowsOffset[i]][colsOffset[j]]=m_Val[i][j];
		//Fill the gaps with the default value
		for (size_t i=0;i<m_Rows;++i) for (size_t j=0;j<insertedCols.size();++j) newVal[rowsOffset[i]][insertedCols[j]]=defaultValue;
		for (size_t i=0;i<insertedRows.size();++i) for (size_t j=0;j<newCols;++j) newVal[insertedRows[i]][j]=defaultValue;
		//Delete previous memory
		for (size_t i=0;i<m_Rows;++i) mrpt::system::os::aligned_free(m_Val[i]);
		mrpt::system::os::aligned_free(m_Val);
		//Assign the right member variables.
		m_Val=newVal;
		m_Rows=newRows;
		m_Cols=newCols;
	}
	/**
	  * Method to retrieve the vicinity of an element, given its coordinates.
	  * Current implementation allows up to then different vicinities (4, 5, 8, 9, 12, 13, 20, 21, 24 and 25) and two different return values:
	  * CMatrixTemplate<T> and vector<T>.
	  */
	template<size_t N,typename ReturnType> inline ReturnType getVicinity(size_t c,size_t r) const	{
		return detail::getVicinity<CMatrixTemplate<T>,T,ReturnType,N>::get(c,r,*this);
	}


}; // end of class definition


	} // End of namespace
} // End of namespace


#endif
