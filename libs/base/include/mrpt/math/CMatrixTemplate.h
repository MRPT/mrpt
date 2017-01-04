/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMatrixTemplate_H
#define CMatrixTemplate_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/memory.h>
#include <mrpt/math/math_frwds.h>  // forward declarations
#include <mrpt/utils/CArray.h>  // type CMatrixTemplateSize
#include <algorithm>  // swap()

namespace mrpt
{
	namespace math
	{
		/** Auxiliary class used in CMatrixTemplate:size(), CMatrixTemplate::resize(), CMatrixFixedNumeric::size(), CMatrixFixedNumeric::resize(), to mimic the behavior of STL-containers */
		struct CMatrixTemplateSize : public mrpt::utils::CArray<size_t, 2>
		{
			typedef mrpt::utils::CArray<size_t, 2> Base;
			typedef CMatrixTemplateSize mrpt_autotype;

			inline CMatrixTemplateSize() : mrpt::utils::CArray<size_t, 2>() {}
			inline CMatrixTemplateSize(const size_t *d) { (*this)[0] = d[0]; (*this)[1] = d[1]; }

			inline bool operator==(const CMatrixTemplateSize&o) const { return Base::operator[](0) == o[0] && Base::operator[](1) == o[1]; }
			inline bool operator!=(const CMatrixTemplateSize&o) const { return !(*this == o); }
			/** This operator allows the size(N,M) to be compared with a plain size_t N*M  */
			inline operator size_t(void) const { return 2; }
		};


		/**  This template class provides the basic functionality for a general 2D any-size, resizable container of numerical or non-numerical elements.
		 * NOTES:
		 *		- This class is not serializable since it is a template. For using serialization, see mrpt::math::CMatrixNumeric
		 *		- First row or column index is "0".
		 *		- This class includes range checks with ASSERT_() if compiling with "_DEBUG" or "MRPT_ALWAYS_CHECKS_DEBUG_MATRICES=1".
		 *		- Please DO NOT use as template class type any other class. It can be safely used the following types:
		 *			- Elemental types (int,char,float,doble,...)
		 *			- Data struct (Not classes!)
		 *			- Any kind of pointers (user is responsible for allocating and freeing the memory addressed by pointers).
		 *
		 * \note Memory blocks for each row are 16-bytes aligned (since MRPT 0.7.0).
		 * \note For a complete introduction to Matrices and vectors in MRPT, see: http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
		 * \sa CMatrixTemplateNumeric
		 * \ingroup mrpt_base_grp
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

			/** Constructors */
			CMatrixTemplate (const CMatrixTemplate& m) : m_Val(NULL),m_Rows(0),m_Cols(0)
			{
				(*this) = m;
			}

			CMatrixTemplate (size_t row = 1, size_t col = 1) :  m_Val(NULL),m_Rows(0),m_Cols(0)
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

			/** Destructor */
			virtual ~CMatrixTemplate() { realloc(0,0); }

			/** Assignment operator from another matrix */
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
					THROW_EXCEPTION( format("Row index %lu out of range. Matrix is %lux%lu",static_cast<unsigned long>(row),static_cast<unsigned long>(m_Rows),static_cast<unsigned long>(m_Cols)) );
		#endif
				return m_Val[row];
			}

			/** Fast but unsafe method to obtain a pointer to a given row of the matrix (Use only in critical applications)
				*/
			inline const T* get_unsafe_row(size_t row) const	{
				return m_Val[row];
			}

			/** Subscript operator to get a submatrix
			  */
			inline CMatrixTemplate<T> operator() (const size_t row1,const size_t row2,const size_t col1,const size_t col2) const	{
				CMatrixTemplate<T> val(0,0);
				extractSubmatrix(row1,row2,col1,col2,val);
				return val;
			}

			/** Get a submatrix, given its bounds
			  * \sa extractSubmatrixSymmetricalBlocks
			  */
			void extractSubmatrix(const size_t row1,const size_t row2,const size_t col1,const size_t col2,CMatrixTemplate<T> &out) const
			{
				int nrows=int(row2)-int(row1)+1;
				int ncols=int(col2)-int(col1)+1;
				if (nrows<=0||ncols<=0)	{
					out.realloc(0,0);
					return;
				}
				if (row2>=m_Rows||col2>=m_Cols) THROW_EXCEPTION("Indices out of range!");
				out.realloc(nrows,ncols);
				for (int i=0;i<nrows;i++) for (int j=0;j<ncols;j++) out.m_Val[i][j]=m_Val[i+row1][j+col1];
			}
			/// @overload
			template <class EIGEN_MATRIX>
			void extractSubmatrix(const size_t row1,const size_t row2,const size_t col1,const size_t col2,EIGEN_MATRIX &out) const
			{
				int nrows=int(row2)-int(row1)+1;
				int ncols=int(col2)-int(col1)+1;
				if (nrows<=0||ncols<=0)	{
					out = typename EIGEN_MATRIX::PlainObject();
					return;
				}
				if (row2>=m_Rows||col2>=m_Cols) THROW_EXCEPTION("Indices out of range!");
				out.resize(nrows,ncols);
				for (int i=0;i<nrows;i++) for (int j=0;j<ncols;j++) out.coeffRef(i,j)=m_Val[i+row1][j+col1];
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

			/** Appends a new row to the MxN matrix from a 1xN vector.
				*  The lenght of the vector must match the width of the matrix, unless it's empty: in that case the matrix is resized to 1xN.
				*  \code
				*    CMatrixDouble  M(0,0);
				*    CVectorDouble  v(7),w(7);
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

			/** Returns a vector containing the matrix's values.
			  */
			void getAsVector(std::vector<T> &out) const	{
				out.clear();
				out.reserve(m_Rows*m_Cols);
				for (size_t i=0;i<m_Rows;i++) out.insert(out.end(),&(m_Val[i][0]),&(m_Val[i][m_Cols]));
			}

		}; // end of class CMatrixTemplate

		/** Declares a matrix of booleans (non serializable).
		  *  \sa CMatrixDouble, CMatrixFloat, CMatrixB
		  */
		//typedef CMatrixTemplate<bool> CMatrixBool;
		class BASE_IMPEXP CMatrixBool : public CMatrixTemplate<bool>
		{
		public:
			/** Constructor */
			CMatrixBool(size_t row=1, size_t col=1);
			/** Copy constructor */
			CMatrixBool( const CMatrixTemplate<bool> &m );
			/** Assignment operator for float matrixes */
			CMatrixBool & operator = (const CMatrixTemplate<bool> & m);
		};

	} // End of namespace
} // End of namespace


#endif
