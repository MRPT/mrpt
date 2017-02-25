/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CSparseMatrix_H
#define CSparseMatrix_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/exceptions.h>
#include <mrpt/utils/CUncopiable.h>

#include <mrpt/math/math_frwds.h>
#include <mrpt/utils/types_math.h>
#include <mrpt/math/CSparseMatrixTemplate.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <cstring> // memcpy

// Include CSparse lib headers, either from the system or embedded:
extern "C"{
#if MRPT_HAS_SUITESPARSE
#	define NCOMPLEX    // In MRPT we don't need complex numbers, so avoid the annoying warning: 'cs_ci_house' has C-linkage specified, but returns UDT 'std::complex<double>' which is incompatible with C
#	include "cs.h"
#else
#	include <mrpt/otherlibs/CSparse/cs.h>
#endif
}

namespace mrpt
{
	namespace math
	{
		/** Used in mrpt::math::CSparseMatrix */
		class CExceptionNotDefPos : public mrpt::utils::CMRPTException
		{
		public:
			CExceptionNotDefPos(const std::string &s) : CMRPTException(s) {  }
		};


		/** A sparse matrix structure, wrapping T. Davis' CSparse library (part of suitesparse)
		  *  The type of the matrix entries is fixed to "double".
		  *
		  *  There are two formats for the non-zero entries in this matrix:
		  *		- A "triplet" matrix: a set of (r,c)=val triplet entries.
		  *		- A column-compressed sparse (CCS) matrix.
		  *
		  *  The latter is the "normal" format, which is expected by all mathematical operations defined
		  *   in this class. There're three ways of initializing and populating a sparse matrix:
		  *
		  *   <ol>
		  *    <li> <b>As a triplet (empty), then add entries, then compress:</b>
		  *         \code
		  *             CSparseMatrix  SM(100,100);
		  *             SM.insert_entry(i,j, val);  // or
		  *             SM.insert_submatrix(i,j, MAT); //  ...
		  *             SM.compressFromTriplet();
		  *         \endcode
		  *    </li>
		  *    <li> <b>As a triplet from a CSparseMatrixTemplate<double>:</b>
		  *         \code
		  *             CSparseMatrixTemplate<double>  data;
		  *             data(row,col) = val;
		  *             ...
		  *             CSparseMatrix  SM(data);
		  *         \endcode
		  *    </li>
		  *    <li> <b>From an existing dense matrix:</b>
		  *         \code
		  *             CMatrixDouble data(100,100); // or
		  *             CMatrixFloat  data(100,100); // or
		  *             CMatrixFixedNumeric<double,4,6>  data; // etc...
		  *             CSparseMatrix  SM(data);
		  *         \endcode
		  *    </li>
		  *
		  *   </ol>
		  *
		  *  Due to its practical utility, there is a special inner class CSparseMatrix::CholeskyDecomp to handle Cholesky-related methods and data.
		  *
		  * \note This class was initially adapted from "robotvision", by Hauke Strasdat, Steven Lovegrove and Andrew J. Davison. See http://www.openslam.org/robotvision.html
		  * \note CSparse is maintained by Timothy Davis: http://people.sc.fsu.edu/~jburkardt/c_src/csparse/csparse.html .
		  * \note See also his book "Direct methods for sparse linear systems". http://books.google.es/books?id=TvwiyF8vy3EC&pg=PA12&lpg=PA12&dq=cs_compress&source=bl&ots=od9uGJ793j&sig=Wa-fBk4sZkZv3Y0Op8FNH8PvCUs&hl=es&ei=UjA0TJf-EoSmsQay3aXPAw&sa=X&oi=book_result&ct=result&resnum=8&ved=0CEQQ6AEwBw#v=onepage&q&f=false
		  *
		  * \sa mrpt::math::MatrixBlockSparseCols, mrpt::math::CMatrixFixedNumeric, mrpt::math::CMatrixTemplateNumeric, etc.
		  * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CSparseMatrix
		{
		private:
			cs sparse_matrix;

			/** Initialization from a dense matrix of any kind existing in MRPT. */
			template <class MATRIX>
			void construct_from_mrpt_mat(const MATRIX & C)
			{
				std::vector<int> row_list, col_list;  // Use "int" for convenience so we can do a memcpy below...
				std::vector<double> content_list;
				const int nCol = C.getColCount();
				const int nRow = C.getRowCount();
				for (int c=0; c<nCol; ++c)
				{
					col_list.push_back(row_list.size());
					for (int r=0; r<nRow; ++r)
						if (C.get_unsafe(r,c)!=0)
						{
							row_list.push_back(r);
							content_list.push_back(C(r,c));
						}
				}
				col_list.push_back(row_list.size());

				sparse_matrix.m = nRow;
				sparse_matrix.n = nCol;
				sparse_matrix.nzmax = content_list.size();
				sparse_matrix.i = (int*)malloc(sizeof(int)*row_list.size());
				sparse_matrix.p = (int*)malloc(sizeof(int)*col_list.size());
				sparse_matrix.x = (double*)malloc(sizeof(double)*content_list.size());

				::memcpy(sparse_matrix.i, &row_list[0], sizeof(row_list[0])*row_list.size() );
				::memcpy(sparse_matrix.p, &col_list[0], sizeof(col_list[0])*col_list.size() );
				::memcpy(sparse_matrix.x, &content_list[0], sizeof(content_list[0])*content_list.size() );

				sparse_matrix.nz = -1;  // <0 means "column compressed", ">=0" means triplet.
			}

			/** Initialization from a triplet "cs", which is first compressed */
			void construct_from_triplet(const cs & triplet);

			/** To be called by constructors only, assume previous pointers are trash and overwrite them */
			void construct_from_existing_cs(const cs &sm);

			/** free buffers (deallocate the memory of the i,p,x buffers) */
			void internal_free_mem();

			/** Copy the data from an existing "cs" CSparse data structure */
			void  copy(const cs  * const sm);

			/** Fast copy the data from an existing "cs" CSparse data structure, copying the pointers and leaving NULLs in the source structure. */
			void  copy_fast(cs  * const sm);

		public:

			/** @name Constructors, destructor & copy operations
			    @{  */

			/** Create an initially empty sparse matrix, in the "triplet" form.
			  *  Notice that you must call "compressFromTriplet" after populating the matrix and before using the math operatons on this matrix.
			  *  The initial size can be later on extended with insert_entry() or setRowCount() & setColCount().
			  * \sa insert_entry, setRowCount, setColCount
			  */
			CSparseMatrix(const size_t nRows=0, const size_t nCols=0);

			/** A good way to initialize a sparse matrix from a list of non NULL elements.
			  *  This constructor takes all the non-zero entries in "data" and builds a column-compressed sparse representation.
			  */
			template <typename T>
			inline CSparseMatrix(const CSparseMatrixTemplate<T> & data)
			{
				ASSERTMSG_(!data.empty(), "Input data must contain at least one non-zero element.")
				sparse_matrix.i = NULL; // This is to know they shouldn't be tried to free()
				sparse_matrix.p = NULL;
				sparse_matrix.x = NULL;

				// 1) Create triplet matrix
				CSparseMatrix  triplet(data.getRowCount(),data.getColCount());
				// 2) Put data in:
				for (typename CSparseMatrixTemplate<T>::const_iterator it=data.begin();it!=data.end();++it)
					triplet.insert_entry_fast(it->first.first,it->first.second, it->second);

				// 3) Compress:
				construct_from_triplet(triplet.sparse_matrix);
			}


			// We can't do a simple "template <class ANYMATRIX>" since it would be tried to match against "cs*"...

			/** Constructor from a dense matrix of any kind existing in MRPT, creating a "column-compressed" sparse matrix. */
			template <typename T,size_t N,size_t M> inline explicit CSparseMatrix(const CMatrixFixedNumeric<T,N,M> &MAT) { construct_from_mrpt_mat(MAT); }

			/** Constructor from a dense matrix of any kind existing in MRPT, creating a "column-compressed" sparse matrix. */
			template <typename T>  inline explicit CSparseMatrix(const CMatrixTemplateNumeric<T> &MAT) { construct_from_mrpt_mat(MAT); }

			/** Copy constructor */
			CSparseMatrix(const CSparseMatrix & other);

			/** Copy constructor from an existing "cs" CSparse data structure */
			explicit CSparseMatrix(const cs  * const sm);

			/** Destructor */
			virtual ~CSparseMatrix();

			/** Copy operator from another existing object */
			void operator = (const CSparseMatrix & other);

			/** Fast swap contents with another sparse matrix */
			void swap(CSparseMatrix & other);

			/** Erase all previous contents and leave the matrix as a "triplet" ROWS x COLS matrix without any nonzero entry. */
			void clear(const size_t nRows=1, const size_t nCols=1);

			/** @}  */

			/** @name Math operations (the interesting stuff...)
				@{ */

			void add_AB(const CSparseMatrix & A,const CSparseMatrix & B); //!< this = A+B
			void multiply_AB(const CSparseMatrix & A,const CSparseMatrix & B); //!< this = A*B
			void multiply_Ab(const mrpt::math::CVectorDouble &b, mrpt::math::CVectorDouble &out_res) const; //!< out_res = this * b

			inline CSparseMatrix operator + (const CSparseMatrix & other) const
			{
				CSparseMatrix RES;
				RES.add_AB(*this,other);
				return RES;
			}
			inline CSparseMatrix operator * (const CSparseMatrix & other) const
			{
				CSparseMatrix RES;
				RES.multiply_AB(*this,other);
				return RES;
			}
			inline mrpt::math::CVectorDouble operator * (const mrpt::math::CVectorDouble & other) const {
				mrpt::math::CVectorDouble res;
				multiply_Ab(other,res);
				return res;
			}
			inline void operator += (const CSparseMatrix & other) {
				this->add_AB(*this,other);
			}
			inline void operator *= (const CSparseMatrix & other) {
				this->multiply_AB(*this,other);
			}
			CSparseMatrix transpose() const;

			/** @}  */


			/** @ Access the matrix, get, set elements, etc.
			    @{ */

			/** ONLY for TRIPLET matrices: insert a new non-zero entry in the matrix.
			  *  This method cannot be used once the matrix is in column-compressed form.
			  *  The size of the matrix will be automatically extended if the indices are out of the current limits.
			  * \sa isTriplet, compressFromTriplet
			  */
			void insert_entry(const size_t row, const size_t col, const double val );

			/** This was an optimized version, but is now equivalent to insert_entry() due to the need to be compatible with unmodified CSparse system libraries. */
			inline void insert_entry_fast(const size_t row, const size_t col, const double val )  { insert_entry(row,col,val); }

			/** ONLY for TRIPLET matrices: insert a given matrix (in any of the MRPT formats) at a given location of the sparse matrix.
			  *  This method cannot be used once the matrix is in column-compressed form.
			  *  The size of the matrix will be automatically extended if the indices are out of the current limits.
			  *  Since this is inline, it can be very efficient for fixed-size MRPT matrices.
			  * \sa isTriplet, compressFromTriplet, insert_entry
			  */
			template <class MATRIX>
			inline void insert_submatrix(const size_t row, const size_t col, const MATRIX &M )
			{
				if (!isTriplet()) THROW_EXCEPTION("insert_entry() is only available for sparse matrix in 'triplet' format.")
				const size_t nR = M.getRowCount();
				const size_t nC = M.getColCount();
				for (size_t r=0;r<nR;r++)
					for (size_t c=0;c<nC;c++)
						insert_entry_fast(row+r,col+c, M.get_unsafe(r,c));
				// If needed, extend the size of the matrix:
				sparse_matrix.m = std::max(sparse_matrix.m, int(row+nR));
				sparse_matrix.n = std::max(sparse_matrix.n, int(col+nC));
			}


			/** ONLY for TRIPLET matrices: convert the matrix in a column-compressed form.
			  * \sa insert_entry
			  */
			void compressFromTriplet();

			/** Return a dense representation of the sparse matrix.
			  * \sa saveToTextFile_dense
			  */
			void get_dense(CMatrixDouble &outMat) const;

			/** Static method to convert a "cs" structure into a dense representation of the sparse matrix.
			  */
			static void cs2dense(const cs& SM, CMatrixDouble &outMat);

			/** save as a dense matrix to a text file \return False on any error.
			  */
			bool saveToTextFile_dense(const std::string &filName);

			/** Save sparse structure to a text file loadable from MATLAB (can be called on triplet or CCS matrices).
			  *
			  *  The format of the text file is:
			  *  \code
			  *   NUM_ROWS   NUM_COLS   NUM_NON_ZERO_MAX
			  *   row_1   col_1   value_1
			  *   row_2   col_2   value_2
			  *   ...
			  *  \endcode
			  *
			  *  Instructions for loading from MATLAB in triplet form will be automatically writen to the
			  *  output file as comments in th first lines:
			  *
			  *   \code
			  *      D=load('file.txt');
			  *      SM=spconvert(D(2:end,:));
			  *        or, to always preserve the actual matrix size m x n:
                 *      m=D(1,1); n=D(1,2); nzmax=D(1,3);
			  *      Di=D(2:end,1); Dj=D(2:end,2); Ds=D(2:end,3);
			  *      M=sparse(Di,Dj,Ds, m,n, nzmax);
			  *   \endcode
			  * \return False on any error.
			  */
			bool saveToTextFile_sparse(const std::string &filName);

			// Very basic, standard methods that MRPT methods expect for any matrix:
			inline size_t getRowCount() const { return sparse_matrix.m; }
			inline size_t getColCount() const { return sparse_matrix.n; }

			/** Change the number of rows in the matrix (can't be lower than current size) */
			inline void setRowCount(const size_t nRows) { ASSERT_(nRows>=(size_t)sparse_matrix.m); sparse_matrix.m = nRows; }
			inline void setColCount(const size_t nCols) { ASSERT_(nCols>=(size_t)sparse_matrix.n); sparse_matrix.n = nCols; }

			/** Returns true if this sparse matrix is in "triplet" form. \sa isColumnCompressed */
			inline bool isTriplet() const { return sparse_matrix.nz>=0; }  // <0 means "column compressed", ">=0" means triplet.

			/** Returns true if this sparse matrix is in "column compressed" form. \sa isTriplet */
			inline bool isColumnCompressed() const { return sparse_matrix.nz<0; }  // <0 means "column compressed", ">=0" means triplet.

			/** @} */


			/** @name Cholesky factorization
			    @{  */

			/** Auxiliary class to hold the results of a Cholesky factorization of a sparse matrix.
			  *  This implementation does not allow updating/downdating.
			  *
			  *  Usage example:
			  *   \code
			  *     CSparseMatrix  SM(100,100);
			  *     SM.insert_entry(i,j, val); ...
			  *     SM.compressFromTriplet();
			  *
			  *     // Do Cholesky decomposition:
			  *		CSparseMatrix::CholeskyDecomp  CD(SM);
			  *     CD.get_inverse();
			  *     ...
			  *   \endcode
			  *
			  * \note Only the upper triangular part of the input matrix is accessed.
			  * \note This class was initially adapted from "robotvision", by Hauke Strasdat, Steven Lovegrove and Andrew J. Davison. See http://www.openslam.org/robotvision.html
			  * \note This class designed to be "uncopiable".
			  * \sa The main class: CSparseMatrix
			  */
			class BASE_IMPEXP CholeskyDecomp  : public mrpt::utils::CUncopiable
			{
			private:
				css * m_symbolic_structure;
				csn * m_numeric_structure;
				const CSparseMatrix *m_originalSM;  //!< A const reference to the original matrix used to build this decomposition.

			public:
				/** Constructor from a square definite-positive sparse matrix A, which can be use to solve Ax=b
				  *   The actual Cholesky decomposition takes places in this constructor.
				  *  \note Only the upper triangular part of the matrix is accessed.
				  *  \exception std::runtime_error On non-square input matrix.
				  *  \exception mrpt::math::CExceptionNotDefPos On non-definite-positive matrix as input.
				  */
				CholeskyDecomp(const CSparseMatrix &A);

				/** Destructor */
				virtual ~CholeskyDecomp();

				/** Return the L matrix (L*L' = M), as a dense matrix. */
				inline CMatrixDouble get_L() const { CMatrixDouble L; get_L(L); return L; }

				/** Return the L matrix (L*L' = M), as a dense matrix. */
				void get_L(CMatrixDouble &out_L) const;

				/** Return the vector from a back-substitution step that solves: Ux=b   */
				template <class VECTOR>
				inline VECTOR backsub(const VECTOR &b) const { VECTOR res; backsub(b,res); return res; }

				/** Return the vector from a back-substitution step that solves: Ux=b. Vectors can be Eigen::VectorXd or mrpt::math::CVectorDouble   */
				void backsub(const Eigen::VectorXd &b, Eigen::VectorXd &result_x) const;

				/** overload for double pointers which assume the user has reserved the output memory for \a result */
				void backsub(const double *b, double *result, const size_t N) const;

				/** Update the Cholesky factorization from an updated vesion of the original input, square definite-positive sparse matrix.
				  *  NOTE: This new matrix MUST HAVE exactly the same sparse structure than the original one.
				  */
				void update(const CSparseMatrix &new_SM);
			};


			/** @} */

		}; // end class CSparseMatrix

	}
}
#endif
