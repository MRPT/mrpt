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
#ifndef CMatrixTemplateNumeric_H
#define CMatrixTemplateNumeric_H

#include <mrpt/math/CMatrixTemplate.h>
#include <mrpt/math/matrix_iterators.h>
#include <mrpt/math/matrices_metaprogramming.h>  // TMatrixProductType, ...
#include <mrpt/math/CMatrixViews.h>
#include <mrpt/utils/CSerializable.h>

#include <mrpt/system/os.h>
#include <cmath>
#include <limits>

namespace mrpt
{
	namespace poses
	{
		class CPose2D;
		class CPose3D;
		class CPoint2D;
		class CPoint3D;
	}
		namespace math
		{
			using namespace mrpt::system;


		/**  This template class extends the class "CMatrixTemplate" with many common operations with numerical matrixes.
		 *
		 *   The template can be instanced for data types: float, double, long double
		 *
		 *   The following operators have been implemented:
		<P>
		<TABLE cellSpacing=1 cellPadding=1 width="75%" align=center border=1>
		  <TR>
			<TD align=center width="30%"><b>Implemented Operators</b></TD>
			<TD width="70%"><center><b>Meaning</b></center></TD></TR>
		  <TR>
			<TD align=center>x=M(i,j)<br>M(i,j)=x</TD>
			<TD>This () operator is used to access/change the element at i'th row, j'th column. First index is 0.</TD></TR>
		  <TR>
			<TD align=center>!M</TD>
			<TD>The matrix inverse M<sup>-1</sup></TD></TR>
		  <TR>
			<TD align=center>~M</TD>
			<TD>The matrix transpose M<sup>T</sup></TD></TR>
		  <TR>
			<TD align=center>(M^n)</TD>
			<TD>Power of a matrix: (M*M*M*...M) n times. Use parenthesis with this operator. Use "scalarPow" for the power of individual elements in the matrix. </TD></TR>
		  <TR>
			<TD align=center> M1 = M2</TD>
			<TD>Assignment operator: Copy matrix M2 to M1</TD></TR>
		  <TR>
			<TD align=center>M1 == M2<br>M1 != M2</TD>
			<TD>Logical comparison: Returns true or false if all elements are identical.</TD></TR>
		  <TR>
			<TD align=center>x * M</TD>
			<TD>Scalar multiplication</TD></TR>
		  <TR>
			<TD align=center>M1 * M2</TD>
			<TD>Matrix multiplication, with the usual mathematical meaning</TD></TR>
		  <TR>
			<TD align=center>M1 + M2<br>M1  M2</TD>
			<TD>Matrixes addition and substraction.</TD></TR>
		  <TR>
			<TD align=center>M / x</TD>
			<TD>Scalar division</TD></TR>
		 <TR>
			<TD align=center>M1 / M2</TD>
			<TD>Equivalent to (M1 * M2<sup>-1</sup>)</TD></TR>
		 <TR>
			<TD align=center>stream &lt;&lt; M;</TD>
			<TD>Write to a binary CStream, since this class is CSerializable</TD></TR>
		 <TR>
			<TD align=center>stream &gt;&gt; M;</TD>
			<TD>Read from a binary CStream, since this class is CSerializable</TD></TR>
		</TABLE>
		</P>
		 *  See also this useful methods:
		 *		- SAVE_MATRIX
		 * \sa CMatrixTemplate
		 */
		template <class T>
		class BASE_IMPEXP CMatrixTemplateNumeric : public CMatrixTemplate<T>
		{
		public:
			typedef CMatrixTemplate<T> BASE;
			typedef CMatrixTemplateNumeric<T> mrpt_autotype;
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX
			DECLARE_MRPT_MATRIX_ITERATORS
			DECLARE_COMMON_MATRIX_MEMBERS(T)
						

			inline CMatrixTemplateNumeric( const TPose2D &p)  : CMatrixTemplate<T>( 3,1 )	{ mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixTemplateNumeric( const TPose3D &p)  : CMatrixTemplate<T>( 6,1 )	{ mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixTemplateNumeric( const TPose3DQuat &p)  : CMatrixTemplate<T>( 7,1 )	{ mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixTemplateNumeric( const TPoint2D &p) : CMatrixTemplate<T>( 2,1 )	{ mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixTemplateNumeric( const TPoint3D &p) : CMatrixTemplate<T>( 3,1 )	{ mrpt::math::containerFromPoseOrPoint(*this,p); }

			inline CMatrixTemplateNumeric( const mrpt::poses::CPose2D &p)  : CMatrixTemplate<T>( 3,1 )	{ mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixTemplateNumeric( const mrpt::poses::CPose3D &p)  : CMatrixTemplate<T>( 6,1 )	{ mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixTemplateNumeric( const mrpt::poses::CPose3DQuat &p)  : CMatrixTemplate<T>( 7,1 )	{ mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixTemplateNumeric( const mrpt::poses::CPoint2D &p) : CMatrixTemplate<T>( 2,1 )	{ mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixTemplateNumeric( const mrpt::poses::CPoint3D &p) : CMatrixTemplate<T>( 3,1 )	{ mrpt::math::containerFromPoseOrPoint(*this,p); }

			/** Default constructor, builds a 1x1 matrix */
			CMatrixTemplateNumeric();

			/** Default constructor, builds a 0x0 matrix, with a signature identical to that of the fixed matrix constructor for uninitialized matrix constructor */
			inline CMatrixTemplateNumeric(bool,bool,bool) : CMatrixTemplate<T>( 0,0 )
			{
			}

			/** Constructor */
			CMatrixTemplateNumeric(size_t row, size_t col);

			/** Copy & crop constructor, which copies the given matrix but only up to the given size */
			CMatrixTemplateNumeric(const CMatrixTemplate<T> & m, const size_t cropRowCount, const size_t cropColCount) : CMatrixTemplate<T>(m,cropRowCount,cropColCount)
			{ }

			/** Copy constructor from a fixed-size matrix */
			template <size_t NROWS,size_t NCOLS>
			explicit CMatrixTemplateNumeric(const CMatrixFixedNumeric<T,NROWS,NCOLS> &M ) {
				this->assignMatrix(M);
			}

			/** Copy operator from a fixed-size matrix */
			template <size_t NROWS,size_t NCOLS>
			CMatrixTemplateNumeric& operator =(const CMatrixFixedNumeric<T,NROWS,NCOLS> &M ) {
				return this->assignMatrix(M);
			}

			/** Assignment operator of other types
			*/
			template <class R>
			CMatrixTemplateNumeric<T>& operator = (const CMatrixTemplate<R>& m)
			{
				CMatrixTemplate<T>::realloc( m.getRowCount(), m.getColCount() );

				for (size_t i=0; i < CMatrixTemplate<T>::getRowCount(); i++)
					for (size_t j=0; j < CMatrixTemplate<T>::getColCount(); j++)
						CMatrixTemplate<T>::m_Val[i][j] = static_cast<T>(m.get_unsafe(i,j));
				return *this;
			}

			/** Copy constructor from a matrix of any type */
			template <class R>
			inline CMatrixTemplateNumeric(const CMatrixTemplate<R>& m) {
				*this = m;
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
			CMatrixTemplateNumeric(size_t row, size_t col, V (&theArray)[N] ) : CMatrixTemplate<T>( row, col, theArray )
			{ }

			/** Constructor from a given size and a STL container (std::vector, std::list,...) with the initial values. The vector length must match cols x row.
			  */
			template <typename V>
			CMatrixTemplateNumeric(size_t row, size_t col, const V &theVector ) : CMatrixTemplate<T>( row, col, theVector )
			{ }

			/** Destructor
			  */
			virtual ~CMatrixTemplateNumeric()
			{ }


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
			CMatrixTemplateNumeric& operator = (V (&theArray)[N] )
			{
				CMatrixTemplate<T>::operator = (theArray);
				return *this;
			}

			/** Changes the size of matrix, maintaining its previous content as possible and padding with zeros where applicable.
			  *  setSize and resize are exactly equivalent methods.
			  */
			void setSize(size_t row, size_t col);

			/** Changes the size of matrix, maintaining its previous content as possible and padding with zeros where applicable.
			  *  setSize and resize are exactly equivalent methods.
			  */
			void resize(size_t row, size_t col);

			/** This method just checks has no effects in this class, but raises an exception if the expected size does not match */
			inline void resize(const CMatrixTemplateSize &siz) {
				setSize(siz[0],siz[1]);
			}

			/** Computes the laplacian of the matrix, useful for graph matrixes.
			 */
			void laplacian( CMatrixTemplateNumeric<T> &ret ) const;

			/** Computes the SVD (Singular Value Decomposition) of the matrix.
			  *  If "this" matrix is named A with dimensions M x N, this method computes: <br>
			  *			A = U * W * V' <br>
			  * <br>
			  *  , where U is a M x N column orthogonal matrix, W is a diagonal matrix
			  *  containing the singular values, and V is a NxN matrix. <br>
			  * This method returns the U matrix, the N elements in the diagonal of W as a vector,
			  *  and the matrix V, NOT TRANSPOSED.
			  */
			void  svd(CMatrixTemplateNumeric<T> &U, std::vector<T> &W,CMatrixTemplateNumeric<T> &V) const;

			/** Efficiently computes only the biggest eigenvector of the matrix using the Power Method, and returns it as a column vector.
			  *  The computation time for this method, in a Pentium 4 1.4Ghz is:<br>
			  *         T = 7.0867e-008*n<sup>2</sup> + 1.9191e-005*n + 0.0017494 seconds  <br>
			  *  where N is the matrix size.
			  */
			CMatrixTemplateNumeric<T>  largestEigenvector(
				T			resolution = 0.01f,
				size_t			maxIterations = 6,
				int			*out_Iterations = NULL,
				float		*out_estimatedResolution = NULL ) const;

			/** combined power and assignment operator
			*/
			CMatrixTemplateNumeric<T>&  operator ^= (const unsigned int& pow);

			/** Scalar power of all elements to a given power, this is diferent of ^ operator.
				*/
			void  scalarPow(T s);

		   /** Set all elements to zero
			*/
		   void  zeros(const size_t row, const size_t col);

		   /** Set all elements to zero
			*/
		   void  zeros();

		   /** Set all elements to one
			*/
		   void  ones(const size_t row, const size_t col);

		   /** Set all elements to one
			*/
		   void  ones();

		   /** Build an unit matrix.
			*/
		   void  unit (const size_t row);

		   /** Build an unit matrix - this is a shortcut for unit(). */
		   inline void eye(const size_t size) { this->unit(size); }

		   /** Build an unit matrix.
			*/
		   void  unit();

		   /** Build an unit matrix - this is a shortcut for unit(). */
		   inline void eye() { this->unit(); }

			/** Solve the matrix as linear equations system.
			*/
			CMatrixTemplateNumeric<T>  solve (const CMatrixTemplateNumeric<T>& v) const;

			/** Computes the adjunt of matrix.
			*/
			CMatrixTemplateNumeric<T>  adj() const;

			/** Computes the rank of the matrix using a slight variation of Gauss method.
			  */
			//size_t rank(T eps=0.0) const;

			/** Computes the cofact.
			*/
			T  cofact (size_t row, size_t col) const;

			/** Computes the cond.
			*/
			T	 cond();

			/** Checks for matrix type
			  */
			bool  isDiagonal() const;

			/** Checks for matrix type
			  */
			bool  isScalar() const;

			/** Checks for matrix type
			  */
			bool  isUnit() const;

			/** Checks for matrix type
			  */
			bool  isNull() const;

			/** Checks for matrix type
			  */
			bool  isSymmetric() const;

			/** Checks for matrix type
			  */
			bool  isSkewSymmetric() const;

			/** Checks for matrix type
			  */
			bool  isUpperTriangular() const;

			/** Checks for matrix type
			  */
			bool  isLowerTriangular() const;

			/** Round towards minus infinity modifying the matrix
			  * (by AJOGD @ JAN-2007)
			  */
			void  matrix_floor();

			/** Round towards minus infinity
			  * (by AJOGD @ JAN-2007)
			  */
			void  matrix_floor(CMatrixTemplateNumeric<T> &out);

			/** Round towards plus infinity
			  * (by AJOGD @ JAN-2007)
			  */
			void  matrix_ceil();

			/** Finds the maximum value in the matrix, and returns its position.
			  * (by AJOGD @ JAN-2007)
			  */
			void  find_index_max_value(size_t &umax, size_t &vmax, T &max_val) const;

			/** Finds the maximum value in the diagonal of the matrix.
			  */
			T  maximumDiagonal() const;

			/** Finds the minimum value in the matrix, and returns its position.
			  * (by AJOGD @ JAN-2007)
			  */
			void  find_index_min_value(size_t  &umin, size_t  &vmin, T &min_val) const;

			/** Force symmetry in the matrix
			  * (by AJOGD @ JAN-2007)
			  */
			void  force_symmetry();

			/** Computes a row with the mean values of each column in the matrix.
			  * \sa meanAndStdAll
			  */
			void  mean( std::vector<T> &outMeanVector ) const;

			/** Computes a row with the mean values of each column in the matrix and the associated vector with the standard deviation of each column.
			  * \sa mean,meanAndStdAll
			  */
			void  meanAndStd(
				std::vector<T> &outMeanVector,
				std::vector<T> &outStdVector ) const;

			/** Computes the mean and standard deviation of all the elements in the matrix as a whole.
			  * \sa mean,meanAndStd
			  */
			void  meanAndStdAll(
				T &outMean,
				T &outStd )  const;

			void  asCol(CMatrixTemplateNumeric<T>	&aux) const;

			void  asRow(CMatrixTemplateNumeric<T>	&aux) const;

			/** Finds elements whose values are a given number of times above (or below) the mean, in 1D Mahalanobis distance.
			  *  This returns two lists with the "row" and "column" indexes (i,j) of those elements m[i][j] such as:
			  *    m[i][j] > mean(matrix) + stdTimes·std(matrix)
			  *  The elements below the threshold
			  *    mean(matrix) - stdTimes·std(matrix)
			  *  can also be obtained setting "below" to "true".
			  */
			void  findElementsPassingMahalanobisThreshold(
				double					stdTimes,
				std::vector<size_t>		&rowIndexes,
				std::vector<size_t>		&colIndexes,
				bool					below = false ) const;

			/** Returns the sum of a given part of the matrix.
			  *  The default value (std::numeric_limits<size_t>::max()) for the last column/row means to sum up to the last column/row.
			  * \sa sumAll
			  */
			T  sum(
				size_t firstRow = 0,
				size_t firstCol = 0,
				size_t lastRow  = std::numeric_limits<size_t>::max(),
				size_t lastCol  = std::numeric_limits<size_t>::max() ) const;

			/** Computes:  R = H * C * H^t , where H is this matrix.
			  *
			  */
			void  multiplyByMatrixAndByTransposeNonSymmetric(
				const CMatrixTemplateNumeric<T>		&C,
				CMatrixTemplateNumeric<T>			&R,
				bool								accumOnOutput = false,
				bool								substractInsteadOfSum = false
				) const;

		}; // end of class definition


		/** Declares a matrix of float numbers (non serializable).
		  *  For a serializable version, use math::CMatrix
		  *  \sa CMatrixDouble, CMatrix, CMatrixD
		  */
		typedef CMatrixTemplateNumeric<float> CMatrixFloat;

		/** Declares a matrix of double numbers (non serializable).
		  *  For a serializable version, use math::CMatrixD
		  *  \sa CMatrixFloat, CMatrix, CMatrixD
		  */
		typedef CMatrixTemplateNumeric<double> CMatrixDouble;

		/** Declares a matrix of unsigned ints (non serializable).
		  *  \sa CMatrixDouble, CMatrixFloat
		  */
		typedef CMatrixTemplateNumeric<unsigned int> CMatrixUInt;

		/** Declares a matrix of booleans (non serializable).
		  *  \sa CMatrixDouble, CMatrixFloat, CMatrixB
		  */
		typedef CMatrixTemplate<bool> CMatrixBool;

#ifdef HAVE_LONG_DOUBLE
		/** Declares a matrix of "long doubles" (non serializable), or of "doubles" if the compiler does not support "long double".
		  *  \sa CMatrixDouble, CMatrixFloat
		  */
		typedef CMatrixTemplateNumeric<long double> CMatrixLongDouble;
#else
		/** Declares a matrix of "long doubles" (non serializable), or of "doubles" if the compiler does not support "long double".
		  *  \sa CMatrixDouble, CMatrixFloat
		  */
		typedef CMatrixTemplateNumeric<double> CMatrixLongDouble;
#endif

		namespace detail
		{
			// Specializations of "isMatrixTypeResizable":
			template <> inline bool isMatrixTypeResizable<CMatrixFloat>(const CMatrixFloat&) { return true; }
			template <> inline bool isMatrixTypeResizable<CMatrixDouble>(const CMatrixDouble&) { return true; }
			template <> inline bool isMatrixTypeResizable<CMatrixLongDouble>(const CMatrixLongDouble&) { return true; }
		}

	namespace detail	{
		/**
		  * Vicinity traits class specialization for fixed size matrices.
		  */
		template<typename T> class VicinityTraits<CMatrixTemplateNumeric<T> >	{
		public:
			inline static void initialize(CMatrixTemplateNumeric<T> &mat,size_t N)	{
				mat.setSize(N,N);
				mat.fill(0);
			}
			inline static void insertInContainer(CMatrixTemplateNumeric<T> &mat,size_t r,size_t c,const T &t)	{
				mat.get_unsafe(r,c)=t;
			}
		};
	}	//End of detail namespace.

	} // End of namespace


	namespace utils
	{
		// Extensions to mrpt::utils::TTypeName for matrices:
		template<typename T> struct TTypeName <mrpt::math::CMatrixTemplateNumeric<T> > {
			static std::string get() { return std::string("CMatrixTemplateNumeric<")+ std::string( TTypeName<T>::get() )+std::string(">"); }
		};
	}

} // End of namespace

#endif
