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
#ifndef CMatrixFixedNumeric_H
#define CMatrixFixedNumeric_H

#include <mrpt/math/CArray.h>
#include <mrpt/math/math_frwds.h>  // Fordward declarations
#include <mrpt/math/matrices_metaprogramming.h>  // TMatrixProductType, ...
#include <mrpt/math/CMatrixViews.h>
#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
	namespace math
	{
		using namespace mrpt::system;
		using namespace mrpt::poses;

		/**  A numeric matrix of compile-time fixed size.
		 *   The template can be instanced for data types "float" or "double"
		 *   Virtually all methods have specializations and/or SSE2 optimized implementations, so use this class when time is critical.
		 *
		 *  These matrices have iterators and reverse iterators to access all the elements in the matrix as a sequence, starting from the element (0,0), then row by row, from left to right.
		 *
		 * \note To enable SSE2 optimizations, add the definition "#define MRPT_USE_SSE2" BEFORE including MRPT headers in your code. This is because these optimizations are only applicable to static matrix objects, but not when they are created in dynamic memory.
		 *
		 * \sa CMatrixTemplateNumeric (for dynamic-size matrices)
		 */
		template <typename T,size_t NROWS,size_t NCOLS>
		class CMatrixFixedNumeric    // Must have no "BASE_IMPEXP"
		{
		public:
			typedef CArray<T,NROWS*NCOLS>      array_type;     //!< The underlying array of this matrix

			typedef T              value_type;		//!< The type of the matrix elements
			typedef T&             reference;
			typedef const T&       const_reference;
			typedef std::size_t    size_type;
			typedef std::ptrdiff_t difference_type;

			//! See ops_containers.h
			typedef CMatrixFixedNumeric<T,NROWS,NCOLS> mrpt_autotype;
			DECLARE_MRPT_CONTAINER_TYPES
			DECLARE_MRPT_CONTAINER_IS_MATRIX_FIXED(NROWS,NCOLS)
			// "DECLARE_MRPT_MATRIX_ITERATORS": Not needed by this matrix since we have much simpler iterators...
			DECLARE_COMMON_MATRIX_MEMBERS(T)


			/** @name Iterator-related stuff
				@{ */
			typedef typename array_type::iterator               iterator;       		//!< Iterator
			typedef typename array_type::const_iterator         const_iterator; 		//!< Const iterator
			typedef typename array_type::reverse_iterator       reverse_iterator; 	//!< Reverse iterator
			typedef typename array_type::const_reverse_iterator const_reverse_iterator; //!< Const reverse iterator

			inline iterator 		begin()   { return m_Val.begin(); }
			inline iterator 		end()     { return m_Val.end(); }
			inline const_iterator 	begin() const	{ return m_Val.begin(); }
			inline const_iterator 	end() const		{ return m_Val.end(); }

			inline reverse_iterator       rbegin()   { return m_Val.rbegin(); }
			inline reverse_iterator       rend()     { return m_Val.rend(); }
			inline const_reverse_iterator rbegin() const  { return m_Val.rbegin(); }
			inline const_reverse_iterator rend() const    { return m_Val.rend(); }
			/** @} */

			/** The stored data of the matrix: elements are saved by rows, left to right, from top to bottom.
			  *   (We use a CArray wrapper instead of a plain array so it handles the cases of 0-length arrays without problems.)
			  */
#if MRPT_HAS_SSE2 && defined(MRPT_USE_SSE2)
			MRPT_ALIGN16
#endif
			array_type  m_Val;

		public:
			/** Default constructor, fills the whole matrix with zeros */
			CMatrixFixedNumeric() {
#if defined(_DEBUG) && MRPT_HAS_SSE2 && defined(MRPT_USE_SSE2)
				if ((uintptr_t(m_Val) & 0x0f) != 0 )
					THROW_EXCEPTION("16-unaligned memory!")
#endif
				if (NROWS*NCOLS) ::memset(&m_Val[0],0,sizeof(T)*NROWS*NCOLS);
			}

			/** Constructor which leaves the matrix uninitialized.
			  *  Example of usage: CMatrixFixedNumeric<double,3,2> M(UNINITIALIZED_MATRIX);
			  */
			inline CMatrixFixedNumeric(TConstructorFlags_Matrices constructor_flag)
			{ }


			/** Copy constructor from a matrix of any type  */
			template <typename R>
			inline CMatrixFixedNumeric(const CMatrixTemplateNumeric<R>& M) {
				*this = M;
			}

			/** Copy operator from another matrix of any type  */
			template <class OTHERMATRIX> inline RET_TYPE_ASSERT_MRPTMATRIX(OTHERMATRIX,mrpt_autotype)&
			operator =(const OTHERMATRIX& M) {
				return this->assignMatrix(M);
			}

			/** Copy operator from a CMatrixTemplate<T> matrix (not a CMatrixTemplateNumeric<T>, which is included by the template above!) */
			template <typename R>
			inline mrpt_autotype& operator =(const CMatrixTemplate<R>& M) {
				return this->assignMatrix(M);
			}

			/** Constructor from a given size and a C array. The array length must match cols x row.
			  * \code
			  *  const double numbers[] = {
			  *    1,2,3,
			  *    4,5,6 };
			  *	 CMatrixFixedNumeric<double,3,2>    M(numbers);
			  * \endcode
			  */
			template <typename V, size_t N>
			inline CMatrixFixedNumeric ( V (&theArray)[N] ) {
				loadFromArray(theArray);
			}

			/** Load the matrix from a C array of the proper size - the array length must match cols x row.
			  * \code
			  *  const double numbers[] = {
			  *    1,2,3,
			  *    4,5,6 };
			  *	 CMatrixFixedNumeric<double,3,2>    M(numbers);
			  *   // or
			  *  M.loadFromArray(number);
			  * \endcode
			  */
			template <typename V, size_t N>
			void loadFromArray(V (&theArray)[N] )
			{
				MRPT_COMPILE_TIME_ASSERT(N!=0)
				MRPT_COMPILE_TIME_ASSERT(N==NROWS * NCOLS)
				if (sizeof(V)==sizeof(T))
					::memcpy(&m_Val[0],theArray,sizeof(m_Val));
				else
				for (size_t i=0;i<N;i++)
					m_Val[i] = static_cast<T>(theArray[i]);
			}


			/** Copy constructor from another matrix of a different size: it's explicit so matrices of different sizes are not mixed by mistake.
			  */
			template <size_t N,size_t M>
			explicit CMatrixFixedNumeric(const CMatrixFixedNumeric<T,N,M> &B)
			{
				::memset(&m_Val[0],0,sizeof(m_Val));
				const size_t nr = std::min(NROWS,N);
				const size_t nc = std::min(NCOLS,M);
				for (size_t r=0;r<nr;r++)
					::memcpy(&m_Val[0]+NCOLS*r, &B.m_Val[0]+M*r, sizeof(T)*nc );
			}

			/** Copy constructor from another matrix of a different size and type: it's explicit so matrices of different sizes are not mixed by mistake.
			  */
			template <typename R,size_t N,size_t M>
			explicit CMatrixFixedNumeric(const CMatrixFixedNumeric<R,N,M> &B)
			{
				::memset(&m_Val[0],0,sizeof(m_Val));
				const size_t nr = std::min(NROWS,N);
				const size_t nc = std::min(NCOLS,M);
				for (size_t r=0;r<nr;r++)
					for (size_t c=0;c<nc;c++)
						this->set_unsafe(r,c, B.get_unsafe(r,c) );
			}

			inline CMatrixFixedNumeric( const TPose2D &p)  { mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixFixedNumeric( const TPose3D &p)  { mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixFixedNumeric( const TPose3DQuat &p)  { mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixFixedNumeric( const TPoint2D &p) { mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixFixedNumeric( const TPoint3D &p) { mrpt::math::containerFromPoseOrPoint(*this,p); }

			inline CMatrixFixedNumeric( const mrpt::poses::CPose2D &p)  { mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixFixedNumeric( const mrpt::poses::CPose3D &p)  { mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixFixedNumeric( const mrpt::poses::CPose3DQuat &p)  { mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixFixedNumeric( const mrpt::poses::CPoint2D &p) { mrpt::math::containerFromPoseOrPoint(*this,p); }
			inline CMatrixFixedNumeric( const mrpt::poses::CPoint3D &p) { mrpt::math::containerFromPoseOrPoint(*this,p); }

			/** Get number of rows */
			static inline size_t getRowCount() { return NROWS; }

			/** Get number of columns */
			static inline size_t getColCount() { return NCOLS; }

			/** Get a 2-vector with [NROWS NCOLS] (as in MATLAB command size(x)) */
			static inline const CMatrixTemplateSize & size() {
				static size_t dims_arr[] = {NROWS, NCOLS};
				static const CMatrixTemplateSize dims(dims_arr);
				return dims;
				}

			/** This method has no effects in this class, but raises an exception if the expected size does not match */
			static inline void setSize(const size_t nRows, const size_t nCols) {
				if (nRows!=NROWS || nCols!=NCOLS)
					throw std::logic_error(format("Try to change the size of a %ux%u fixed-sized matrix to %ux%u.",static_cast<unsigned>(NROWS),static_cast<unsigned>(NCOLS),static_cast<unsigned>(nRows),static_cast<unsigned>(nCols)));
			}
			/** This method has no effects in this class, but raises an exception if the expected size does not match */
			static inline void resize(const size_t nRows_times_nCols) {
				if (nRows_times_nCols!=NROWS*NCOLS)
					throw std::logic_error(format("Try to change the size of a %ux%u fixed-sized matrix to %u elements.",static_cast<unsigned>(NROWS),static_cast<unsigned>(NCOLS),static_cast<unsigned>(nRows_times_nCols)));
			}


			/** Make the matrix an identity matrix */
			inline void unit() {
				::memset(&m_Val[0],0,sizeof(m_Val));
				for (size_t i=0;i<NROWS * NCOLS;i+=(NROWS+1))
					m_Val[i] = 1;
			}

			/** Make the matrix an identity matrix - this is a shortcut for unit(). */
			inline void eye() { this->unit(); }

			/** Set all elements to zero */
			inline void zeros() {
				::memset(&m_Val[0],0,sizeof(m_Val));
			}

			/** Set all elements to zero */
			inline void zeros(size_t N, size_t M) {
				ASSERT_(N==NROWS && M==NCOLS);
				this->zeros();
			}

			/** Read-only access to one element (Use with caution, bounds are not checked!) */
			inline T get_unsafe(const size_t row, const size_t col) const {
#ifdef _DEBUG
				if (row >= NROWS || col >= NCOLS)
					THROW_EXCEPTION( format("Indexes (%lu,%lu) out of range. Matrix is %lux%lu",static_cast<unsigned long>(row),static_cast<unsigned long>(col),static_cast<unsigned long>(NROWS),static_cast<unsigned long>(NCOLS)) );
#endif
				return m_Val[NCOLS*row+col];
			}

			/** Reference access to one element (Use with caution, bounds are not checked!) */
			inline T& get_unsafe(const size_t row, const size_t col) {
#ifdef _DEBUG
				if (row >= NROWS || col >= NCOLS)
					THROW_EXCEPTION( format("Indexes (%lu,%lu) out of range. Matrix is %lux%lu",static_cast<unsigned long>(row),static_cast<unsigned long>(col),static_cast<unsigned long>(NROWS),static_cast<unsigned long>(NCOLS)) );
#endif
				return m_Val[NCOLS*row+col];
			}

			/** Sets an element  (Use with caution, bounds are not checked!) */
			inline void set_unsafe(const size_t row, const size_t col, const T val) {
#ifdef _DEBUG
				if (row >= NROWS || col >= NCOLS)
					THROW_EXCEPTION( format("Indexes (%lu,%lu) out of range. Matrix is %lux%lu",static_cast<unsigned long>(row),static_cast<unsigned long>(col),static_cast<unsigned long>(NROWS),static_cast<unsigned long>(NCOLS)) );
#endif
				m_Val[NCOLS*row+col] = val;
			}

			/** Subscript operator to get/set individual elements
				*/
			inline T& operator () (const size_t row, const size_t col)
			{
		#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
				if (row >= NROWS || col >= NCOLS)
					THROW_EXCEPTION( format("Indexes (%lu,%lu) out of range. Matrix is %lux%lu",static_cast<unsigned long>(row),static_cast<unsigned long>(col),static_cast<unsigned long>(NROWS),static_cast<unsigned long>(NCOLS)) );
		#endif
				return m_Val[NCOLS*row+col];
			}

			/** Subscript operator to get/set individual elements
				*/
			inline T operator () (const size_t row, const size_t col) const
			{
		#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
				if (row >= NROWS || col >= NCOLS)
					THROW_EXCEPTION( format("Indexes (%lu,%lu) out of range. Matrix is %lux%lu",static_cast<unsigned long>(row),static_cast<unsigned long>(col),static_cast<unsigned long>(NROWS),static_cast<unsigned long>(NCOLS)) );
		#endif
				return m_Val[NCOLS*row+col];
			}

			/** Read a matrix from a string in Matlab-like format, for example "[1 0 2; 0 4 -1]"
			  *  The string must start with '[' and end with ']'. Rows are separated by semicolons ';' and
			  *  columns in each row by one or more whitespaces ' ' or tabs.
			  *
			  * This format is also used for CConfigFile::read_matrix.
			  *
			  *  This template method can be instantiated for matrices of the types: int, long, unsinged int, unsigned long, float, double, long double
			  *
			  * \return true on success. false if the string is malformed, or it is of the wrong size.
			  * \sa inMatlabFormat, CConfigFile::read_matrix
			  */
			bool fromMatlabStringFormat(const std::string &s)
			{
				CMatrixTemplate<T>	M;
				if (!M.fromMatlabStringFormat(s)) return false;
				if (M.getColCount()!=NCOLS || M.getRowCount()!=NROWS) return false;
				*this = M;
				return true; // Ok
			}

			/** Copy the upper half of the matrix into the lower half */
			void  force_symmetry() {
				for (size_t i=0;i<NROWS;i++)
					for (size_t j=i+1;j<NCOLS;j++)
						get_unsafe(i,j) = get_unsafe(j,i);
			}

			/** @name Import/export as text
				@{ */

			/** Load matrix from a text file, compatible with MATLAB text format.
			  *  Lines starting with '%' or '#' are interpreted as comments and ignored.
			  * \sa saveToTextFile, fromMatlabStringFormat
			  */
			void  loadFromTextFile(const std::string &file)
			{
				// This matrix is NROWS x NCOLS
				std::ifstream	f(file.c_str());
				if (f.fail()) THROW_EXCEPTION_CUSTOM_MSG1("loadFromTextFile: can't open file:'%s'",file.c_str());

				std::string		str;
				std::vector<double>	fil(512);

				const char	*ptr;
				char		*ptrEnd;
				size_t	i;
				size_t	nRows = 0;

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

						// "i": # of columns:
						if (i!=NCOLS) THROW_EXCEPTION(format("The matrix in the text file does not match fixed matrix size %ux%u",static_cast<unsigned>(NROWS),static_cast<unsigned>(NCOLS)));
						if (nRows>=NROWS) THROW_EXCEPTION(format("The matrix in the text file does not match fixed matrix size %ux%u",static_cast<unsigned>(NROWS),static_cast<unsigned>(NCOLS)));

						// Copy row to matrix:
						for (size_t j=0;j<NCOLS;j++)
							get_unsafe(nRows,j) = static_cast<T>(fil[j]);

						nRows++;
					} // end if fgets
				} // end while not feof

				// Report error as exception
				if (!nRows)
					THROW_EXCEPTION("loadFromTextFile: Error loading from text file");
			}

			/** @} */

			void multiplyColumnByScalar(
				size_t c,
				T scalar)
			{
				ASSERT_( c < NCOLS );
				T *c1 = &m_Val[0]+c;
				for(unsigned int k = 0; k < NCOLS; k++)
				{
					*c1 *= scalar;
					c1 += NCOLS;
				}
			}

			inline void swapRows(size_t i1,size_t i2)
			{
				ASSERT_( i1 < NROWS && i2 < NROWS );
				T  tmprow[NCOLS];
				::memcpy(tmprow, &m_Val[0]+i1*NCOLS, sizeof(tmprow));
				::memcpy(&m_Val[0]+i1*NCOLS,&m_Val[0]+i2*NCOLS, sizeof(tmprow));
				::memcpy(&m_Val[0]+i2*NCOLS,tmprow, sizeof(tmprow));
			}

			void swapCols(size_t i1,size_t i2)
			{
				ASSERT_( i1 < NCOLS && i2 < NCOLS );
				T *c1 = &m_Val[0]+i1, *c2 = &m_Val[0]+i2;

				for( unsigned int k = 0; k < NROWS; k++)
				{
					T aux = *c1;
					*c1 = *c2;
					*c2 = aux;

					c1 += NCOLS;
					c2 += NCOLS;
				}
			}

			template<size_t N> inline void ASSERT_ENOUGHROOM(size_t r,size_t c) const	{
				#if defined(_DEBUG)||(MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
					ASSERT_((r>=N)&&(r<NROWS-N)&&(c>=N)&&(c<NCOLS-N));
				#endif
			}

			template<size_t N,typename ReturnType> inline ReturnType getVicinity(size_t c,size_t r) const	{
				return detail::getVicinity<CMatrixFixedNumeric<T,NROWS,NCOLS>,T,ReturnType,N>::get(c,r,*this);
			}

		}; // end of class definition ------------------------------

		/** @name Typedefs for common sizes
			@{ */
		typedef CMatrixFixedNumeric<double,2,2> CMatrixDouble22;
		typedef CMatrixFixedNumeric<double,2,3> CMatrixDouble23;
		typedef CMatrixFixedNumeric<double,3,2> CMatrixDouble32;
		typedef CMatrixFixedNumeric<double,3,3> CMatrixDouble33;
		typedef CMatrixFixedNumeric<double,4,4> CMatrixDouble44;
		typedef CMatrixFixedNumeric<double,6,6> CMatrixDouble66;
		typedef CMatrixFixedNumeric<double,7,7> CMatrixDouble77;
		typedef CMatrixFixedNumeric<double,1,3> CMatrixDouble13;
		typedef CMatrixFixedNumeric<double,3,1> CMatrixDouble31;
		typedef CMatrixFixedNumeric<double,1,2> CMatrixDouble12;
		typedef CMatrixFixedNumeric<double,2,1> CMatrixDouble21;
		typedef CMatrixFixedNumeric<double,6,1> CMatrixDouble61;
		typedef CMatrixFixedNumeric<double,1,6> CMatrixDouble16;
		typedef CMatrixFixedNumeric<double,7,1> CMatrixDouble71;
		typedef CMatrixFixedNumeric<double,1,7> CMatrixDouble17;
		typedef CMatrixFixedNumeric<double,5,1> CMatrixDouble51;
		typedef CMatrixFixedNumeric<double,1,5> CMatrixDouble15;

		typedef CMatrixFixedNumeric<float,2,2> CMatrixFloat22;
		typedef CMatrixFixedNumeric<float,2,3> CMatrixFloat23;
		typedef CMatrixFixedNumeric<float,3,2> CMatrixFloat32;
		typedef CMatrixFixedNumeric<float,3,3> CMatrixFloat33;
		typedef CMatrixFixedNumeric<float,4,4> CMatrixFloat44;
		typedef CMatrixFixedNumeric<float,6,6> CMatrixFloat66;
		typedef CMatrixFixedNumeric<float,7,7> CMatrixFloat77;
		typedef CMatrixFixedNumeric<float,1,3> CMatrixFloat13;
		typedef CMatrixFixedNumeric<float,3,1> CMatrixFloat31;
		typedef CMatrixFixedNumeric<float,1,2> CMatrixFloat12;
		typedef CMatrixFixedNumeric<float,2,1> CMatrixFloat21;
		typedef CMatrixFixedNumeric<float,6,1> CMatrixFloat61;
		typedef CMatrixFixedNumeric<float,1,6> CMatrixFloat16;
		typedef CMatrixFixedNumeric<float,7,1> CMatrixFloat71;
		typedef CMatrixFixedNumeric<float,1,7> CMatrixFloat17;
		typedef CMatrixFixedNumeric<float,5,1> CMatrixFloat51;
		typedef CMatrixFixedNumeric<float,1,5> CMatrixFloat15;
		/**  @} */


	namespace detail	{
		/**
		  * Vicinity traits class specialization for fixed size matrices.
		  */
		template<typename T,size_t D> class VicinityTraits<CMatrixFixedNumeric<T,D,D> >	{
		public:
			inline static void initialize(CMatrixFixedNumeric<T,D,D> &mat,size_t N)	{
				ASSERT_(N==D);
			}
			inline static void insertInContainer(CMatrixFixedNumeric<T,D,D> &mat,size_t r,size_t c,const T &t)	{
				mat.get_unsafe(r,c)=t;
			}
		};
	}	//End of detail namespace.


	} // End of namespace

	namespace utils
	{
		// Extensions to mrpt::utils::TTypeName for matrices:
		template<typename T,size_t N,size_t M> struct TTypeName <mrpt::math::CMatrixFixedNumeric<T,N,M> > {
			static std::string get() { return mrpt::format("CMatrixFixedNumeric<%s,%u,%u>",TTypeName<T>::get().c_str(),(unsigned int)N,(unsigned int)M); }
		};
	}

} // End of namespace

#endif
