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
#ifndef mrpt_math_forwddecls_H
#define mrpt_math_forwddecls_H

#include <mrpt/config.h>
#include <mrpt/base/link_pragmas.h>
#include <mrpt/utils/types.h>
#include <cmath>
#include <cstdlib>
#include <algorithm>

/*! \file math_frwds.h
  * Forward declarations of all mrpt::math classes related to vectors, arrays and matrices.
  * Many of the function implementations are in ops_matrices.h, others in ops_containers.h
  */

namespace mrpt
{
	namespace utils
	{
		class BASE_IMPEXP CStream;
		template<class T> inline T square(const T x);
	}

	namespace poses
	{
		class CPoint2D;
		class CPoint3D;
		class CPose2D;
		class CPose3D;
		class CPose3DQuat;
	}

	namespace math
	{
		struct TPoint2D;
		struct TPoint3D;
		struct TPose2D;
		struct TPose3D;
		struct TPose3DQuat;

		/*! Selection of the number format in CMatrixTemplate::saveToTextFile
		  */
		enum TMatrixTextFileFormat
		{
			MATRIX_FORMAT_ENG = 0,   //!< engineering format '%e'
			MATRIX_FORMAT_FIXED = 1, //!< fixed floating point '%f'
			MATRIX_FORMAT_INT = 2	 //!< intergers '%i'
		};

		/** For usage in one of the constructors of CMatrixFixedNumeric or CMatrixTemplate (and derived classes), if it's not required
		     to fill it with zeros at the constructor to save time. */
		#define UNINITIALIZED_MATRIX  false,false,false


		// ---------------- Forward declarations: Classes ----------------
		template <class T> class CVectorTemplate;

		template <class T> class CMatrixTemplate;
		template <class T> class BASE_IMPEXP CMatrixTemplateNumeric;
		template <typename T,size_t NROWS,size_t NCOLS> class CMatrixFixedNumeric;
		template<typename MATRIXTYPE> class CArbitrarySubmatrixView;

		template <typename T, std::size_t N> class CArray;
		template <typename T> class CArray<T,0>;
		template <typename T, std::size_t N> class CArrayPOD;
		template <typename T, std::size_t N> class CArrayNumeric;

		class BASE_IMPEXP CMatrix;
		class BASE_IMPEXP CMatrixD;

		// Functions shared by all kind of matrices/vectors/array
		// These are member functions that invoke functions in mrpt::math, most of them in ops_containers.h
		// ------------------------------------------------------------------------------------------------
#define DECLARE_COMMON_CONTAINERS_MEMBERS(NUMTYPE) \
		/*! @name Methods common to all containers  */ \
		/*! Count how many elements in a container are non-zero. */ \
		inline size_t countNonZero() const { return mrpt::math::countNonZero(*this); } \
		/*! Fill all the elements with a given value */ \
		inline void fill(const NUMTYPE &val) { std::fill(mrpt_autotype::begin(),mrpt_autotype::end(),val); } \
		/*! Finds the maximum value (and the corresponding zero-based index). */ \
		inline NUMTYPE maximum(size_t *maxIndex = NULL) const { return mrpt::math::maximum(*this,maxIndex); } \
		/*! Finds the minimum value (and the corresponding zero-based index). */ \
		inline NUMTYPE minimum(size_t *minIndex = NULL) const { return mrpt::math::minimum(*this,minIndex ); } \
		/*! Minimum and maximum of a vector at once */ \
		inline void minimum_maximum(NUMTYPE &out_min, NUMTYPE &out_max, \
			size_t *minIndex = NULL, size_t *maxIndex = NULL) const \
		{ mrpt::math::minimum_maximum(*this,out_min,out_max,minIndex,maxIndex); } \
		/*! Compute the norm-infinite of a vector ($f[ ||\mathbf{v}||_\infnty $f]), ie the maximum absolute value of the elements. */ \
		inline NUMTYPE norm_inf(size_t *maxIndex = NULL) const { return mrpt::math::norm_inf(*this,maxIndex); } \
		/*! Sum all the elements, returning a value of the same type than the container  */ \
		inline NUMTYPE sumAll() const { return mrpt::math::sum(*this); } \
		/*! Sum all the elements, returning a value of a custom type (this could be used to avoid overflow with containers of integers) */ \
		template <typename RET_TYPE> inline RET_TYPE sumAllRetType() const { return mrpt::math::sumRetType<mrpt_autotype,NUMTYPE>(*this); } \
		/*! Cumsum of all the elements, saving in an output container */ \
		template <class CONTAINEROUT> inline void cumsum(CONTAINEROUT &out) const { mrpt::math::cumsum(*this,out); } \
		/*! Cumsum of all the elements, returning a container of the given type, for example: CMatrixDouble m = mat.cumsum<CMatrixDouble>(); */ \
		template <class CONTAINEROUT> inline CONTAINEROUT cumsum() const { CONTAINEROUT o; mrpt::math::cumsum(*this,o); return o;} \
		/** Counts the number of elements that appear in both containers (comparison through the == operator). It is assumed that no repeated elements appear within each of the containers.  */ \
		template <class CONTAINER2> inline size_t countCommonElements(const CONTAINER2 &b) const { return mrpt::math::countCommonElements(*this,b); } \
		/** Computes the normalized or normal histogram of a sequence of numbers given the number of bins and the limits. In any case this is a "linear" histogram, i.e. for matrices, all the elements are taken as if they were a plain sequence, not taking into account they were in columns or rows. */ \
		inline std::vector<double> histogram(double limit_min,double limit_max,size_t number_bins,bool do_normalization = false ) const { \
			return mrpt::math::histogram(*this,limit_min,limit_max,number_bins,do_normalization); } \
		/*! Applies a generic operation "T func(T val)" to all the elements. */ \
		template <class F> mrpt_autotype & applyToAllElements( NUMTYPE (*function)(NUMTYPE) ) { \
			std::for_each(this->begin(),this->end(), function); \
			return *this; } \
		inline mrpt_autotype & Sqrt() /*!< Applies sqrt() to all the elements  */ { for (typename mrpt_autotype::iterator it=this->begin();it!=this->end();++it) *it = ::sqrt(*it); return *this; }  \
		inline mrpt_autotype & Abs()  /*!< Applies abs() to all the elements  */ { for (typename mrpt_autotype::iterator it=this->begin();it!=this->end();++it) *it = ::fabs(*it); return *this; }  \
		inline mrpt_autotype & Square()  /*!< Applies square() to all the elements */ { for (typename mrpt_autotype::iterator it=this->begin();it!=this->end();++it) *it = mrpt::utils::square(*it); return *this; }  \
		inline mrpt_autotype & Exp()  /*!< Applies exp() to all the elements */ { for (typename mrpt_autotype::iterator it=this->begin();it!=this->end();++it) *it = ::exp(*it); return *this; }  \
		inline mrpt_autotype & Log()  /*!< Applies log() to all the elements */ { for (typename mrpt_autotype::iterator it=this->begin();it!=this->end();++it) *it = ::log(*it); return *this; }  \
		/*! Return the square norm of the container (for matrices, as if it were a vector with all its elements).  */ \
		inline NUMTYPE squareNorm() const { return mrpt::math::squareNorm(*this); } \
		/*! Return the norm of the container (for matrices, as if it were a vector with all its elements).  */ \
		inline NUMTYPE norm() const { return mrpt::math::norm(*this); } \
		/*! Return the mean of the container (for matrices, as if it were a vector with all its elements).  */ \
		inline double mean() const { return mrpt::math::mean(*this); } \
		inline void adjustRange(const NUMTYPE min_val,const NUMTYPE max_val) /*!< Normalize all the elements such as they are in the given range */ { mrpt::math::adjustRange(*this,min_val,max_val); } \
		inline void normalize(const NUMTYPE min_val=0,const NUMTYPE max_val=1) /*!< Normalize all the elements such as they are in the given range */ { mrpt::math::adjustRange(*this,min_val,max_val); } \
		inline double std(bool unbiased=true) const /*! Return the standard deviation of all the elements in the container. */ { return mrpt::math::stddev(*this); } \
		void meanAndStd(double &out_mean, double &out_std, bool unbiased = true) /*! Return the mean and standard deviation of all the elements in the container. */ { \
			return mrpt::math::meanAndStd(*this,out_mean,out_std,unbiased); } \
		/*! Load this matrix or vector from a text file which can contain a column or row vector or a matrix, then assigns to this container all those elements in order (for example, a file with a 10x2 matrix can be loaded into a 20-length array) \exception std::exception On any error. */ \
		inline void loadFromTextFileAsVector(const std::string &file) {  mrpt::math::detail::loadFromTextFileAsVector(*this,file); } \
		/*! Save this matrix or vector to a text file as a row (or optionally as a vector), independently of this containers actually being a vector, an array or a matrix. \exception std::exception On any error. */ \
		inline void saveToTextFileAsVector(const std::string &file, mrpt::math::TMatrixTextFileFormat fileFormat = mrpt::math::MATRIX_FORMAT_ENG ,bool asColumnVector=false) const {  mrpt::math::detail::saveToTextFileAsVector(*this,file,fileFormat ,asColumnVector); }


		//Some operations could be made more efficient if inside each class:
		//operator-() const
		//  Already implemented in ops_containers.h:  +=, -=
#define DECLARE_COMMON_MATRIX_MEMBERS(NUMTYPE) \
		DECLARE_COMMON_CONTAINERS_MEMBERS(NUMTYPE) \
		/*! @name Methods common to all matrices */ \
		/* ========= sum / substract functions: =========  */ \
		inline mrpt_autotype operator+()	{\
			return *this;\
		}\
		inline mrpt_autotype operator-() const	{\
			mrpt_autotype res(*this);\
			for (size_t i=0;i<mrpt_autotype::getRowCount();++i) for (size_t j=0;j<mrpt_autotype::getColCount();++j) res.set_unsafe(i,j,-res.get_unsafe(i,j));\
			return res;\
		}\
		mrpt_autotype &operator+=(const NUMTYPE n) /*!< Sum a scalar */	{\
			for (size_t i=0;i<mrpt_autotype::getRowCount();++i) for (size_t j=0;j<mrpt_autotype::getColCount();++j) mrpt_autotype::get_unsafe(i,j)+=n;\
			return *this;\
		}\
		mrpt_autotype &operator-=(const NUMTYPE n) /*!< Substract a scalar */	{\
			for (size_t i=0;i<mrpt_autotype::getRowCount();++i) for (size_t j=0;j<mrpt_autotype::getColCount();++j) mrpt_autotype::get_unsafe(i,j)-=n;\
			return *this;\
		}\
		mrpt_autotype &operator*=(const NUMTYPE n) /*!< Multiply by scalar */	{\
			for (size_t i=0;i<mrpt_autotype::getRowCount();++i) for (size_t j=0;j<mrpt_autotype::getColCount();++j) mrpt_autotype::get_unsafe(i,j)*=n;\
			return *this;\
		}\
		inline mrpt_autotype& operator/=(const NUMTYPE n) /*!< Divide by scalar */	{\
			return (*this)*=NUMTYPE(1)/n; } \
		template<typename OTHERMATRIX> \
		inline RET_VOID_ASSERT_MRPTMATRIX(OTHERMATRIX) operator*=(const OTHERMATRIX &m)	{\
			mrpt::math::detail::multiply_AB(*this,m,*this); } \
		template<typename OTHERMATRIX> mrpt_autotype &add_At(const OTHERMATRIX &m)	{\
			ASSERT_((m.getRowCount()==mrpt_autotype::getColCount())&&(m.getColCount()==mrpt_autotype::getRowCount()));\
			for (size_t i=0;i<mrpt_autotype::getRowCount();++i) for (size_t j=0;j<mrpt_autotype::getColCount();++j) mrpt_autotype::get_unsafe(i,j)+=m.get_unsafe(j,i);\
			return *this;\
		}\
		template<typename OTHERMATRIX> mrpt_autotype &add_Ac(const OTHERMATRIX &m,const NUMTYPE c)	{\
			ASSERT_((m.getRowCount()==mrpt_autotype::getRowCount())&&(m.getColCount()==mrpt_autotype::getColCount()));\
			for (size_t i=0;i<mrpt_autotype::getRowCount();++i) for (size_t j=0;j<mrpt_autotype::getColCount();++j) mrpt_autotype::get_unsafe(i,j)+=c*m.get_unsafe(i,j);\
			return *this;\
		}\
		template<typename OTHERMATRIX> mrpt_autotype &substract_At(const OTHERMATRIX &m)	{\
			ASSERT_((m.getRowCount()==mrpt_autotype::getColCount())&&(m.getColCount()==mrpt_autotype::getRowCount()));\
			for (size_t i=0;i<mrpt_autotype::getRowCount();++i) for (size_t j=0;j<mrpt_autotype::getColCount();++j) mrpt_autotype::get_unsafe(i,j)-=m.get_unsafe(j,i);\
			return *this;\
		} \
		/*! Substract n (integer) times A to this matrix: this -= A * n  */ \
		template<typename OTHERMATRIX> mrpt_autotype &  substract_An(const OTHERMATRIX& m, const size_t n)	{\
			ASSERT_((m.getRowCount()==mrpt_autotype::getRowCount())&&(m.getColCount()==mrpt_autotype::getColCount()));\
			for (size_t i=0;i<mrpt_autotype::getRowCount();++i) for (size_t j=0;j<mrpt_autotype::getColCount();++j) mrpt_autotype::get_unsafe(i,j)-=n*m.get_unsafe(i,j);\
			return *this;\
		}\
		/*! Substract c (scalar) times A to this matrix: this -= A * c  */ \
		template<typename OTHERMATRIX> mrpt_autotype &substract_Ac(const OTHERMATRIX &m,const NUMTYPE c)	{\
			ASSERT_((m.getRowCount()==mrpt_autotype::getRowCount())&&(m.getColCount()==mrpt_autotype::getColCount()));\
			for (size_t i=0;i<mrpt_autotype::getRowCount();++i) for (size_t j=0;j<mrpt_autotype::getColCount();++j) mrpt_autotype::get_unsafe(i,j)-=c*m.get_unsafe(i,j);\
			return *this;\
		}\
		/*! this += A + A<sup>T</sup>  */ \
		template<typename OTHERMATRIX> mrpt_autotype &add_AAt(const OTHERMATRIX &m)	{\
			ASSERT_((mrpt_autotype::getRowCount()==mrpt_autotype::getColCount())&&(mrpt_autotype::getRowCount()==m.getRowCount())&&(mrpt_autotype::getColCount()==m.getColCount()));\
			for (size_t i=0;i<mrpt_autotype::getRowCount();++i) for (size_t j=0;j<mrpt_autotype::getColCount();++j) mrpt_autotype::get_unsafe(i,j)+=m.get_unsafe(i,j)+m.get_unsafe(j,i);\
			return *this;\
		}\
		/*! this -= A + A<sup>T</sup>  */ \
		template<typename OTHERMATRIX> mrpt_autotype &substract_AAt(const OTHERMATRIX &m)	{\
			ASSERT_((mrpt_autotype::getRowCount()==mrpt_autotype::getColCount())&&(mrpt_autotype::getRowCount()==m.getRowCount())&&(mrpt_autotype::getColCount()==m.getColCount()));\
			for (size_t i=0;i<mrpt_autotype::getRowCount();++i) for (size_t j=0;j<mrpt_autotype::getColCount();++j) mrpt_autotype::get_unsafe(i,j)-=m.get_unsafe(i,j)+m.get_unsafe(j,i);\
			return *this;\
		}\
		/* =========  multiply_* functions =========  */ \
		bool empty() const { return this->getColCount()==0 || this->getRowCount()==0; } \
		template <class MATRIX1,class MATRIX2> \
		inline void multiply( const MATRIX1& A, const MATRIX2 &B ) /*!< this = A * B */ { mrpt::math::detail::multiply_AB(A,B,*this); } \
		template <class MATRIX1,class MATRIX2> \
		inline void multiply_AB( const MATRIX1& A, const MATRIX2 &B ) /*!< this = A * B */ { mrpt::math::detail::multiply_AB(A,B,*this); } \
		template <typename MATRIX1,typename MATRIX2> inline void multiply_AtB(const MATRIX1 &A,const MATRIX2 &B) /*!< this=A^t * B */	{ mrpt::math::detail::multiply_AtB(A,B,*this); }	\
		/*! Computes the vector vOut = this * vIn, where "vIn" is a column vector of the appropriate length. */ \
		template<typename OTHERVECTOR1,typename OTHERVECTOR2> \
		inline void multiply_Ab(const OTHERVECTOR1 &vIn,OTHERVECTOR2 &vOut,bool accumToOutput = false) const {\
			mrpt::math::detail::multiply_Ab(*this,vIn,vOut,accumToOutput); } \
		/*! Computes the vector vOut = this<sup>T</sup> * vIn, where "vIn" is a column vector of the appropriate length. */ \
		template<typename OTHERVECTOR1,typename OTHERVECTOR2> \
		inline void multiply_Atb(const OTHERVECTOR1 &vIn,OTHERVECTOR2 &vOut,bool accumToOutput = false) const {\
			mrpt::math::detail::multiply_Atb(*this,vIn,vOut,accumToOutput); } \
		template <typename MAT_C, typename MAT_R> \
		inline void multiply_HCHt(const MAT_C &C,MAT_R &R,bool accumResultInOutput=false,bool allow_submatrix_mult=false) const /*!< R = this * C * this<sup>T</sup> \sa detail::multiply_HCHt */ { \
			mrpt::math::detail::multiply_HCHt(*this,C,R,accumResultInOutput,allow_submatrix_mult); } \
		/*! R = H * C * H<sup>T</sup> (with a vector H and a symmetric matrix C) In fact when H is a vector, multiply_HCHt_scalar and multiply_HtCH_scalar are exactly equivalent */ \
		template <typename MAT_C> NUMTYPE multiply_HCHt_scalar(const MAT_C &C) const { return mrpt::math::detail::multiply_HCHt_scalar(*this,C); } \
		/*! R = H<sup>T</sup> * C * H (with a vector H and a symmetric matrix C) In fact when H is a vector, multiply_HCHt_scalar and multiply_HtCH_scalar are exactly equivalent */ \
		template <typename MAT_C> NUMTYPE multiply_HtCH_scalar(const MAT_C &C) const { return mrpt::math::detail::multiply_HCHt_scalar(*this,C); } \
		/*! this = C * C<sup>T</sup> * f (with a matrix C and a scalar f). */\
		template<typename MAT> inline void multiply_AAt_scalar(const MAT &A,typename MAT::value_type f)	{\
			mrpt::math::detail::multiply_AAt_scalar(A,f,*this);\
		}\
		/*! this = C<sup>T</sup> * C * f (with a matrix C and a scalar f). */\
		template<typename MAT> inline void multiply_AtA_scalar(const MAT &A,typename MAT::value_type f)	{\
			mrpt::math::detail::multiply_AtA_scalar(A,f,*this);\
		}\
		/*! R = H<sup>T</sup> * C * H (with a symmetric matrix C) */ \
		template <typename MAT_C, typename MAT_R> \
		inline void multiply_HtCH(const MAT_C &C,MAT_R &R,bool accumResultInOutput=false,bool allow_submatrix_mult=false) const /*!< R = this<sup>T</sup> * C * this \sa detail::multiply_HtCH */  { \
			mrpt::math::detail::multiply_HtCH(*this,C,R,accumResultInOutput,allow_submatrix_mult); } \
		template <class MAT_A,class MAT_OUT> \
		inline void multiply_subMatrix(const MAT_A &A,MAT_OUT &outResult,const size_t A_cols_offset,const size_t A_rows_offset,const size_t A_col_count) const /*!< outResult = this * A */ { \
			mrpt::math::detail::multiply_subMatrix(*this,A,outResult,A_cols_offset,A_rows_offset,A_col_count); } \
		template <class MAT_A,class MAT_B,class MAT_C> \
		void multiply_ABC(const MAT_A &A, const MAT_B &B, const MAT_C &C) /*!< this = A*B*C  */ { \
			mrpt::math::detail::multiply_ABC(A,B,C, *this); } \
		template <class MAT_A,class MAT_B,class MAT_C> \
		void multiply_ABCt(const MAT_A &A, const MAT_B &B, const MAT_C &C) /*!< this = A*B*(C<sup>T</sup>) */ { \
			mrpt::math::detail::multiply_ABCt(A,B,C, *this); } \
		template <class MAT_A,class MAT_B> \
		inline void multiply_ABt(const MAT_A &A,const MAT_B &B) /*!< this = A * B<sup>T</sup> */ { \
			mrpt::math::detail::multiply_ABt(A,B,*this); } \
		template <class MAT_A> \
		inline void multiply_AAt(const MAT_A &A) /*!< this = A * A<sup>T</sup> */ { \
			mrpt::math::detail::multiply_AAt(A,*this); } \
		template <class MAT_A> \
		inline void multiply_AtA(const MAT_A &A) /*!< this = A<sup>T</sup> * A */ { \
			mrpt::math::detail::multiply_AtA(A,*this); } \
		template <class MAT_A,class MAT_B> \
		inline void multiply_result_is_symmetric(const MAT_A &A,const MAT_B &B) /*!< this = A * B (result is symmetric) */ { \
			mrpt::math::detail::multiply_result_is_symmetric(A,B,*this); } \
		/* =========  Assignment functions : =========  */ \
		/*! Assignment of a matrix from any other kind of matrix */ \
		template<typename OTHERMATRIX> inline mrpt_autotype &assignMatrix(const OTHERMATRIX &m)	{\
			setSize(m.getRowCount(),m.getColCount());\
			for (size_t i=0;i<mrpt_autotype::getRowCount();++i) for (size_t j=0;j<mrpt_autotype::getColCount();++j) mrpt_autotype::set_unsafe(i,j,m.get_unsafe(i,j));\
			return *this;\
		}\
		/*! Assignment of a matrix to another one (of any type!), transposed. */\
		template<typename OTHERMATRIX> inline mrpt_autotype &assignMatrixTransposed(const OTHERMATRIX &m)	{\
			setSize(m.getColCount(),m.getRowCount());\
			for (size_t i=0;i<m.getRowCount();++i) for (size_t j=0;j<m.getColCount();++j) set_unsafe(j,i,m.get_unsafe(i,j));\
			return *this;\
		}\
		/*! Assignment of a matrix from any other kind of matrix */ \
		/*template<typename MATRIX> inline RET_TYPE_ASSERT_MRPTMATRIX(MATRIX,mrpt_autotype)& operator=(const MATRIX &m) { return assignMatrix(m); } */ \
		inline mrpt_autotype & operator=( const TPose2D &p)  { this->setSize(3,1); mrpt::math::containerFromPoseOrPoint(*this,p); return *this; } \
		inline mrpt_autotype & operator=( const TPose3D &p)  { this->setSize(6,1); mrpt::math::containerFromPoseOrPoint(*this,p); return *this;} \
		inline mrpt_autotype & operator=( const TPose3DQuat &p)  { this->setSize(7,1); mrpt::math::containerFromPoseOrPoint(*this,p); return *this;} \
		inline mrpt_autotype & operator=( const TPoint2D &p) { this->setSize(2,1); mrpt::math::containerFromPoseOrPoint(*this,p); return *this;} \
		inline mrpt_autotype & operator=( const TPoint3D &p) { this->setSize(3,1); mrpt::math::containerFromPoseOrPoint(*this,p); return *this;} \
		inline mrpt_autotype & operator=( const mrpt::poses::CPose2D &p)  { this->setSize(3,1); mrpt::math::containerFromPoseOrPoint(*this,p); return *this;} \
		inline mrpt_autotype & operator=( const mrpt::poses::CPose3D &p)  { this->setSize(6,1); mrpt::math::containerFromPoseOrPoint(*this,p); return *this;} \
		inline mrpt_autotype & operator=( const mrpt::poses::CPose3DQuat &p)  { this->setSize(7,1); mrpt::math::containerFromPoseOrPoint(*this,p); return *this;} \
		inline mrpt_autotype & operator=( const mrpt::poses::CPoint2D &p) { this->setSize(2,1); mrpt::math::containerFromPoseOrPoint(*this,p); return *this;} \
		inline mrpt_autotype & operator=( const mrpt::poses::CPoint3D &p) { this->setSize(3,1); mrpt::math::containerFromPoseOrPoint(*this,p); return *this;} \
		/* =========  misc funcs: =========  */ \
		/*! Alternative access to element (i,j) with first index being (1,1) instead of (0,0) */ \
		inline NUMTYPE _E(const size_t row, const size_t col) const { return mrpt_autotype::get_unsafe(row-1,col-1); } \
		/*! Alternative access to element (i,j) with first index being (1,1) instead of (0,0) */ \
		inline NUMTYPE & _E(const size_t row, const size_t col) { return mrpt_autotype::get_unsafe(row-1,col-1); } \
		/*! Access the n'th element viewing the matrix as an array, with n=row*COLS+col */ \
		inline NUMTYPE _A(const size_t n) const { return mrpt_autotype::get_unsafe(n/mrpt_autotype::getColCount(),n%mrpt_autotype::getColCount()); } \
		/*! Access the n'th element viewing the matrix as an array, with n=row*COLS+col */ \
		inline NUMTYPE &_A(const size_t n) { return mrpt_autotype::get_unsafe(n/mrpt_autotype::getColCount(),n%mrpt_autotype::getColCount()); } \
		inline bool IsSquare() const { return mrpt_autotype::getColCount()==mrpt_autotype::getRowCount(); } \
		inline bool isSquare() const { return mrpt_autotype::getColCount()==mrpt_autotype::getRowCount(); } \
		/*! Checks for the absolute value of the determinant of the matrix being equal or below a given threshold (by default, exactly zero). */ \
		inline bool isSingular(const NUMTYPE epsilon=0) const { \
			if (!isSquare()) return false; else return std::abs(this->det())<=epsilon; } \
		/*! The trace of this square matrix */ \
		inline NUMTYPE trace() const { return mrpt::math::detail::trace(*this); }\
		template <class MAT_OUT> \
		void pseudoInverse( MAT_OUT &out ) const /*!< Pseudo inverse of this matrix M: out = (M<sup>T</sup> * M)^-1 * M<sup>T</sup> */ { \
			mrpt_autotype A, Ainv; A.multiply_AtA(*this); A.inv_fast(Ainv); out.multiply_ABt(Ainv,*this); } \
		template <class MAT_OUT> \
		MAT_OUT pseudoInverse() const /*!< Pseudo inverse of this matrix M: return (M<sup>T</sup> * M)^-1 * M<sup>T</sup>. Invoke with template argument for desired return matrix type: m.pseudoInverse<CMatrixDouble>() */ { \
			mrpt_autotype A, Ainv; A.multiply_AtA(*this); A.inv_fast(Ainv); \
			MAT_OUT out; out.multiply_ABt(Ainv,*this); return out; } \
		inline NUMTYPE det() const /*!< Returns the determinant of the matrix  */ { return mrpt::math::detail::detMatrix(*this); } \
		template <class MATRIXOUT> inline void inv( MATRIXOUT& out ) const /*!< out = inv(this) */ { mrpt::math::detail::invMatrix(*this,out); } \
		inline mrpt_autotype inv() const /*!< mrpt_autotype out = inv(this) */ { mrpt_autotype ret; mrpt::math::detail::invMatrix(*this,ret); return ret; } \
		template <class MATRIXOUT> inline void inv_fast( MATRIXOUT& out ) /*!< out = inv(this), overwriting contents of this */ { mrpt::math::detail::invMatrix_destroySrc(*this,out); } \
		/*! Matrix left divide: RES = A<sup>-1</sup> &#183; this  (A must be a square matrix). \sa rightDivideSquare,fastLeftDivideSquare */ \
		template <class MAT2,class MAT3> inline void leftDivideSquare(const MAT2 &A,MAT3 &RES) const { mrpt::math::detail::leftDivideSquare(*this,A,RES); } \
		/*! Matrix right divide: RES = this &#183; B<sup>-1</sup> (B must be a square matrix). \sa leftDivideSquare,fastRightDivideSquare */ \
		template <class MAT2,class MAT3> inline void rightDivideSquare(const MAT2 &B, MAT3 &RES) const { mrpt::math::detail::rightDivideSquare(*this,B,RES); } \
		/*! Matrix left divide: this = A<sup>-1</sup> &#183; this   (A must be a square matrix, and will be modified on return). \sa leftDivideSquare,fastRightDivideSquare */ \
		template <typename MAT2> inline void fastLeftDivideSquare(MAT2 &A) { mrpt::math::detail::fastLeftDivideSquare(*this,A); } \
		/*! Matrix right divide: this = this &#183; B<sup>-1</sup> (B must be a square matrix, and will be modified on return).  \sa leftDivideSquare,fastRightDivideSquare */ \
		template <typename MAT2> inline void fastRightDivideSquare(MAT2 &B) { mrpt::math::detail::fastRightDivideSquare(*this,B); } \
		/*! Save matrix to a text file, compatible with MATLAB text format. */ \
		inline void  saveToTextFile(const std::string &file,mrpt::math::TMatrixTextFileFormat fileFormat = MATRIX_FORMAT_ENG,bool appendMRPTHeader=false,const std::string &userHeader = std::string("") ) const { \
			mrpt::math::detail::saveMatrixToTextFile(*this, file,fileFormat,appendMRPTHeader,userHeader); } \
		/*! Return a string representation of the matrix in the form: "[m11 m12 m13...;m21 m22 m23...]", compatible with MATLAB matrices. */ \
		inline std::string inMatlabFormat(const size_t decimal_digits=6) const { return detail::matrix_inMatlabFormat(*this,decimal_digits); } \
		inline int pivot(const size_t row) { return mrpt::math::detail::matrix_pivot(*this,row); } \
		/* =========  Decomposition functions: =========  */ \
		/*! Cholesky factorization: in = out' *  out \return True on success, false on singular matrix \sa mrpt::math::chol */ \
		template <class MATRIXOUT> inline bool chol(MATRIXOUT &out) const { return mrpt::math::detail::chol(*this,out); } \
		/*! Eigenvalues and eigenvector of symmetric matrix: M = eVecs * eVals * eVecs<sup>T</sup>, with eigenvectors in columns in eVecs, eigenvalues in ascending order in diagonal of eVals  \sa eigenVectorsVec, eigenValues */ \
		template <class MATRIX1,class MATRIX2> \
		inline RET_VOID_ASSERT_MRPTMATRICES(MATRIX1,MATRIX2) eigenVectors( MATRIX1& eVecs, MATRIX2& eVals) const { \
			ARRAY_TYPE_SAMESIZE_ROWS_OF(mrpt_autotype) eigVals; \
			eigVals.resize(this->getRowCount()); \
			mrpt::math::detail::eigenVectorsMatrix(*this, &eVecs,eigVals); \
			const size_t n = eigVals.size(); \
			eVals.zeros(n,n); for (size_t i=0;i<n;i++) eVals.get_unsafe(i,i)=eigVals[i]; \
			} \
		/*! Like eigenVectors, but eigenvalues are returned in a vector instead of a matrix's diagonal. \sa eigenVectors,eigenValues */ \
		template <class MATRIX1,class VECTOR1> \
		inline void eigenVectorsVec( MATRIX1& eVecs, VECTOR1& eVals) const { mrpt::math::detail::eigenVectorsMatrix(*this, &eVecs,eVals); } \
		/*! Return the eigenvalues of the matrix in an ordered vector. \sa eigenVectors */ \
		template <class VECTOR1> \
		inline void eigenValues( VECTOR1& eVals) const { mrpt::math::detail::eigenVectorsMatrix(*this,static_cast<const mrpt_autotype*>(NULL),eVals); } \
		inline void setIdentity()	{\
			ASSERT_(isSquare());\
			set_unsafe(0,0,NUMTYPE(1));\
			for (size_t i=1;i<mrpt_autotype::getRowCount();++i)	{\
				set_unsafe(i,i,NUMTYPE(1));\
				for (size_t j=0;j<i;++j)	{\
					set_unsafe(i,j,NUMTYPE(0));\
					set_unsafe(j,i,NUMTYPE(0));\
				}\
			}\
		}\
		inline void setIdentity(size_t N)	{\
			setSize(N,N);\
			setIdentity();\
		}\
		inline size_t rank(NUMTYPE eps=1e-7) const	{\
			return mrpt::math::detail::rank(*this,eps);\
		}\
		/*! Inserts a matrix into this matrix, at the position (nRow,nCol). An exception is raised in the inserted matrix does not fit. */ \
		template <class MATRIXLIKE> inline void  insertMatrix(const size_t nRow, const size_t nCol, const MATRIXLIKE &in) \
		{ detail::insertMatrixInto(*this,nRow,nCol,in); } \
		/*! Inserts the transpose of a matrix into this matrix, at the position (nRow,nCol). An exception is raised in the inserted matrix does not fit. */ \
		template <class MATRIXLIKE> inline void  insertMatrixTranspose(const size_t nRow, const size_t nCol, const MATRIXLIKE &in) \
		{ detail::insertMatrixTransposeInto(*this,nRow,nCol,in); } \
		/*! Extract a sub matrix from this matrix (the size of the output matrix upon call determines the size of the submatrix to extract). */ \
		template <class MATOUT> inline void extractMatrix(const size_t first_row, const size_t first_col, MATOUT &outMat) const { detail::extractMatrix(*this,first_row,first_col,outMat); } \
		/*! Extract a sub matrix of the given size from this matrix. */ \
		template <class MATOUT> inline void extractMatrix(const size_t first_row, const size_t first_col, const size_t nRows, const size_t nCols, MATOUT &outMat) const { outMat.setSize(nRows,nCols); detail::extractMatrix(*this,first_row,first_col,outMat); } \
		template <class VECLIKE> inline void extractRow(const size_t nRow, VECLIKE &out, const size_t startingCol = 0) const { detail::extractRowFromMatrix(*this,nRow,out,startingCol); } \
		template <class VECLIKE> inline void extractCol(const size_t nCol, VECLIKE &out, const size_t startingRow = 0) const { detail::extractColFromMatrix(*this,nCol,out,startingRow); } \
		template <class VECLIKE> inline void insertRow(const size_t nRow, VECLIKE &in, const size_t startingCol = 0) { detail::insertRowToMatrix(*this,nRow,in,startingCol); } \
		template <class VECLIKE> inline void insertCol(const size_t nCol, VECLIKE &in, const size_t startingRow = 0) { detail::insertColToMatrix(*this,nCol,in,startingRow); } \



		// Generic functions in mrpt::math::detail ----------------------------------------------------
		namespace detail
		{
			template<class MATRIX1,class MATRIX2>
				bool chol(const MATRIX1 &in, MATRIX2 &out);
			template<class MATRIX1>
				typename MATRIX1::value_type trace(const MATRIX1 &m);

			template <class MATRIX1,class MATRIX2,class MATRIXRES>
				void multiply_AB(const MATRIX1& m1,const MATRIX2& m2, MATRIXRES& RESULT );
			template<class MATRIX1, class OTHERVECTOR1,class OTHERVECTOR2>
				void multiply_Ab(const MATRIX1 &m, const OTHERVECTOR1 &vIn,OTHERVECTOR2 &vOut,bool accumToOutput );
			template<class MATRIX1, class OTHERVECTOR1,class OTHERVECTOR2>
				void multiply_Atb(const MATRIX1 &m, const OTHERVECTOR1 &vIn,OTHERVECTOR2 &vOut,bool accumToOutput );
			template <class MATRIX1,class MATRIX2>
				void multiply_AAt( const MATRIX1& m1, MATRIX2& RESULT );
			template <class MATRIX1,class MATRIX2>
				void multiply_AtA( const MATRIX1& m1, MATRIX2& RESULT );

			/*! This executes the operation \f$ \mathbf{R} = \mathbf{H} \mathbf{C} \mathbf{H}^t \f$, where 'this' matrix is \f$ \mathbf{H} \f$ and \f$ \mathbf{C} \f$ is symmetric, in an efficient and numerically stable way.
			  *  If 'this' matrix is \f$ N \times M \f$, then \f$ \mathbf{C} \f$ must be \f$ M \times M \f$, and the result matrix \f$ R \f$ will be \f$ N \times N \f$.
			  * The result from this method is assured to be symmetric (if \f$ \mathbf{C} \f$ is symmetric), whereas executing:
			  \code
				 R = H * C * (~H);
			  \endcode
			  * may lead to non-symmetric matrixes due to numerical rounding errors. In addition, this method is more efficient that the code above (see the MRPT's code examples on matrixes).
			  *  If accumResultInOutput=true, the contents of the output matrix will not be cleared, but added to the result of the operations. In this case it must have the correct size
			  *   before calling or an exception will be raised.
			  * \sa multiply_HCHt_scalar
			  */
			template <typename MAT_H, typename MAT_C, typename MAT_R>
				void multiply_HCHt(const MAT_H &H,const MAT_C &C,MAT_R &R,bool accumResultInOutput=false,bool allow_submatrix_mult=false);
			template <typename MAT_H, typename MAT_C, typename MAT_R>
				void multiply_HtCH(const MAT_H &H,const MAT_C &C,MAT_R &R,bool accumResultInOutput=false,bool allow_submatrix_mult=false);
			template <typename VECTOR_H, typename MAT_C> typename MAT_C::value_type
				multiply_HCHt_scalar(const VECTOR_H &H, const MAT_C &C);

			template <class MAT_X,class MAT_A,class MAT_OUT>
				void multiply_subMatrix(const MAT_X &X,const MAT_A &A,MAT_OUT &outResult,const size_t A_cols_offset,const size_t A_rows_offset,const size_t A_col_count);

			template <class MAT_A,class MAT_B,class MAT_C,class MAT_OUT>
				void multiply_ABC(const MAT_A &A, const MAT_B &B, const MAT_C &C, MAT_OUT & RES);
			template <class MAT_A,class MAT_B,class MAT_C,class MAT_OUT>
				void multiply_ABCt(const MAT_A &A, const MAT_B &B, const MAT_C &C, MAT_OUT & RES);

			template <class MAT_A,class MAT_B,class MAT_OUT>
				void multiply_ABt(const MAT_A &A,const MAT_B &B, MAT_OUT &out);
			template <class MAT_A,class MAT_OUT>
				void multiply_AAt(const MAT_A &A, MAT_OUT &out);
			template <class MAT_A,class MAT_OUT>
				void multiply_AAt(const MAT_A &A,MAT_OUT &out);
			template <class MAT_A,class MAT_B,class MAT_OUT>
				void multiply_result_is_symmetric(const MAT_A &A,const MAT_B &B, MAT_OUT &out);


			template<typename MatrixType> size_t rank(const MatrixType &m,typename MatrixType::value_type eps=1e-7);

			// -------- MATRIX INVERSES --------
			template <class MATRIXIN,class MATRIXOUT>	inline void  invMatrix( const MATRIXIN &M, MATRIXOUT &out_inv );
			template <class MATRIXIN,class MATRIXOUT>	inline void  invMatrix_destroySrc( MATRIXIN &M, MATRIXOUT &out_inv );
			// Specializations in ops_matrices.h

			// templates for special cases:
			template <class MATRIXIN,class MATRIXOUT> inline void invMatrix_special_2x2( const MATRIXIN &M, MATRIXOUT &out_inv );
			template <class MATRIXIN,class MATRIXOUT> inline void invMatrix_special_3x3( const MATRIXIN &M, MATRIXOUT &out_inv );


			// -------- MATRIX DETERMINANT --------
			template <class MATRIX> RET_ELEMENT_ASSERT_MRPTCONTAINER(MATRIX)
				detMatrix(const MATRIX& M);
			// Specializations in ops_matrices.h

			// Special cases:
			template <class MATRIX> inline typename MATRIX::value_type detMatrix_special_2x2(const MATRIX &M);
			template <class MATRIX> inline typename MATRIX::value_type detMatrix_special_3x3(const MATRIX &M);
			template <class MATRIX>        typename MATRIX::value_type detMatrix_special_4x4(const MATRIX &M);


			// -------- Eigenvalues --------
			template <class MATRIX1,class MATRIX2,class VECTOR1> void
				eigenVectorsMatrix(const MATRIX1 &M,MATRIX2 *eVecs,VECTOR1 &eVals );

			// Special cases
			template <class MATRIX1,class MATRIX2,class VECTOR1> void eigenVectorsMatrix_special_2x2(const MATRIX1 &M,MATRIX2 *eVecs,VECTOR1 &eVals );

			// specializations that call the above special cases:
			template <> inline void eigenVectorsMatrix<CMatrixFixedNumeric<float,2,2>,CMatrixFixedNumeric<float,2,2>,CArrayNumeric<float,2> >(const CMatrixFixedNumeric<float,2,2> &M,CMatrixFixedNumeric<float,2,2> *eVecs,CArrayNumeric<float,2> &eVals ) 	{ eigenVectorsMatrix_special_2x2(M,eVecs,eVals); }
			template <> inline void eigenVectorsMatrix<CMatrixFixedNumeric<double,2,2>,CMatrixFixedNumeric<double,2,2>,CArrayNumeric<double,2> >(const CMatrixFixedNumeric<double,2,2> &M,CMatrixFixedNumeric<double,2,2> *eVecs,CArrayNumeric<double,2> &eVals ) 	{ eigenVectorsMatrix_special_2x2(M,eVecs,eVals); }

			// -------- Others --------
			template <class MAT> void
				saveMatrixToTextFile(const MAT &theMatrix, const std::string &file, mrpt::math::TMatrixTextFileFormat fileFormat = MATRIX_FORMAT_ENG, bool appendMRPTHeader = false, const std::string &userHeader = std::string("") );
			template <class MATRIX>
				std::string  matrix_inMatlabFormat(const MATRIX &m,const size_t decimal_digits);

			template <typename T,size_t NROWS,size_t NCOLS> void fixedToDynMatrix( const CMatrixFixedNumeric<T,NROWS,NCOLS> &SRC, CMatrixTemplateNumeric<T> &DST);
			template <class MAT1,class MAT2> void insertMatrixInto( MAT1 &M,const size_t nRow,const size_t nCol,const MAT2 &in);
			template <class MAT1,class MAT2> void insertMatrixTransposeInto( MAT1 &M,const size_t nRow,const size_t nCol,const MAT2 &in);

			template <class MATORG, class MATDEST> void extractMatrix(const MATORG &origin,const size_t first_row,const size_t first_col, MATDEST &out_subMatrix);
			template <class MAT,class VEC> void  extractRowFromMatrix(const MAT &m, size_t nRow, VEC &out, const size_t startingCol);
			template <class MAT,class VEC> void  extractColFromMatrix(const MAT &m, size_t nCol, VEC &out, const size_t startingRow);
			template <class MAT,class VEC> void  insertRowToMatrix(MAT &m, size_t nRow, const VEC &in, const size_t startingCol);
			template <class MAT,class VEC> void  insertColToMatrix(MAT &m, size_t nCol, const VEC &in, const size_t startingRow);

			template <class MATRIX> int matrix_pivot(MATRIX &M, const size_t row);

			template <typename MAT1,typename MAT2,typename MAT3> inline void leftDivideSquare(const MAT1 &C,const MAT2 &A,MAT3 &RES);
			template <typename MAT1,typename MAT2,typename MAT3> inline void rightDivideSquare(const MAT1 &C,const MAT2 &B, MAT3 &RES);
			template <typename MAT1,typename MAT2> inline void fastLeftDivideSquare(MAT1 &inout_CB,MAT2 &willBeDestroyed_A);
			template <typename MAT1,typename MAT2> inline void fastRightDivideSquare(MAT1 &inout_CA,MAT2 &willBeDestroyed_B);


			/*! An auxiliary function, which return true only for CMatrixTemplate-derived classes */
			template <class MATRIX> inline bool isMatrixTypeResizable(const MATRIX&) { return false; }
			// Specializations are in CMatrixTemplateNumeric.h

			template <class CONTAINER> void loadFromTextFileAsVector(CONTAINER &M, const std::string &file);
			template <class CONTAINER> void saveToTextFileAsVector(const CONTAINER &M, const std::string &file, mrpt::math::TMatrixTextFileFormat fileFormat, bool asColumnVector);

			// Implemented in "lightweight_geom_data.cpp"
			TPoint2D BASE_IMPEXP lightFromPose(const mrpt::poses::CPoint2D &p);	//!< Convert a pose into a light-weight structure (functional form, needed for forward declarations)
			TPoint3D BASE_IMPEXP lightFromPose(const mrpt::poses::CPoint3D &p);	//!< Convert a pose into a light-weight structure (functional form, needed for forward declarations)
			TPose2D  BASE_IMPEXP lightFromPose(const mrpt::poses::CPose2D &p);	//!< Convert a pose into a light-weight structure (functional form, needed for forward declarations)
			TPose3D  BASE_IMPEXP lightFromPose(const mrpt::poses::CPose3D &p);	//!< Convert a pose into a light-weight structure (functional form, needed for forward declarations)
			TPose3DQuat BASE_IMPEXP lightFromPose(const mrpt::poses::CPose3DQuat &p);	//!< Convert a pose into a light-weight structure (functional form, needed for forward declarations)

		} // end detail

		// ------- CONTAINERS -----------------
		//  Implementations in ops_contaienrs.h

		template <class CONTAINER> size_t countNonZero(const CONTAINER &a);
		template <class CONTAINER> typename CONTAINER::value_type maximum(const CONTAINER &v, size_t *maxIndex = NULL);
		template <class CONTAINER> typename CONTAINER::value_type minimum(const CONTAINER &v, size_t *minIndex = NULL);
		template <class CONTAINER> void minimum_maximum(const CONTAINER &v,typename CONTAINER::mrpt_autotype::value_type & out_min,typename CONTAINER::mrpt_autotype::value_type& out_max,size_t *minIndex = static_cast<size_t*>(NULL), size_t *maxIndex = static_cast<size_t*>(NULL));
		template <class CONTAINER> typename CONTAINER::value_type norm_inf(const CONTAINER &v, size_t *maxIndex = NULL);
		template <class CONTAINER> typename CONTAINER::value_type squareNorm(const CONTAINER &v);
		template <class CONTAINER> inline typename CONTAINER::value_type norm(const CONTAINER &v);
		template <class CONTAINER> inline double mean(const CONTAINER &v);
		template <class CONTAINER> inline typename CONTAINER::value_type sum(const CONTAINER &v);
		template <class CONTAINER,typename RET>	inline RET sumRetType(const CONTAINER &v);
		template <class CONTAINER> void  adjustRange(CONTAINER &m, const typename CONTAINER::value_type minVal,const typename CONTAINER::value_type maxVal);
		template <class CONTAINER1,class CONTAINER2> void cumsum(const CONTAINER1 &in_data, CONTAINER2 &out_cumsum);
		template <class CONTAINER1,class CONTAINER2> size_t  countCommonElements(const CONTAINER1 &a,const CONTAINER2 &b);
		template <class CONTAINER> std::vector<double> histogram(const CONTAINER &v,double limit_min,double limit_max,size_t number_bins,bool do_normalization=false, std::vector<double> *out_bin_centers=NULL);
		template<class VECTORLIKE> double  stddev(const VECTORLIKE &v, bool unbiased=true);
		template<class VECTORLIKE> void  meanAndStd(const VECTORLIKE &v,double &out_mean, double &out_std, bool unbiased = true);

		/** Conversion of poses to MRPT containers (vector/matrix) */
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPoint2D &p);
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPoint3D &p);
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPose2D &p);
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPose3D &p);
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPose3DQuat &p);

		template <class CONTAINER> inline CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const mrpt::poses::CPoint2D &p) { return containerFromPoseOrPoint(C, mrpt::math::detail::lightFromPose(p)); }
		template <class CONTAINER> inline CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const mrpt::poses::CPoint3D &p) { return containerFromPoseOrPoint(C, mrpt::math::detail::lightFromPose(p)); }
		template <class CONTAINER> inline CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const mrpt::poses::CPose2D &p)  { return containerFromPoseOrPoint(C, mrpt::math::detail::lightFromPose(p)); }
		template <class CONTAINER> inline CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const mrpt::poses::CPose3D &p)  { return containerFromPoseOrPoint(C, mrpt::math::detail::lightFromPose(p)); }
		template <class CONTAINER> inline CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const mrpt::poses::CPose3DQuat &p)  { return containerFromPoseOrPoint(C, mrpt::math::detail::lightFromPose(p)); }


		// Vicinity classes ----------------------------------------------------
		namespace detail	{
			/**
			  * The purpose of this class is to model traits for containers, so that they can be used as return values for the function CMatrixTemplate::getVicinity.
			  * This class is NOT defined for any base container, because correctness would not be guaranteed. Instead, each class must define its own specialization
			  * of the template, containing two functions:
			  * - static void initialize(container<T>,size_t N): must reserve space to allow at least the insertion of N*N elements, in a square fashion when appliable.
			  * - static void insertInContainer(container<T>,size_t r,size_t c,const T &): must insert the given element in the container. Whenever it's possible, it
			  * must insert it in the (r,c) coordinates.
			  * For linear containers, the vicinity functions are guaranteed to insert elements in order, i.e., starting from the top and reading from left to right.
			  */
			template<typename T> class VicinityTraits;

			/**
			  * This huge template encapsulates a function to get the vicinity of an element, with maximum genericity. Although it's not meant to be called directly,
			  * every type defining the ASSERT_ENOUGHROOM assert and the get_unsafe method will work. The assert checks if the boundaries (r-N,r+N,c-N,c+N) fit in
			  * the matrix.
			  * The template parameters are the following:
			  * - MatrixType: the matrix or container base type, from which the vicinity is required.
			  * - T: the base type of the matrix or container.
			  * - ReturnType: the returning container type. The class VicinityTraits<ReturnType> must be completely defined.
			  * - D: the dimension of the vicinity. Current implementations are 4, 5, 8, 9, 12, 13, 20, 21, 24 and 25, although it's easy to implement new variants.
			  */
			template<typename MatrixType,typename T,typename ReturnType,size_t D> struct getVicinity;

		}


		// Other forward decls:
		template <class T> T wrapTo2Pi(T a);


	} // End of namespace
} // End of namespace

#endif
