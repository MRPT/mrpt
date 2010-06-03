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
#ifndef  mrpt_matrices_metaprogramming_H
#define  mrpt_matrices_metaprogramming_H

#include <mrpt/math/math_frwds.h>  // Fordward declarations

namespace mrpt { namespace math { namespace detail
{
	/** @name Matrix metaprogramming helpers @{ */

	/** TMatrixProductType: Metaprogramming helper to determine the type of the product of two matrices,
	  * in compile time.  This assume usage ofDECLARE_MRPT_CONTAINER_IS_MATRIX and DECLARE_MRPT_CONTAINER_IS_MATRIX_FIXED
	  * Example of usage with the helper macro  MAT_TYPE_PRODUCT_OF():
	  *
	  * \code
	  *   void f(MAT1 m1,MAT2 m2) {
	  *   	MAT_TYPE_PRODUCT_OF(MAT1,MAT2) res;
	  * 	res = m1*m2;
	  *  	...
	  * \endcode
	  */
	template <typename T,size_t NR1,size_t NC1,size_t NR2,size_t NC2>
	struct TMatrixProductType // FIX * FIX -> FIX
	{
		typedef CMatrixFixedNumeric<T,NR1,NC2> mat_type;
	};

	template <typename T>  // DYN * DYN -> DYN
	struct TMatrixProductType<T,size_t(-1),size_t(-1),size_t(-1),size_t(-1)>
	{
		typedef CMatrixTemplateNumeric<T> mat_type;
	};

	template <typename T,size_t NR1,size_t NC1>  // FIX * DYN -> DYN
	struct TMatrixProductType<T,NR1,NC1,size_t(-1),size_t(-1)>
	{
		typedef CMatrixTemplateNumeric<T> mat_type;
	};

	template <typename T,size_t NR2,size_t NC2>  // DYN * FIX -> DYN
	struct TMatrixProductType<T,size_t(-1),size_t(-1),NR2,NC2>
	{
		typedef CMatrixTemplateNumeric<T> mat_type;
	};


	// Macro for easy usage of "TMatrixProductType":
	// For use out of templates:
	#define MAT_TYPEDECL_PRODUCT_OF(MAT_1,MAT_2) \
		mrpt::math::detail::TMatrixProductType< \
			MAT_1::value_type, \
			MAT_1::mrpt_matrix_type_nrows, \
			MAT_1::mrpt_matrix_type_ncols, \
			MAT_2::mrpt_matrix_type_nrows, \
			MAT_2::mrpt_matrix_type_ncols>::mat_type

	// For use within templates:
	#define MAT_TYPE_PRODUCT_OF(MAT_1,MAT_2) \
		typename mrpt::math::detail::TMatrixProductType< \
			typename MAT_1::value_type, \
			MAT_1::mrpt_matrix_type_nrows, \
			MAT_1::mrpt_matrix_type_ncols, \
			MAT_2::mrpt_matrix_type_nrows, \
			MAT_2::mrpt_matrix_type_ncols>::mat_type


	/** TMatrixTransposeType: Metaprogramming helper to determine the type of the transpose of a matrix in compile time.
	  * This assume usage of DECLARE_MRPT_CONTAINER_IS_MATRIX and DECLARE_MRPT_CONTAINER_IS_MATRIX_FIXED.
	  * Example of usage with the helper macro  MAT_TYPE_TRANSPOSE_OF():
	  *
	  * \code
	  *   void f(MAT1 m1) {
	  *   	MAT_TYPE_TRANSPOSE_OF(MAT1) res = ~m1;
	  *  	...
	  * \endcode
	  */
	template <typename T,size_t NR1,size_t NC1>
	struct TMatrixTransposeType // FIX -> FIX
	{
		typedef CMatrixFixedNumeric<T,NC1,NR1> mat_type;
	};

	template <typename T>  // DYN -> DYN
	struct TMatrixTransposeType<T,size_t(-1),size_t(-1)>
	{
		typedef CMatrixTemplateNumeric<T> mat_type;
	};

	// Macro for easy usage of "TMatrixProductType":
	// For use out of templates:
	#define MAT_TYPEDECL_TRANSPOSE_OF(MAT) \
		mrpt::math::detail::TMatrixTransposeType< \
			MAT::value_type, \
			MAT::mrpt_matrix_type_nrows, \
			MAT::mrpt_matrix_type_ncols>::mat_type
			
	// For use within templates:
	#define MAT_TYPE_TRANSPOSE_OF(MAT) \
		typename mrpt::math::detail::TMatrixTransposeType< \
			typename MAT::value_type, \
			MAT::mrpt_matrix_type_nrows, \
			MAT::mrpt_matrix_type_ncols>::mat_type



	/** TMatrixCovarianceType: Metaprogramming helper to determine the type of the MxM covariance matrix of an NxM matrix with samples in each row, in compile time.
	  * This assume usage of DECLARE_MRPT_CONTAINER_IS_MATRIX and DECLARE_MRPT_CONTAINER_IS_MATRIX_FIXED.
	  * Example of usage with the helper macro  MAT_TYPE_COVARIANCE_OF():
	  *
	  * \code
	  *   void f(MAT1 m1) {
	  *   	MAT_TYPE_COVARIANCE_OF(MAT1) res = ...
	  *  	...
	  * \endcode
	  */
	template <typename T,size_t NR1,size_t NC1>
	struct TMatrixCovarianceType // FIX -> FIX
	{
		typedef CMatrixFixedNumeric<T,NC1,NC1> mat_type;
	};

	template <typename T>  // DYN -> DYN
	struct TMatrixCovarianceType<T,size_t(-1),size_t(-1)>
	{
		typedef CMatrixTemplateNumeric<T> mat_type;
	};

	// Macro for easy usage of "TMatrixCovarianceType":
	// For use out of templates:
	#define MAT_TYPEDECL_COVARIANCE_OF(MAT) \
		mrpt::math::detail::TMatrixCovarianceType< \
			MAT::value_type, \
			MAT::mrpt_matrix_type_nrows, \
			MAT::mrpt_matrix_type_ncols>::mat_type
			
	// For use within templates:
	#define MAT_TYPE_COVARIANCE_OF(MAT) \
		typename mrpt::math::detail::TMatrixCovarianceType< \
			typename MAT::value_type, \
			MAT::mrpt_matrix_type_nrows, \
			MAT::mrpt_matrix_type_ncols>::mat_type


	/** TMatrixJacobianType: Metaprogramming helper to determine the type of the MxN matrix from the types of two vector-like objects VECX (length N) and VECY (length M).
	  * Example of usage with the helper macro  MAT_TYPE_JACOBIAN_OF():
	  *
	  * \code
	  *   void f(VEC1 v1,VEC2 v2) {
	  *   	MAT_TYPE_JACOBIAN_OF(VEC1,VEC2) res = ...
	  *  	...
	  * \endcode
	  */
	template <typename T,size_t N1,size_t N2>
	struct TMatrixJacobianType // In general -> Fix.
	{
		typedef CMatrixFixedNumeric<T,N2,N1> mat_type;
	};
	template <typename T,size_t N1>  // One Dyn -> Dyn
	struct TMatrixJacobianType<T,N1,size_t(-1)>
	{
		typedef CMatrixTemplateNumeric<T> mat_type;
	};
	template <typename T,size_t N2>  // One Dyn -> Dyn
	struct TMatrixJacobianType<T,size_t(-1),N2>
	{
		typedef CMatrixTemplateNumeric<T> mat_type;
	};
	template <typename T>  // One Dyn -> Dyn
	struct TMatrixJacobianType<T,size_t(-1),size_t(-1)>
	{
		typedef CMatrixTemplateNumeric<T> mat_type;
	};
	// Macro for easy usage of "TMatrixJacobianType":
	// For use out of templates:
	#define MAT_TYPEDECL_JACOBIAN_OF(VECX,VECY) \
		mrpt::math::detail::TMatrixJacobianType< \
			VECX::value_type, \
			VECX::mrpt_vector_type_len, \
			VECY::mrpt_vector_type_len>::mat_type

	// For use within templates:
	#define MAT_TYPE_JACOBIAN_OF(VECX,VECY) \
		typename mrpt::math::detail::TMatrixJacobianType< \
			typename VECX::value_type, \
			VECX::mrpt_vector_type_len, \
			VECY::mrpt_vector_type_len>::mat_type

	/** TMatrixSameSizeOfType: Metaprogramming helper to create a matrix of the same size that another type, in compile time.
	  * This assume usage of DECLARE_MRPT_CONTAINER_IS_MATRIX and DECLARE_MRPT_CONTAINER_IS_MATRIX_FIXED.
	  * Example of usage with the helper macro  MAT_TYPE_SAMESIZE_OF():
	  *
	  * \code
	  *   void f(MAT1 m1) {
	  *   	MAT_TYPE_SAMESIZE_OF(MAT1) res = ...
	  *  	...
	  * \endcode
	  */
	template <typename T,size_t NR1,size_t NC1>
	struct TMatrixSameSizeOfType // FIX -> FIX
	{
		typedef CMatrixFixedNumeric<T,NR1,NC1> mat_type;
	};

	template <typename T>  // DYN -> DYN
	struct TMatrixSameSizeOfType<T,size_t(-1),size_t(-1)>
	{
		typedef CMatrixTemplateNumeric<T> mat_type;
	};

	// Macro for easy usage of "TMatrixSameSizeOfType":
	// For use out of templates:
	#define MAT_TYPEDECL_SAMESIZE_OF(MAT) \
		mrpt::math::detail::TMatrixSameSizeOfType< \
			MAT::value_type, \
			MAT::mrpt_matrix_type_nrows, \
			MAT::mrpt_matrix_type_ncols>::mat_type

	// For use within templates:
	#define MAT_TYPE_SAMESIZE_OF(MAT) \
		typename mrpt::math::detail::TMatrixSameSizeOfType< \
			typename MAT::value_type, \
			MAT::mrpt_matrix_type_nrows, \
			MAT::mrpt_matrix_type_ncols>::mat_type


	/** TArrayOrVectorRowCountOfType: Metaprogramming helper to create a CArray or a std::vector<> with its size to the number of rows of a matrix, in compile time.
	  * This assume usage of DECLARE_MRPT_CONTAINER, etc.
	  * Example of usage with the helper macro ARRAY_TYPE_SAMESIZE_ROWS_OF():
	  *
	  * \code
	  *   void f(MAT1 m1) {
	  *   	ARRAY_TYPE_SAMESIZE_ROWS_OF(MAT1) res = ...
	  *  	...
	  * \endcode
	  */
	template <typename T,size_t NR1>
	struct TArrayOrVectorRowCountOfType // FIX -> CArray
	{
		typedef CArrayNumeric<T,NR1> vec_type;
	};

	template <typename T>  // DYN -> std::vector<>
	struct TArrayOrVectorRowCountOfType<T,size_t(-1)>
	{
		typedef std::vector<T> vec_type;
	};

	// Macro for easy usage of "TArrayOrVectorRowCountOfType":
	// For use out of templates:
	#define ARRAY_TYPEDECL_SAMESIZE_ROWS_OF(MAT) \
		mrpt::math::detail::TArrayOrVectorRowCountOfType< \
			MAT::value_type, \
			MAT::mrpt_matrix_type_nrows>::vec_type

	// For use within templates:
	#define ARRAY_TYPE_SAMESIZE_ROWS_OF(MAT) \
		typename mrpt::math::detail::TArrayOrVectorRowCountOfType< \
			typename MAT::value_type, \
			MAT::mrpt_matrix_type_nrows>::vec_type


	/** @} */
}}} // end namespaces

#endif
