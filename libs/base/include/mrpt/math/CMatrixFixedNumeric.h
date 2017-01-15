/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMatrixFixedNumeric_H
#define CMatrixFixedNumeric_H

#include <mrpt/math/math_frwds.h>  // Forward declarations
#include <mrpt/math/eigen_frwds.h>
#include <mrpt/utils/types_math.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/point_poses2vectors.h> // MRPT_MATRIX_CONSTRUCTORS_FROM_POSES()

namespace mrpt
{
	namespace math
	{
		/**  A numeric matrix of compile-time fixed size.
		 *   Basically, this class is a wrapper on Eigen::Matrix<T,NROWS,NCOLS>, but
		 *   with a RowMajor element memory layout (except for column vectors).
		 *
		 *  These matrices also have iterators to access all the elements in the matrix as a sequence, starting from the element (0,0), then row by row, from left to right.
		 *
		 * \note This class exists for backward compatibility of ancient times when MRPT didn't rely on Eigen, feel free to directly use Eigen::Matrix<> types instead.
		 * \sa CMatrixTemplateNumeric (for dynamic-size matrices)
		 * \note For a complete introduction to Matrices and vectors in MRPT, see: http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
		 * \ingroup mrpt_base_grp
		 */
		template <typename T,size_t NROWS,size_t NCOLS>
		class CMatrixFixedNumeric  :
			public Eigen::Matrix<
				T,
				NROWS,
				NCOLS,
				// Use row major storage for backward compatibility with MRPT matrices in all cases, except in column vectors:
				Eigen::AutoAlign |
				( (NCOLS==1 && NROWS!=1) ? Eigen::ColMajor : Eigen::RowMajor )
				>
		{
		public:
			typedef Eigen::Matrix<T,NROWS,NCOLS, Eigen::AutoAlign | ( (NCOLS==1 && NROWS!=1) ? Eigen::ColMajor : Eigen::RowMajor ) > Base;
			typedef CMatrixFixedNumeric<T,NROWS,NCOLS> mrpt_autotype;

			MRPT_EIGEN_DERIVED_CLASS_CTOR_OPERATOR_EQUAL(CMatrixFixedNumeric) // Implements ctor and "operator =" for any other Eigen class
			MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CMatrixFixedNumeric)

			/** Default constructor, initializes all elements to zero */
			inline CMatrixFixedNumeric() : Base() { Base::setZero(); }

			/** Constructor from an array in row major */
			inline CMatrixFixedNumeric(const T * vals) : Base(vals) { }

			/** Constructor which leaves the matrix uninitialized.
			  *  Example of usage: CMatrixFixedNumeric<double,3,2> M(mrpt::math::UNINITIALIZED_MATRIX);
			  */
			inline CMatrixFixedNumeric(TConstructorFlags_Matrices ) : Base() { }

			template<size_t N,typename ReturnType> inline ReturnType getVicinity(size_t c,size_t r) const	{
				return detail::getVicinity<CMatrixFixedNumeric<T,NROWS,NCOLS>,T,ReturnType,N>::get(c,r,*this);
			}

			inline void loadFromArray(const T* vals)
			{
				Base b(vals);
				*this = b;
			}

			/** == comparison of two matrices; it differs from default Eigen operator in that returns false if matrices are of different sizes instead of raising an assert. */
			template <typename Derived>
			inline bool operator ==(const Eigen::MatrixBase<Derived>& m2) const
			{
				return Base::cols()==m2.cols() &&
					   Base::rows()==m2.rows() &&
					   Base::cwiseEqual(m2).all();
			}
			/** != comparison of two matrices; it differs from default Eigen operator in that returns true if matrices are of different sizes instead of raising an assert. */
			template <typename Derived>
			inline bool operator !=(const Eigen::MatrixBase<Derived>& m2) const { return !((*this)==m2); }


		}; // end of class definition ------------------------------

		namespace detail
		{
			/**
			  * Vicinity traits class specialization for fixed size matrices.
			  */
			template<typename T,size_t D> class VicinityTraits<CMatrixFixedNumeric<T,D,D> >	{
			public:
				inline static void initialize(CMatrixFixedNumeric<T,D,D> &mat,size_t N)	{
               UNUSED(mat);
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
