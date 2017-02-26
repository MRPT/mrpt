/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMatrixTemplateNumeric_H
#define CMatrixTemplateNumeric_H

#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/types_math.h>
#include <mrpt/utils/TTypeName.h>
#include <mrpt/math/point_poses2vectors.h> // MRPT_MATRIX_CONSTRUCTORS_FROM_POSES()

namespace mrpt
{
	namespace math
	{
		/**  A matrix of dynamic size.
		  *   Basically, this class is a wrapper on Eigen::Matrix<T,Dynamic,Dynamic>, but
		  *   with a RowMajor element memory layout (except for column vectors).
		  *
		  * \note This class exists for backward compatibility of ancient times when MRPT didn't rely on Eigen, feel free to directly use Eigen::Matrix<> types instead.
		  *
		  * \sa CMatrixTemplate (a non Eigen lib-based  class, which can hold arbitrary objects, not only numerical types).
		 * \note For a complete introduction to Matrices and vectors in MRPT, see: http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
		 * \ingroup mrpt_base_grp
		  */
		template <class T>
		class CMatrixTemplateNumeric :
			public Eigen::Matrix<
				T,
				Eigen::Dynamic,
				Eigen::Dynamic,
				// Use row major storage for backward compatibility with MRPT matrices in all cases (even in column vectors!)
				Eigen::AutoAlign | Eigen::RowMajor
				>
		{
		public:
			typedef Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic,Eigen::AutoAlign|Eigen::RowMajor> Base;
			typedef CMatrixTemplateNumeric<T> mrpt_autotype;

			MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CMatrixTemplateNumeric)

			/** Default constructor, builds a 1x1 matrix */
			inline CMatrixTemplateNumeric() : Base(1,1) { Base::setZero(); }

			/** Constructor that builds a 0x0 matrix (that is, uninitialized), for usage in places where efficiency is a priority.
			  *  Use as:
			  *   \code
			  *     CMatrixTemplateNumeric<double>  M( UNINITIALIZED_MATRIX);
			  *   \endcode
			  */
			inline CMatrixTemplateNumeric(TConstructorFlags_Matrices ) : Base( 0,0 ) { }

			/** Constructor, creates a matrix of the given size, filled with zeros. */
			inline CMatrixTemplateNumeric(size_t row, size_t col) : Base(row,col) { Base::setZero(); }

			MRPT_EIGEN_DERIVED_CLASS_CTOR_OPERATOR_EQUAL(CMatrixTemplateNumeric) // Implements ctor and "operator =" for any other Eigen class

			/** Assignment operator of other types
			*/
			template <class R>
			inline CMatrixTemplateNumeric<T>& operator = (const CMatrixTemplate<R>& m)
			{
				Base::resize( m.getRowCount(), m.getColCount() );

				for (size_t i=0; i < CMatrixTemplate<T>::getRowCount(); i++)
					for (size_t j=0; j < CMatrixTemplate<T>::getColCount(); j++)
						Base::coeffRef(i,j) = static_cast<T>(m.get_unsafe(i,j));
				return *this;
			}

			/** Assignment from any Eigen matrix/vector */
			template <typename Derived>
			inline CMatrixTemplateNumeric<T>& operator =(const Eigen::MatrixBase<Derived>& m) const
			{
				Base::operator =(m);
				return *this;
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
			inline CMatrixTemplateNumeric(size_t row, size_t col, V (&theArray)[N] ) : Base(row,col)
			{
				ASSERT_EQUAL_(row*col,N)
				ASSERT_EQUAL_(sizeof(theArray[0]),sizeof(T))
				::memcpy(Base::data(),&theArray[0],sizeof(T)*N); // Remember, row-major order!
			}

			/** Destructor
			  */
			inline ~CMatrixTemplateNumeric() { }

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
			inline bool operator !=(const Eigen::MatrixBase<Derived>& m2)  const{ return !((*this)==m2); }

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
