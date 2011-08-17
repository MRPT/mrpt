/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
	namespace math
	{
		using namespace mrpt::system;
		using namespace mrpt::poses;

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
			  *  Example of usage: CMatrixFixedNumeric<double,3,2> M(UNINITIALIZED_MATRIX);
			  */
			inline CMatrixFixedNumeric(TConstructorFlags_Matrices constructor_flag) : Base() { }

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


		namespace detail	
		{
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
