/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMATRIXD_H
#define CMATRIXD_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>

namespace mrpt
{
	namespace math
	{
		// This must be added to any CSerializable derived class:
		// Note: instead of the standard "DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE", classes inheriting
		// from templates need special nasty handling for MSVC DLL exports...
		DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE2(CMatrixD, mrpt::utils::CSerializable, CMatrixD)
		BASE_IMPEXP ::mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in, CMatrixDPtr &pObj);


		/**  This class is a "CSerializable" wrapper for "CMatrixTemplateNumeric<double>".
		 * \note For a complete introduction to Matrices and vectors in MRPT, see: http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP_TEMPL CMatrixD : public mrpt::utils::CSerializable, public CMatrixTemplateNumeric<double>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE_CUSTOM_LINKAGE( CMatrixD, void BASE_IMPEXP, static BASE_IMPEXP, virtual BASE_IMPEXP )
		public:
			/** Constructor */
			CMatrixD() : CMatrixTemplateNumeric<double>(1,1)
			{ }

			/** Constructor */
			CMatrixD(size_t row, size_t col) : CMatrixTemplateNumeric<double>(row,col)
			{ }

			/** Copy constructor */
			CMatrixD( const CMatrixTemplateNumeric<double> &m ) : CMatrixTemplateNumeric<double>(m)
			{ }

			/** Copy constructor  */
			CMatrixD( const CMatrixFloat &m ) : CMatrixTemplateNumeric<double>(0,0) {
				*this = m.eval().cast<double>();
			}

			/*! Assignment operator from any other Eigen class */
			template<typename OtherDerived>
			inline CMatrixD & operator= (const Eigen::MatrixBase <OtherDerived>& other) {
				CMatrixTemplateNumeric<double>::operator=(other);
				return *this;
			}
			/*! Constructor from any other Eigen class */
			template<typename OtherDerived>
			inline CMatrixD(const Eigen::MatrixBase <OtherDerived>& other) : CMatrixTemplateNumeric<double>(other) { }

			/** Constructor from a TPose2D, which generates a 3x1 matrix \f$ [x y \phi]^T \f$  */
			explicit CMatrixD( const TPose2D &p);
			/** Constructor from a TPose3D, which generates a 6x1 matrix \f$ [x y z yaw pitch roll]^T \f$  */
			explicit CMatrixD( const TPose3D &p);
			/** Constructor from a TPoint2D, which generates a 2x1 matrix \f$ [x y]^T \f$ */
			explicit CMatrixD( const TPoint2D &p);
			/** Constructor from a TPoint3D, which generates a 3x1 matrix \f$ [x y z]^T \f$ */
			explicit CMatrixD( const TPoint3D &p);

		}; // end of class definition
		DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE2(CMatrixD, mrpt::utils::CSerializable, CMatrixD)

	} // End of namespace
} // End of namespace

#endif
