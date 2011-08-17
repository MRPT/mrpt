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
#ifndef CMATRIXD_H
#define CMATRIXD_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/utils_defs.h>


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
			//DEFINE_SERIALIZABLE( CMatrixD )
			//DEFINE_MRPT_OBJECT(CMatrixD)
		protected:
			static  const mrpt::utils::TRuntimeClassId* _GetBaseClass();
			static mrpt::utils::CLASSINIT _init_CMatrixD;
		public:
			/*! A typedef for the associated smart pointer */
			typedef CMatrixDPtr SmartPtr;
			static BASE_IMPEXP  mrpt::utils::TRuntimeClassId  classCMatrixD;
			static BASE_IMPEXP  const mrpt::utils::TRuntimeClassId *classinfo;
			virtual BASE_IMPEXP  const mrpt::utils::TRuntimeClassId* GetRuntimeClass() const;
			static  BASE_IMPEXP mrpt::utils::CObject* CreateObject();
			static BASE_IMPEXP CMatrixDPtr Create();
			virtual BASE_IMPEXP mrpt::utils::CObject *duplicate() const;
		protected:
			/*! @name CSerializable virtual methods */
			/*! @{ */
			BASE_IMPEXP void writeToStream(mrpt::utils::CStream &out, int *getVersion) const;
			BASE_IMPEXP void readFromStream(mrpt::utils::CStream &in, int version);
			/*! @} */

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

			/** Copy constructor
			  */
			CMatrixD( const CMatrixFloat &m ) : CMatrixTemplateNumeric<double>(0,0)
			{
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

			/** Constructor from a TPose2D, which generates a 3x1 matrix \f$ [x y \phi]^T \f$
			   */
			explicit CMatrixD( const TPose2D &p) : CMatrixDouble(p) {}

			/** Constructor from a TPose3D, which generates a 6x1 matrix \f$ [x y z yaw pitch roll]^T \f$
			   */
			explicit CMatrixD( const TPose3D &p) : CMatrixDouble(p) {}

			/** Constructor from a TPoint2D, which generates a 2x1 matrix \f$ [x y]^T \f$
			   */
			explicit CMatrixD( const TPoint2D &p) : CMatrixDouble(p) {}

			/** Constructor from a mrpt::poses::CPoint3D, which generates a 3x1 matrix \f$ [x y z]^T \f$
			   */
			explicit CMatrixD( const TPoint3D &p) : CMatrixDouble(p) {}


			/** Assignment operator for float matrixes
			*/
			template <class OTHERMAT>
			inline CMatrixD & operator = (const OTHERMAT& m)
			{
				CMatrixDouble::operator =(m);
				return *this;
			}

		}; // end of class definition

	} // End of namespace
} // End of namespace

#endif
