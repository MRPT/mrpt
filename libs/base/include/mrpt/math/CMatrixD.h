/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
