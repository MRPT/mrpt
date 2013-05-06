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
#ifndef CMATRIX_H
#define CMATRIX_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/utils/CStream.h>

namespace mrpt
{
	namespace math
	{
		// This must be added to any CSerializable derived class:
		// Note: instead of the standard "DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE", classes inheriting
		// from templates need special nasty handling for MSVC DLL exports...
		DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE2(CMatrix, mrpt::utils::CSerializable, CMatrix)
		BASE_IMPEXP ::mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in, CMatrixPtr &pObj);

		/**  This class is a "CSerializable" wrapper for "CMatrixFloat".
		 * \note For a complete introduction to Matrices and vectors in MRPT, see: http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP_TEMPL CMatrix : public mrpt::utils::CSerializable, public CMatrixFloat
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE_CUSTOM_LINKAGE( CMatrix, void BASE_IMPEXP, static BASE_IMPEXP, virtual BASE_IMPEXP )

		public:
			/** Constructor  */
			CMatrix() : CMatrixFloat(1,1)
			{ }

			/** Constructor  */
			CMatrix(size_t row, size_t col) : CMatrixFloat(row,col)
			{ }

			/** Copy constructor
			  */
			CMatrix( const CMatrixFloat &m ) : CMatrixFloat(m)
			{ }

			/** Copy constructor
			  */
			CMatrix( const CMatrixTemplateNumeric<double> &m ) : CMatrixFloat(0,0)
			{
				*this = m.eval().cast<float>();
			}

			/** Constructor from a TPose2D, which generates a 3x1 matrix \f$ [x y \phi]^T \f$
			   */
			explicit CMatrix( const TPose2D &p) : CMatrixFloat(p) {}

			/** Constructor from a mrpt::poses::CPose6D, which generates a 6x1 matrix \f$ [x y z yaw pitch roll]^T \f$
			   */
			explicit CMatrix( const TPose3D &p) : CMatrixFloat(p) {}

			/** Constructor from a TPoint2D, which generates a 2x1 matrix \f$ [x y]^T \f$
			   */
			explicit CMatrix( const TPoint2D &p) : CMatrixFloat(p) {}

			/** Constructor from a TPoint3D, which generates a 3x1 matrix \f$ [x y z]^T \f$
			   */
			explicit CMatrix( const TPoint3D &p) : CMatrixFloat(p) {}

			/** Assignment operator for float matrixes
			*/
			template <class OTHERMAT>
			inline CMatrix & operator = (const OTHERMAT& m)
			{
				CMatrixFloat::operator =(m);
				return *this;
			}

			/*! Assignment operator from any other Eigen class */
			template<typename OtherDerived>
			inline CMatrix & operator= (const Eigen::MatrixBase <OtherDerived>& other) {
				CMatrixTemplateNumeric<float>::operator=(other);
				return *this;
			}
			/*! Constructor from any other Eigen class */
			template<typename OtherDerived>
			inline CMatrix(const Eigen::MatrixBase <OtherDerived>& other) : CMatrixTemplateNumeric<float>(other) { }

		}; // end of class definition

	} // End of namespace
} // End of namespace

#endif
