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
#ifndef CMATRIX_H
#define CMATRIX_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/CVectorTemplate.h>
#include <mrpt/utils/CStream.h>

namespace mrpt
{

	namespace math
	{

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CMatrix, mrpt::utils::CSerializable )

		/**  This class is a "CSerializable" wrapper for "CMatrixFloat".
		 */
		class BASE_IMPEXP CMatrix : public mrpt::utils::CSerializable, public CMatrixFloat
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CMatrix )
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
				*this = m;
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

		}; // end of class definition

	} // End of namespace
} // End of namespace

#endif
