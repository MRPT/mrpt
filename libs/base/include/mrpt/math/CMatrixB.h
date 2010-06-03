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
#ifndef CMATRIXB_H
#define CMATRIXB_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CVectorTemplate.h>

namespace mrpt
{
	namespace math
	{

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CMatrixB, mrpt::utils::CSerializable )


		/**  This class is a "CSerializable" wrapper for "CMatrixBool".
		 */
		class BASE_IMPEXP CMatrixB : public mrpt::utils::CSerializable, public CMatrixBool
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CMatrixB )
		public:
			/** Constructor
			  */
			CMatrixB(size_t row = 1, size_t col = 1) : CMatrixBool(row,col)
			{
			}

			/** Copy constructor
			  */
			CMatrixB( const CMatrixBool &m ) : CMatrixBool(m)
			{
			}

			/** Copy constructor
			  */
			CMatrixB( const CMatrixTemplateNumeric<bool> &m ) : CMatrixBool(0,0)
			{
				*this = m;
			}

			/** Assignment operator for float matrixes
			*/
			CMatrixB & operator = (const CMatrixBool& m)
			{
				CMatrixBool::operator =(m);
				return *this;
			}

		}; // end of class definition

	} // End of namespace
} // End of namespace

#endif
