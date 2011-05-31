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

#include <mrpt/base.h>  // Precompiled headers 

#include <mrpt/math/CMatrixB.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;


// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CMatrixB, CSerializable, mrpt::math)

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CMatrixB::writeToStream(CStream &out, int *out_Version) const
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		out << (uint32_t)sizeof(m_Val[0][0]);

		// First, write the number of rows and columns:
		out << (uint32_t)m_Rows << (uint32_t)m_Cols;

		if (m_Rows>0 && m_Cols>0)
			for (unsigned int i=0;i<m_Rows;i++)
				out.WriteBuffer(m_Val[i],sizeof(m_Val[0][0])*m_Cols);
	}

}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CMatrixB::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t size_bool;
			in >> size_bool;
			if ( size_bool != sizeof(m_Val[0][0]) )
				THROW_EXCEPTION("Error: size of 'bool' is different in serialized data!")
	
			uint32_t nRows,nCols;

			// First, write the number of rows and columns:
			in >> nRows >> nCols;

			setSize(nRows,nCols);

			if (nRows>0 && nCols>0)
				for (unsigned int i=0;i<nRows;i++)
					in.ReadBuffer(m_Val[i],sizeof(m_Val[0][0])*m_Cols);
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


