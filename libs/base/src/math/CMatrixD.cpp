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


#include <mrpt/math/CMatrixD.h>


using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CMatrixD, CSerializable, mrpt::math)



/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CMatrixD::writeToStream(CStream &out, int *out_Version) const
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		// First, write the number of rows and columns:
		out << (uint32_t)rows() << (uint32_t)cols();

		if (rows()>0 && cols()>0)
			for (Index i=0;i<rows();i++)
				out.WriteBufferFixEndianness<Scalar>(&coeff(i,0),cols());
	}

}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CMatrixD::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t nRows,nCols;

			// First, write the number of rows and columns:
			in >> nRows >> nCols;

			setSize(nRows,nCols);

			if (nRows>0 && nCols>0)
				for (Index i=0;i<rows();i++)
					in.ReadBufferFixEndianness<Scalar>(&coeffRef(i,0),nCols);
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

