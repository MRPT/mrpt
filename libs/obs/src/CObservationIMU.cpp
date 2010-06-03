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

#include <mrpt/obs.h>   // Precompiled headers 


#include <mrpt/slam/CObservationIMU.h>
#include <mrpt/math/CMatrixD.h>

using namespace mrpt::slam; 
using namespace mrpt::utils; 
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationIMU, CObservation,mrpt::slam)

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationIMU::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 1;
	else
	{
		out << sensorPose
		    << dataIsPresent
		    << timestamp;
		
		out << rawMeasurements;

		out << sensorLabel;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationIMU::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
		in >> sensorPose
		   >> dataIsPresent;

		in >> timestamp;

		// In version 0 it was a vector of floats:
		if (version<1)
		{
			vector_float	tmp;
			in >> tmp;
			rawMeasurements.resize(tmp.size());
			for (size_t i=0;i<rawMeasurements.size();i++)
				rawMeasurements[i] = tmp[i];
		}
		else
		{
			in >> rawMeasurements;
		}


		in >> sensorLabel;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

