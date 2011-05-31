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

#include <mrpt/vision.h>  // Precompiled headers



#include <mrpt/slam/CObservationVisualLandmarks.h>

using namespace mrpt::slam; 
using namespace mrpt::utils; 
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationVisualLandmarks, CObservation,mrpt::slam)

/** Constructor
 */
CObservationVisualLandmarks::CObservationVisualLandmarks( ) :
	refCameraPose(),
	landmarks()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationVisualLandmarks::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 1;
	else
	{
		out << refCameraPose
			<< timestamp
		// The landmarks:
			<< landmarks
			<< sensorLabel;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationVisualLandmarks::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			in 	>> refCameraPose
				>> timestamp

			// The landmarks:
				>> landmarks;

			if (version>0)
					in >> sensorLabel;
			else	sensorLabel = "";

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
 Inserts a pure virtual method for finding the likelihood between this
   and another observation, probably only of the same derived class. The operator
   may be asymmetric.

 \param anotherObs The other observation to compute likelihood with.
 \param anotherObsPose If known, the belief about the robot pose when the other observation was taken can be supplied here, or NULL if it is unknown.

 \return Returns a likelihood measurement, in the range [0,1].
 \exception std::exception On any error, as another observation being of an invalid class.
  ---------------------------------------------------------------*/
float  CObservationVisualLandmarks::likelihoodWith(
	const CObservation		*anotherObs,
	const CPosePDF			*anotherObsPose ) const
{
	MRPT_UNUSED_PARAM(anotherObs); MRPT_UNUSED_PARAM(anotherObsPose);
	return 0;
}

