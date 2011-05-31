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

#include <mrpt/obs.h>   // Precompiled headers



#include <mrpt/slam/CObservationBearingRange.h>
#include <mrpt/system/os.h>

using namespace mrpt::slam; using namespace mrpt::utils; using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationBearingRange, CObservation,mrpt::slam)

/*---------------------------------------------------------------
 Default constructor.
 ---------------------------------------------------------------*/
CObservationBearingRange::CObservationBearingRange( ) :
	minSensorDistance(0),
	maxSensorDistance(0),
	fieldOfView_yaw(DEG2RAD(180)),
	fieldOfView_pitch(DEG2RAD(90)),
	sensorLocationOnRobot(),
	sensedData(),
	validCovariances(false),
	sensor_std_range(0),
	sensor_std_yaw(0),
	sensor_std_pitch(0)
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationBearingRange::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 3;
	else
	{
		uint32_t	i,n;

		// The data
		out << minSensorDistance
		    << maxSensorDistance
		    << fieldOfView_yaw 
			<< fieldOfView_pitch
		    << sensorLocationOnRobot
		    << timestamp;

		out << validCovariances;
		if (!validCovariances)
			out << sensor_std_range << sensor_std_yaw << sensor_std_pitch;

		// Detect duplicate landmarks ID, which is an error!
		std::set<int32_t>  lstIDs;

		n = sensedData.size();
		out << n;
		for (i=0;i<n;i++)
		{
			int32_t  id = sensedData[i].landmarkID;
			if (id!=INVALID_LANDMARK_ID)
			{
				if (0!=lstIDs.count(id))
					THROW_EXCEPTION_CUSTOM_MSG1("Duplicate landmark ID=%i found.",(int)id);
				lstIDs.insert(id);
			}

			out << sensedData[i].range
			    << sensedData[i].yaw
			    << sensedData[i].pitch
			    << id;

			if (validCovariances)
				out << sensedData[i].covariance;
		}

		out << sensorLabel;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationBearingRange::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
			uint32_t		i,n;

			// The data
			in >> minSensorDistance
			   >> maxSensorDistance;

			if (version>=3)
			{
				in >> fieldOfView_yaw
				   >> fieldOfView_pitch;
			}
			else
			{
				float fieldOfView;
				in >> fieldOfView;
				
				fieldOfView_yaw = 
				fieldOfView_pitch = fieldOfView;
			}

			in >> sensorLocationOnRobot;

			if (version>=2)
					in >> timestamp;
			else 	timestamp = INVALID_TIMESTAMP;

			if (version>=3)
			{
				in >> validCovariances;
				if (!validCovariances)
					in >> sensor_std_range >> sensor_std_yaw >> sensor_std_pitch;
			} else
				validCovariances = false;

			in >> n;
			sensedData.resize(n);

			// Detect duplicate landmarks ID, what is an error!
			std::set<int32_t>  lstIDs;

			for (i=0;i<n;i++)
			{
				in >> sensedData[i].range
				   >> sensedData[i].yaw
				   >> sensedData[i].pitch
				   >> sensedData[i].landmarkID;
				
				if (version>=3 && validCovariances)
					in >> sensedData[i].covariance;

				int32_t  id = sensedData[i].landmarkID;
				if (id!=INVALID_LANDMARK_ID)
				{
					if (0!=lstIDs.count(id))
						THROW_EXCEPTION_CUSTOM_MSG1("Duplicate landmark ID=%i found.",(int)id);
					lstIDs.insert(id);
				}
			}

			if (version>=1)
					in >> sensorLabel;
			else 	sensorLabel = "";

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationBearingRange::debugPrintOut()
{
	printf("[CObservationBearingRange::debugPrintOut] Dumping:\n");
	printf("[CObservationBearingRange::debugPrintOut] minSensorDistance:\t%f\n",minSensorDistance);
	printf("[CObservationBearingRange::debugPrintOut] maxSensorDistance:\t%f:\n",maxSensorDistance);
	printf("[CObservationBearingRange::debugPrintOut] %u landmarks:\n",static_cast<unsigned>(sensedData.size()) );

	size_t		i, n = sensedData.size();
	for (i=0;i<n;i++)
		printf("[CObservationBearingRange::debugPrintOut] \tID[%i]: y:%fdeg p:%fdeg range: %f\n",
		sensedData[i].landmarkID,
		RAD2DEG( sensedData[i].yaw ),
		RAD2DEG( sensedData[i].pitch ),
		sensedData[i].range );
}
