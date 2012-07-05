/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
