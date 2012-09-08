/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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



#include <mrpt/slam/CObservationBeaconRanges.h>
#include <mrpt/system/os.h>

using namespace mrpt::slam; using namespace mrpt::utils; using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationBeaconRanges, CObservation,mrpt::slam)

/** Default constructor.
 */
CObservationBeaconRanges::CObservationBeaconRanges( ) :
	minSensorDistance ( 0 ),
	maxSensorDistance ( 1e2f ),
	stdError ( 1e-2f ),
	sensedData(),
	auxEstimatePose()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationBeaconRanges::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 3;
	else
	{
		uint32_t	i,n;

		// The data
		out << minSensorDistance << maxSensorDistance << stdError;

		n = sensedData.size();
		out << n;
		for (i=0;i<n;i++)
			out << sensedData[i].sensorLocationOnRobot << sensedData[i].sensedDistance << sensedData[i].beaconID;

		out << auxEstimatePose;

		out << sensorLabel
			<< timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationBeaconRanges::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
			uint32_t		i,n,id;

			// The data
			in >> minSensorDistance >> maxSensorDistance >> stdError;

			in >> n;
			sensedData.resize(n);
			for (i=0;i<n;i++)
			{
				in >> sensedData[i].sensorLocationOnRobot >> sensedData[i].sensedDistance;
				in >> id; sensedData[i].beaconID = id;
			}

			if (version>=1)
				in >> auxEstimatePose;

			if (version>=2)
					in >> sensorLabel;
			else 	sensorLabel="";

			if (version>=3)
					in >> timestamp;
			else 	timestamp = INVALID_TIMESTAMP;


		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationBeaconRanges::debugPrintOut()
{
	printf("[CObservationBeaconRanges::debugPrintOut] Dumping:\n");
	printf("[CObservationBeaconRanges::debugPrintOut] minSensorDistance:\t%f\n",minSensorDistance);
	printf("[CObservationBeaconRanges::debugPrintOut] maxSensorDistance:\t%f:\n",maxSensorDistance);
	printf("[CObservationBeaconRanges::debugPrintOut] stdError:\t%f\n",stdError);
	printf("[CObservationBeaconRanges::debugPrintOut] %u ranges:\n",static_cast<unsigned>( sensedData.size() ));

	size_t		i, n = sensedData.size();
	for (i=0;i<n;i++)
		printf("[CObservationBeaconRanges::debugPrintOut] \tID[%u]: %f\n",
		sensedData[i].beaconID,
		sensedData[i].sensedDistance );
}

/*---------------------------------------------------------------
                     getSensorPose
 ---------------------------------------------------------------*/
void CObservationBeaconRanges::getSensorPose( CPose3D &out_sensorPose ) const
{
	if (sensedData.size())
		out_sensorPose=sensedData[0].sensorLocationOnRobot;
	else 	out_sensorPose = CPose3D(0,0,0);
}

/*---------------------------------------------------------------
                     setSensorPose
 ---------------------------------------------------------------*/
void CObservationBeaconRanges::setSensorPose( const CPose3D &newSensorPose )
{
	size_t		i, n = sensedData.size();
	if (n)
		for (i=0;i<n;i++)
			sensedData[i].sensorLocationOnRobot=CPoint3D(newSensorPose);
}

/*---------------------------------------------------------------
                     getSensedRangeByBeaconID
 ---------------------------------------------------------------*/
float CObservationBeaconRanges::getSensedRangeByBeaconID(int32_t beaconID)
{
	for (size_t i=0;i<sensedData.size();i++)
		if (sensedData[i].beaconID==beaconID)
			return sensedData[i].sensedDistance;
	return 0;
}
