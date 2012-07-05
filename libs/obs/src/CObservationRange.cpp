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



#include <mrpt/slam/CObservationRange.h>

using namespace mrpt::slam; using namespace mrpt::utils; using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRange, CObservation,mrpt::slam)


/** Default constructor.
 */
CObservationRange::CObservationRange( ) :
	minSensorDistance	( 0 ),
	maxSensorDistance	( 5 ),
	sensorConeApperture	( DEG2RAD(20) ),
	sensedData()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRange::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 3;
	else
	{
		uint32_t		i,n;

		// The data
		out << minSensorDistance << maxSensorDistance << sensorConeApperture;

		n = sensedData.size();
		out << n;
		for (i=0;i<n;i++)
			out << sensedData[i].sensorID << CPose3D(sensedData[i].sensorPose) << sensedData[i].sensedDistance;

		out << sensorLabel
			<< timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRange::readFromStream(CStream &in, int version)
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
			in >> minSensorDistance >> maxSensorDistance >> sensorConeApperture;

			in >> n;
			sensedData.resize(n);
			CPose3D aux;
			for (i=0;i<n;i++)
			{
				if (version>=3)
						in >> sensedData[i].sensorID;
				else 	sensedData[i].sensorID = i;

				in >> aux >> sensedData[i].sensedDistance;
				sensedData[i].sensorPose = aux;
			}

			if (version>=1)
				in >> sensorLabel;
			else sensorLabel = "";

			if (version>=2)
					in >> timestamp;
			else 	timestamp = INVALID_TIMESTAMP;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
                     getSensorPose
 ---------------------------------------------------------------*/
void CObservationRange::getSensorPose( CPose3D &out_sensorPose ) const
{
	if (sensedData.size())
		out_sensorPose = sensedData[0].sensorPose;
	else 	out_sensorPose = CPose3D(0,0,0);
}

/*---------------------------------------------------------------
                     setSensorPose
 ---------------------------------------------------------------*/
void CObservationRange::setSensorPose( const CPose3D &newSensorPose )
{
	size_t		i, n = sensedData.size();
	if (n)
		for (i=0;i<n;i++)
			sensedData[i].sensorPose=newSensorPose;
}



