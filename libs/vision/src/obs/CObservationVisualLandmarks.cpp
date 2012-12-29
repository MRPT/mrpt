/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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

