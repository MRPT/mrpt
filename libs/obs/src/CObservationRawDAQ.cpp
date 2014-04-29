/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/slam/CObservationRawDAQ.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRawDAQ, CObservation,mrpt::slam)

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRawDAQ::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << sensorLabel << timestamp << sample_rate
			<< AIN_8bits << AIN_16bits << AIN_32bits << AIN_float << AIN_double
			<< AIN_channel_count << AIN_interleaved
			<< AOUT_8bits << AOUT_16bits << AOUT_float << AOUT_double << DIN << DOUT << CNTRIN_32bits << CNTRIN_double;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRawDAQ::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			in  >> sensorLabel >> timestamp >> sample_rate
				>> AIN_8bits >> AIN_16bits >> AIN_32bits >> AIN_float >> AIN_double
				>> AIN_channel_count >> AIN_interleaved
				>> AOUT_8bits >> AOUT_16bits >> AOUT_float >> AOUT_double >> DIN >> DOUT >> CNTRIN_32bits >> CNTRIN_double;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}
