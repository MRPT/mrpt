/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CObservationRawDAQ.h>
#include <mrpt/serialization/CArchive.h>
#include <iostream>

using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRawDAQ, CObservation, mrpt::obs)

uint8_t CObservationRawDAQ::serializeGetVersion() const { return 0; }
void CObservationRawDAQ::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << sensorLabel << timestamp << sample_rate << AIN_8bits << AIN_16bits
		<< AIN_32bits << AIN_float << AIN_double << AIN_channel_count
		<< AIN_interleaved << AOUT_8bits << AOUT_16bits << AOUT_float
		<< AOUT_double << DIN << DOUT << CNTRIN_32bits << CNTRIN_double;
}

void CObservationRawDAQ::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> sensorLabel >> timestamp >> sample_rate >> AIN_8bits >>
				AIN_16bits >> AIN_32bits >> AIN_float >> AIN_double >>
				AIN_channel_count >> AIN_interleaved >> AOUT_8bits >>
				AOUT_16bits >> AOUT_float >> AOUT_double >> DIN >> DOUT >>
				CNTRIN_32bits >> CNTRIN_double;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CObservationRawDAQ::getDescriptionAsText(std::ostream& o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	cout << "Sample rate             : " << sample_rate << " Hz" << endl;
	cout << "Analog IN Channel count : " << AIN_channel_count << endl;
	cout << "Analog IN interleaved?  : " << (AIN_interleaved ? "yes" : "no")
		 << endl;

#define RAWDAQ_SHOW_FIRSTS(_VEC)                                         \
	cout << "Raw data in " #_VEC " (" << (_VEC).size()                   \
		 << " entries): First values [";                                 \
	if (!(_VEC).empty())                                                 \
	{                                                                    \
		for (size_t i = 1;                                               \
			 i <= std::min((_VEC).size(), static_cast<size_t>(10)); i++) \
			cout << (_VEC)[i - 1] << " ";                                \
		cout << " ... ";                                                 \
	}                                                                    \
	cout << "]\n";

	RAWDAQ_SHOW_FIRSTS(AIN_8bits)
	RAWDAQ_SHOW_FIRSTS(AIN_16bits)
	RAWDAQ_SHOW_FIRSTS(AIN_32bits)
	RAWDAQ_SHOW_FIRSTS(AIN_float)
	RAWDAQ_SHOW_FIRSTS(AIN_double)
	RAWDAQ_SHOW_FIRSTS(AOUT_8bits)
	RAWDAQ_SHOW_FIRSTS(AOUT_16bits)
	RAWDAQ_SHOW_FIRSTS(AOUT_float)
	RAWDAQ_SHOW_FIRSTS(AOUT_double)
	RAWDAQ_SHOW_FIRSTS(DIN)
	RAWDAQ_SHOW_FIRSTS(DOUT)
	RAWDAQ_SHOW_FIRSTS(CNTRIN_32bits)
	RAWDAQ_SHOW_FIRSTS(CNTRIN_double)

	cout << endl;
}
