/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h"  // Precomp header
//
using namespace mrpt::hmtslam;

IMPLEMENTS_SERIALIZABLE(THypothesisIDSet, CSerializable, mrpt::hmtslam)

uint8_t THypothesisIDSet::serializeGetVersion() const { return 0; }
void THypothesisIDSet::serializeTo(mrpt::serialization::CArchive& out) const
{
	auto N = (uint32_t)size();
	out << N;
	for (THypothesisID it : *this)
		out << it;
}

void THypothesisIDSet::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			uint32_t i, N;
			in >> N;

			clear();
			for (i = 0; i < N; i++)
			{
				THypothesisID tmp;
				in >> tmp;
				insert(tmp);
			}
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}
