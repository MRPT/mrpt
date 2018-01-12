/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "serialization-precomp.h"  // Precompiled headers

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/serialization/archiveFrom_std_vector.h>
#include <cstdio>

using namespace mrpt;
using namespace mrpt::serialization;

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(CSerializable, CObject, mrpt::serialization)

void mrpt::serialization::ObjectToOctetVector(
	const CSerializable* o, std::vector<uint8_t>& out_vector)
{
	out_vector.clear();
	auto arch = archiveFrom(out_vector);
	arch << *o;
}

void mrpt::serialization::OctetVectorToObject(
	const std::vector<uint8_t>& in_data, CSerializable::Ptr& obj)
{
	obj.reset();

	if (in_data.empty()) return;
	auto arch = archiveFrom(in_data);
	obj = arch.ReadObject();
}
