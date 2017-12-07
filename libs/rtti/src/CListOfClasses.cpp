/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include "rtti-precomp.h"  // Precompiled headers

#include <mrpt/rtti/CListOfClasses.h>

using namespace mrpt::rtti;

bool CListOfClasses::containsDerivedFrom(
	const mrpt::rtti::TRuntimeClassId* id) const
{
	for (const auto& i : data)
		if (i->derivedFrom(id)) return true;
	return false;
}

std::string CListOfClasses::toString() const
{
	std::string s;
	unsigned int cnt = 0;
	for (const auto& i : data)
	{
		if (cnt++ != 0) s += ", ";
		s += std::string(i->className);
	}
	return s;
}
