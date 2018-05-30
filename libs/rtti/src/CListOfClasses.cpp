/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include "rtti-precomp.h"  // Precompiled headers

#include <mrpt/rtti/CListOfClasses.h>
#include <sstream>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>

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

// trim from start
static inline std::string& ltrim(std::string& s)
{
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](const char c) {
				return !std::isspace(c);
			}));
	return s;
}

// trim from end
static inline std::string& rtrim(std::string& s)
{
	s.erase(
		std::find_if(
			s.rbegin(), s.rend(), [](const char c) { return !std::isspace(c); })
			.base(),
		s.end());
	return s;
}

// trim from both ends
static inline std::string& trim(std::string& s) { return ltrim(rtrim(s)); }
void CListOfClasses::fromString(const std::string& s)
{
	MRPT_TRY_START

	data.clear();
	std::stringstream ss(s);
	std::string lin;
	while (std::getline(ss, lin, ','))
	{
		lin = trim(lin);
		const TRuntimeClassId* id = findRegisteredClass(lin);
		ASSERTMSG_(
			id != nullptr, format("Unknown class name: %s", lin.c_str()));
		this->insert(id);
	}

	MRPT_TRY_END
}
