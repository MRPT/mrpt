/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>

namespace mrpt::containers
{
/** A case-insensitive comparator class for use within STL containers, etc.
 * \note Credits: https://stackoverflow.com/a/1801913/1631514
 * \ingroup mrpt_containers_grp
 */
struct ci_less
{
	struct nocase_compare
	{
		bool operator()(const unsigned char& c1, const unsigned char& c2) const
		{
			return tolower(c1) < tolower(c2);
		}
	};
	bool operator()(const std::string& s1, const std::string& s2) const
	{
		return std::lexicographical_compare(
			s1.begin(), s1.end(),  // source range
			s2.begin(), s2.end(),  // dest range
			nocase_compare());	// comparison
	}
};

}  // namespace mrpt::containers
