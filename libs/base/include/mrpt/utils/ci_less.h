/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <functional>  // binary_function
#include <cctype>  // tolower
#include <string>

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup stlext_grp STL extensions and metaprogramming 
		  * \ingroup mrpt_base_grp
		  * @{ */

		/** A case-insensitive comparator struct for usage within STL containers, eg: map<string,string,ci_less>
		  * \note Defined in #include <mrpt/utils/ci_less.h>
		  */
		struct ci_less : std::binary_function<std::string,std::string,bool>
		{
			// case-independent (ci) compare_less binary function
			struct nocase_compare : public std::binary_function<char,char,bool> {
				bool operator()(const char c1, const char c2) const { return tolower(c1)<tolower(c2); }
			};
			bool operator() (const std::string & s1, const std::string & s2) const {
				return std::lexicographical_compare(s1.begin(),s1.end(), s2.begin(),s2.end(), nocase_compare());
			}
		}; // end of ci_less

		/** @} */  // end of grouping
	} // End of namespace
} // End of namespace
