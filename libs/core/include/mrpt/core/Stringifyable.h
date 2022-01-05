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

namespace mrpt
{
/** Interface for classes whose state can be represented as a human-friendly
 * text.
 *
 * \ingroup mrpt_core_grp
 */
class Stringifyable
{
   public:
	Stringifyable() = default;
	~Stringifyable() = default;

	/** Returns a human-friendly textual description of the object. For classes
	 * with a large/complex internal state, only a summary should be returned
	 * instead of the exhaustive enumeration of all data members.
	 */
	virtual std::string asString() const = 0;
};

}  // namespace mrpt
