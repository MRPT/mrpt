/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt::comms
{
/** Forces manual RTTI registration of all serializable classes in this
 * namespace. Should never be required to be explicitly called by users, except
 * if building MRPT as a static library.
 *
 * \ingroup mrpt_comms_grp
 */
inline void registerAllClasses_mrpt_comms()
{
	// None in this library
}
}  // namespace mrpt::comms
