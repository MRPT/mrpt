/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt::detectors
{
/** Forces manual RTTI registration of all serializable classes in this
 * namespace. Should never be required to be explicitly called by users, except
 * if building MRPT as a static library.
 *
 * \ingroup mrpt_detectors_grp
 */
void registerAllClasses_mrpt_detectors();
}  // namespace mrpt::detectors
