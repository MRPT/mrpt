/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/gui/registerAllClasses.h>
#include <mrpt/slam/registerAllClasses.h>

namespace mrpt::graphslam
{
/** Forces manual RTTI registration of all serializable classes in this
 * namespace. Should never be required to be explicitly called by users, except
 * if building MRPT as a static library.
 *
 * \ingroup mrpt_graphslam_grp
 */
inline void registerAllClasses_mrpt_graphslam()
{
	mrpt::gui::registerAllClasses_mrpt_gui();
	mrpt::slam::registerAllClasses_mrpt_slam();
}
}  // namespace mrpt::graphslam
