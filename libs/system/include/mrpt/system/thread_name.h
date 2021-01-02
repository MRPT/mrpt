/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>
#include <thread>

namespace mrpt::system
{
/** Sets the name of the given thread; useful for debuggers.
 * \ingroup mrpt_system_grp
 * \note New in MRPT 2.0.4
 */
void thread_name(const std::string& name, std::thread& theThread);

/** Sets the name of the current thread; useful for debuggers.
 * \ingroup mrpt_system_grp
 * \note New in MRPT 2.0.4
 */
void thread_name(const std::string& name);

/** Gets the name of the given thread; useful for debuggers.
 * \ingroup mrpt_system_grp
 * \note New in MRPT 2.0.4
 */
std::string thread_name(std::thread& theThread);

/** Gets the name of the current thread; useful for debuggers.
 * \ingroup mrpt_system_grp
 * \note New in MRPT 2.0.4
 */
std::string thread_name();

}  // namespace mrpt::system