/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
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