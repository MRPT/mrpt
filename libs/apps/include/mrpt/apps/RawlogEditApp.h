/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstddef>

namespace mrpt::apps
{
/** The C++ class behinds the rawlog-edit CLI tool.
 *
 *  Refer to the online documentation for rawlog-edit.
 * \ingroup mrpt_apps_grp
 */
class RawlogEditApp
{
   public:
	RawlogEditApp() = default;

	/** @name Main API
	 * @{ */

	/** Initializes and runs the application from CLI parameters. Refer to the
	 * manpage of rawlog-edit. Throws on errors.
	 */
	void run(int argc, const char** argv);

	void run(int argc, char** argv)
	{
		run(argc, const_cast<const char**>(argv));
	}

	/** @} */
};

}  // namespace mrpt::apps
