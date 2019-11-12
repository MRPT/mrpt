/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>

namespace mrpt::apps
{
/** Virtual interface for applications that initialize from CLI parameters.
 *
 * \ingroup mrpt_apps_grp
 */
class BaseAppInitializableCLI
{
   public:
	BaseAppInitializableCLI() = default;
	virtual ~BaseAppInitializableCLI() = default;

   protected:
	virtual void impl_initialize(int argc, const char** argv) = 0;
	virtual std::string impl_get_usage() const = 0;
};

}  // namespace mrpt::apps
