/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/config/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <iostream>

// -----------------------------------------------
//				MAIN
// -----------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		// Parse arguments:
		if (argc != 2) throw std::runtime_error("Usage: ini2yaml <input.ini>");

		const std::string iniFile = std::string(argv[1]);
		ASSERT_FILE_EXISTS_(iniFile);

		mrpt::config::CConfigFile cfg(iniFile);
		const std::string s = cfg.getContentAsYAML();

		std::cout << s << "\n";

		// successful end of program.
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
