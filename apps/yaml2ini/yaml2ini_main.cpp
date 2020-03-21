/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/vector_loadsave.h>
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
		if (argc != 2) throw std::runtime_error("Usage: yaml2ini <input.yaml>");

		const std::string ymlFile = std::string(argv[1]);
		ASSERT_FILE_EXISTS_(ymlFile);

		std::vector<uint8_t> buf;
		if (!mrpt::io::loadBinaryFile(buf, ymlFile))
			THROW_EXCEPTION("loadBinaryFile() failed (?).");

		std::string strYaml;
		strYaml.resize(buf.size());
		std::copy(buf.begin(), buf.end(), strYaml.begin());

		mrpt::config::CConfigFileMemory cfg;
		cfg.setContentFromYAML(strYaml);

		std::cout << cfg.getContent() << "\n";

		// successful end of program.
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
