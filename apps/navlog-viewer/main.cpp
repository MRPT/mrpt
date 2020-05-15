/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/os.h>
#include <mrpt/version.h>
#include <iostream>
#include "navlog-viewer-ui.h"

int main(int argc, char** argv)
{
	try
	{
		// Declare the supported command line switches ===========
		TCLAP::CmdLine cmd("navlog-viewer", ' ', MRPT_version_str);

		TCLAP::UnlabeledMultiArg<std::string> argInputFile(
			"LogFiles", "Input navlog files (*.reactivenavlog)", false,
			"log.reactivenavlog", cmd);

		TCLAP::ValueArg<std::string> argExtraModuleToLoad(
			"l", "load", "Load an additional DLL/SO module (*.so/*.dll)", false,
			"", "myModule.so", cmd);

		if (!cmd.parse(argc, argv)) throw std::runtime_error("");

		if (argExtraModuleToLoad.isSet())
		{
			const std::string sLib = argExtraModuleToLoad.getValue();
			std::cout << "Loading plugin modules: " << sLib << "...\n";
			mrpt::system::loadPluginModules(sLib);
		}

		// UI setup:
		nanogui::init();

		NavlogViewerApp app;

		if (argInputFile.isSet() && argInputFile.getValue().size() > 0)
			app.loadLogfile(argInputFile.getValue().at(0));

		nanogui::mainloop();

		nanogui::shutdown();

		return 0;
	}
	catch (const std::exception& e)
	{
		if (strlen(e.what()) > 0)
			std::cerr << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
