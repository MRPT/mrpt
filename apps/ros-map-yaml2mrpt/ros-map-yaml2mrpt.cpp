/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

int main(int argc, char** argv)
{
	try
	{
		// Declare the supported command line switches ===========
		TCLAP::CmdLine cmd(
			"ros-map-yaml2mrpt", ' ', mrpt::system::MRPT_getVersion().c_str());

		TCLAP::ValueArg<std::string> argInputFile(
			"i", "input", "Input map yaml file (required) (*.yaml)", true,
			"<map.yaml>", "map.yaml", cmd);

		TCLAP::ValueArg<std::string> argOutputDirectory(
			"d", "output-directory",
			"If provided, output files will be written to the specified "
			"directory, instead of the same directory of the input file, which "
			"is the default behavior. The output directory must exist, it will "
			"not be created.",
			false, "", ".", cmd);

		TCLAP::SwitchArg argOverwrite(
			"w", "overwrite", "Force overwrite target file without prompting.",
			cmd, false);

		TCLAP::SwitchArg argQuiet(
			"q", "quiet",
			"Do not print info messages to cout, only errors to cerr", cmd,
			false);

		TCLAP::SwitchArg argGenerate3D(
			"", "generate-3d",
			"Create a .3Dscene view of the gridmap, suitable for quick "
			"visualization in the SceneViewer3D program.",
			cmd, false);

		if (!argQuiet.isSet())
		{
			printf(	 //
				" ros-map-yaml2mrpt - Part of %s\n"
				"------------------------------------------------------------"
				"\n",
				mrpt::system::MRPT_getVersion().c_str());
		}

		// Parse arguments:
		if (!cmd.parse(argc, argv))
			throw std::runtime_error("");  // should exit.

		const std::string inputFile = argInputFile.getValue();

		const auto grid =
			mrpt::maps::COccupancyGridMap2D::FromROSMapServerYAML(inputFile);

		const std::string outDir = argOutputDirectory.isSet()
			? argOutputDirectory.getValue()
			: mrpt::system::extractFileDirectory(inputFile);

		const std::string outGridFil = mrpt::system::pathJoin(
			{outDir,
			 mrpt::system::fileNameChangeExtension(
				 mrpt::system::extractFileName(inputFile), "gridmap.gz")});

		std::string out3D;
		if (argGenerate3D.isSet())
		{
			out3D = mrpt::system::pathJoin(
				{outDir,
				 mrpt::system::fileNameChangeExtension(
					 mrpt::system::extractFileName(inputFile), "3Dscene")});
		}

		if (!argQuiet.isSet())
		{
			std::cout << "Input file        : " << inputFile << "\n";
			std::cout << "Output gridmap    : " << outGridFil << std::endl;
			if (!out3D.empty())
				std::cout << "Output 3D view    : " << out3D << std::endl;
		}

		if (mrpt::system::fileExists(outGridFil) && !argOverwrite.isSet())
		{
			std::cerr << "Output gridmap file already exists, aborting. Use "
						 "`-w` flag "
						 "to overwrite."
					  << std::endl;
			return 1;
		}
		if (mrpt::system::fileExists(out3D) && !argOverwrite.isSet())
		{
			std::cerr
				<< "Output 3D file already exists, aborting. Use `-w` flag "
				   "to overwrite."
				<< std::endl;
			return 1;
		}

		{
			mrpt::io::CFileGZOutputStream f(outGridFil);
			mrpt::serialization::archiveFrom(f) << grid;
		}

		if (!out3D.empty())
		{
			mrpt::opengl ::Scene scene;
			scene.insert(grid.getVisualization());
			scene.saveToFile(out3D);
		}

		std::cout << "All done.\n";

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		return 1;
	}
}
