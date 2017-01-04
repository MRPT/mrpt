/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/system/os.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/filesystem.h>

#include <mrpt/otherlibs/tclap/CmdLine.h>

int main(int argc, char **argv)
{
	try
	{
		// Declare the supported command line switches ===========
		TCLAP::CmdLine cmd("image2gridmap", ' ', mrpt::system::MRPT_getVersion().c_str());

		TCLAP::ValueArg<std::string> arg_input_file ("i","input","Input image file (required) (*.png,*.jpg,...)",true,"","map_image.png",cmd);
		TCLAP::ValueArg<std::string> arg_output_file("o","output","Name of the output file (*.gridmap, *.gridmap.gz)",false,"","map.gridmap.gz",cmd);
		TCLAP::ValueArg<double>      arg_res("r","res","Resolution: size (in meters) of one pixel in the image (required)",true,0.1,"0.1",cmd);
		TCLAP::ValueArg<double>      arg_cx("","cx","(Use either --cx or --px) X coordinate of the image central pixel (Default:0)",false,0.0,"0.0",cmd);
		TCLAP::ValueArg<double>      arg_cy("","cy","(Use either --cy or --py) Y coordinate of the image central pixel (Default:0)",false,0.0,"0.0",cmd);
		TCLAP::ValueArg<double>      arg_px("","px","(Use either --cx or --px) Pixel horizontal coordinate of the origin of coordinates in the image",false,0.0,"0.0",cmd);
		TCLAP::ValueArg<double>      arg_py("","py","(Use either --cx or --px) Pixel verticl coordinate of the origin of coordinates in the image",false,0.0,"0.0",cmd);
		TCLAP::SwitchArg arg_overwrite("w","overwrite","Force overwrite target file without prompting.",cmd, false);

		printf(" image2gridmap - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - Sources timestamp: %s\n", mrpt::system::MRPT_getVersion().c_str(), mrpt::system::MRPT_getCompilationDate().c_str());
		printf("-------------------------------------------------------------------\n");

		// Parse arguments:
		if (!cmd.parse( argc, argv ))
			throw std::runtime_error(""); // should exit.

		const std::string inputFile = arg_input_file.getValue();
		const double cell_res  = arg_res.getValue();

		mrpt::utils::CImage img;
		if (!img.loadFromFile(inputFile)) 
			throw std::runtime_error(mrpt::format("Cannot load the map image file `%s`!",inputFile.c_str()));

		double px,py;
		if ( (arg_px.isSet() && !arg_py.isSet()) || (!arg_px.isSet() && arg_py.isSet()) )
			throw std::runtime_error("You cannot set only one of --px & --py arguments!");

		if ( (arg_cx.isSet() && !arg_cy.isSet()) || (!arg_cx.isSet() && arg_cy.isSet()) )
			throw std::runtime_error("You cannot set only one of --cx & --cy arguments!");

		if ( arg_cx.isSet() && arg_px.isSet() )
			throw std::runtime_error("You cannot set BOTH --cx & --px arguments!");

		if ( arg_px.isSet() ) {
			px=arg_px.getValue();
			py=arg_py.getValue();
		} else {
			px = -arg_cx.getValue() / cell_res + img.getWidth()/2;
			py = -arg_cy.getValue() / cell_res + img.getHeight()/2;
		}

		mrpt::maps::COccupancyGridMap2D grid;
		grid.loadFromBitmap(img, cell_res, px,py);

		const std::string sOutFile =  arg_output_file.isSet() ?
			arg_output_file.getValue()
			:
			mrpt::system::fileNameChangeExtension(inputFile,"gridmap.gz");
		std::cout << "Output map file: " << sOutFile << std::endl;

		if (mrpt::system::fileExists(sOutFile) && !arg_overwrite.isSet())
		{
			std::cerr << "Output file already exists, aborting. Use `-w` flag to overwrite.\n";
			return 1;
		}

		{
			mrpt::utils::CFileGZOutputStream f(sOutFile);
			f << grid;
		}

		std::cout << "All done.\n";

		return 0;
	}
	catch (std::exception &e)
	{
		if (strlen(e.what())) std::cerr << e.what() << std::endl;
		return 1;
	}
}

