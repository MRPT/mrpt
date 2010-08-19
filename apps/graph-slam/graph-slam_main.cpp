/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
    APPLICATION: Graph-SLAM hub application
    PURPOSE: Visualization of .graph files and evaluation of
             Graph-SLAM methods.

    See the "--help" output for list of supported operations and
     further instructions, the man-page or online docs at:
         http://www.mrpt.org/Application:graph-slam
 ---------------------------------------------------------------*/

#include <mrpt/slam.h>
#include <mrpt/gui.h>

#include <mrpt/otherlibs/tclap/CmdLine.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

// Declarations:
#define VERBOSE_COUT	if (verbose) std::cout << "[graph-slam] "
#define DECLARE_OP_FUNCTION(_NAME) void _NAME(const std::string &in_file, bool is3D, TCLAP::CmdLine &cmdline, bool verbose)

typedef void (*TOperationFunctor)(const std::string &in_file, bool is3D, TCLAP::CmdLine &cmdline, bool verbose);


// Frwd. decl:
DECLARE_OP_FUNCTION(op_view);


// Declare the supported command line switches ===========
TCLAP::CmdLine cmd("graph-slam", ' ', MRPT_getVersion().c_str());

TCLAP::ValueArg<std::string> arg_input_file ("i","input","Input file (required) (*.graph,*.graphbin)",true,"","test.graph",cmd);

TCLAP::SwitchArg arg_2d("","2d","Use 2D poses (Must use exactly one of --2d and --3d)",cmd, false);
TCLAP::SwitchArg arg_3d("","3d","Use 3D poses (Must use exactly one of --2d and --3d)",cmd, false);

TCLAP::SwitchArg arg_quiet("q","quiet","Terse output",cmd, false);



// ======================================================================
//     main() of graph-slam
// ======================================================================
int main(int argc, char **argv)
{
	vector<TCLAP::Arg*> arg_ops;  // to be destroyed on exit.
	int ret_val = 0;

	try
	{
		// --------------- List of possible operations ---------------
		map<string,TOperationFunctor>  ops_functors;

		arg_ops.push_back(new TCLAP::SwitchArg("","view",
			"Op: Visualize the VERTEX2 or VERTEX3 poses directly as stored in the input file, wait for user to close the window and exit.\n"
			,cmd, false) );
		ops_functors["view"] = &op_view;
		// --------------- End of list of possible operations --------

		// Parse arguments:
		if (!cmd.parse( argc, argv ))
			throw std::runtime_error(""); // should exit.

		// Exactly 1 or --2d & --3d must be specified:
		if ( (arg_2d.isSet() && arg_3d.isSet()) || (!arg_2d.isSet() && !arg_3d.isSet()) )
			throw std::runtime_error("Exactly one --2d or --3d must be used.");

		const bool is3d = arg_3d.isSet();

		string input_file  = arg_input_file.getValue();
		const bool verbose = !arg_quiet.getValue();

		// Check the selected operation:
		//  Only one of the ops should be selected:
		string selected_op;
		for (size_t i=0;i<arg_ops.size();i++)
			if (arg_ops[i]->isSet())
			{
				if (selected_op.empty())
				{
					selected_op = arg_ops[i]->getName();
				}
				else	throw std::runtime_error(
					"Exactly one operation must be indicated on command line.\n"
					"Use --help to see the list of possible operations.");
			}

		if (selected_op.empty())
		{
			throw std::runtime_error(
				"Don't know what to do: No operation was indicated.\n"
				"Use --help to see the list of possible operations.");
		}

		VERBOSE_COUT << "Operation to perform: " << selected_op << endl;

		// ------------------------------------
		//  EXECUTE THE REQUESTED OPERATION
		// ------------------------------------
		ASSERTMSG_(ops_functors.find(selected_op)!=ops_functors.end(), "Internal error: Unknown operation functor!")

		// Call the selected functor:
		ops_functors[selected_op](input_file,is3d,cmd,verbose);

		// successful end of program.
		ret_val = 0;
	}
	catch(std::exception &e)
	{
		if (strlen(e.what())) std::cerr << e.what() << std::endl;
		ret_val = -1;
	}

	// Free mem:
	for (size_t i=0;i<arg_ops.size();i++)
		delete arg_ops[i];

	// end:
	return ret_val;
}


DECLARE_OP_FUNCTION(op_view)
{
	// ...
}

