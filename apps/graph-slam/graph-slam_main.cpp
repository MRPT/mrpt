/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
    APPLICATION: Graph-SLAM hub application
    PURPOSE: Visualization of .graph files and evaluation of
             Graph-SLAM methods.

    See the "--help" output for list of supported operations and
     further instructions, the man-page or online docs at:
         http://www.mrpt.org/Application:graph-slam
 ---------------------------------------------------------------*/
#include <mrpt/graphs.h>
#include <mrpt/graphslam/levmarq.h>
#include <mrpt/gui.h>

#include <mrpt/otherlibs/tclap/CmdLine.h>

using namespace mrpt;
using namespace mrpt::graphslam;
using namespace mrpt::graphs;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

// Declarations:
#define VERBOSE_COUT	if (verbose) std::cout << "[graph-slam] "

#define DECLARE_OP_FUNCTION(_NAME) void _NAME(const std::string &in_file, bool is3D, TCLAP::CmdLine &cmdline, bool verbose)

#define IMPLEMENT_OP_FUNCTION(_NAME) \
template <class GRAPHTYPE> void _NAME##impl(const std::string &in_file, bool is3D, TCLAP::CmdLine &cmdline, bool verbose); \
void _NAME(const std::string &in_file, bool is3D, TCLAP::CmdLine &cmdline, bool verbose) \
{ \
	if (!is3D) \
	     _NAME##impl<CNetworkOfPoses2D>(in_file,is3D,cmdline,verbose);	\
	else _NAME##impl<CNetworkOfPoses3D>(in_file,is3D,cmdline,verbose);	\
} \
template <class GRAPHTYPE> void _NAME##impl(const std::string &in_file, bool is3D, TCLAP::CmdLine &cmdline, bool verbose)

/**
 * http://stackoverflow.com/questions/3982470/what-does-typedef-void-something-mean
 * Define a pointer-to-function type "TOperationFunctor". TOperationFunctor
 * returns void and takes 4 input parameters as explained in the latter
 * parentheses
 */
typedef void (*TOperationFunctor)(const std::string &in_file, bool is3D, TCLAP::CmdLine &cmdline, bool verbose);


/** Forward declaration.
 * display_graph function template provided in display_graph.cpp
 */
template <class GRAPHTYPE> void display_graph(const GRAPHTYPE & g);

DECLARE_OP_FUNCTION(op_view);
DECLARE_OP_FUNCTION(op_dijkstra);
DECLARE_OP_FUNCTION(op_levmarq);
DECLARE_OP_FUNCTION(op_info);


// Declare the supported command line switches ===========
TCLAP::CmdLine cmd("graph-slam", ' ', MRPT_getVersion().c_str());

TCLAP::ValueArg<std::string> arg_input_file  ("i","input","Input file (required) (*.graph,*.graphbin)",true,"","test.graph",cmd);
TCLAP::ValueArg<std::string> arg_output_file ("o","output","Output file (optional) (*.graph,*.graphbin)",false,"","result.graph",cmd);

TCLAP::SwitchArg arg_2d("","2d","Use 2D poses (Must use exactly one of --2d and --3d)",cmd, false);
TCLAP::SwitchArg arg_3d("","3d","Use 3D poses (Must use exactly one of --2d and --3d)",cmd, false);

TCLAP::SwitchArg arg_view("","view",
			"Op: Visualize the graph in a 3D view. If used alone, represent VERTEX2 or VERTEX3 poses directly as stored in the input file. "
			"If used together with another operation, the final obtained graph after the operation will be shown, not the input original one.\n"
			,cmd, false);

TCLAP::SwitchArg         arg_quiet("q","quiet","Terse output",cmd, false);
TCLAP::ValueArg<int>     arg_max_iters("","max-iters","Maximum number of iterations (optional)",false,100,"N",cmd);
TCLAP::ValueArg<double>  arg_initial_lambda("","initial-lambda","Initial lambda parameter (optional, lev-marq)",false,0,"val",cmd);
TCLAP::SwitchArg         arg_no_span("","no-span","Don't use dijkstra initial spanning tree guess (optional)",cmd, false);


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
    // Only one of the following operators have to be specified by the user
		map<string,TOperationFunctor>  ops_functors;

		arg_ops.push_back(new TCLAP::SwitchArg("","levmarq",
			"Op: Optimizes the graph with sparse Levenberg-Marquartd using global coordinates (via mrpt::graphslam::optimize_graph_spa_levmarq).\n"
			"   Can be used together with: --view, --output, --max-iters, --no-span, --initial-lambda"
			,cmd, false) );
		ops_functors["levmarq"] = &op_levmarq;

		arg_ops.push_back(new TCLAP::SwitchArg("","dijkstra",
			"Op: Executes CNetworkOfPoses::dijkstra_nodes_estimate() to estimate the global pose of nodes from a Dijkstra tree and the edge relative poses.\n"
			"   Can be used together with: --view, --output"
			,cmd, false) );
		ops_functors["dijkstra"] = &op_dijkstra;

		arg_ops.push_back(new TCLAP::SwitchArg("","info",
			"Op: Loads the graph and displays statistics and information on it.\n"
			,cmd, false) );
		ops_functors["info"] = &op_info;
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
    {
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
    }

		// The "--view" argument needs a bit special treatment:
		if (selected_op.empty())
		{
			if (!arg_view.isSet())
				throw std::runtime_error(
					"Don't know what to do: No operation was indicated.\n"
					"Use --help to see the list of possible operations.");
			else
			{
				VERBOSE_COUT << "Operation to perform: " "view" << endl;
				op_view(input_file,is3d,cmd,verbose);
			}
		}
		else
		{
			VERBOSE_COUT << "Operation to perform: " << selected_op << endl;

			// ------------------------------------
			//  EXECUTE THE REQUESTED OPERATION
			// ------------------------------------
			ASSERTMSG_(ops_functors.find(selected_op)!=ops_functors.end(), "Internal error: Unknown operation functor!")

			// Call the selected functor:
			ops_functors[selected_op](input_file,is3d,cmd,verbose);
		}

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

// ===================================================================================
//     Implementation of the functions operations
// ===================================================================================

// -----------------------------------------------------------------------------------
//  op: --view
//   Load and visualize the graph
// -----------------------------------------------------------------------------------
IMPLEMENT_OP_FUNCTION(op_view)
{
	MRPT_UNUSED_PARAM(is3D); MRPT_UNUSED_PARAM(cmdline); MRPT_UNUSED_PARAM(verbose);
	// Load:
	GRAPHTYPE g;
	g.loadFromTextFile(in_file);

	// Display:
	display_graph(g);
}

// -----------------------------------------------------------------------------------
//  op: --info
//   Load and visualize basic info on graph
// -----------------------------------------------------------------------------------
IMPLEMENT_OP_FUNCTION(op_info)
{
	MRPT_UNUSED_PARAM(is3D); MRPT_UNUSED_PARAM(cmdline); MRPT_UNUSED_PARAM(verbose);
	// Load:
	GRAPHTYPE g;
	g.loadFromTextFile(in_file);

	set<TNodeID>  lstNodeIDs;
	g.getAllNodes(lstNodeIDs);

	// Show graph stats:
	cout << "Edge count                         : " << g.edgeCount() << endl;
	cout << "Nodes count (in VERTEX2/3 entries) : " << g.nodes.size() << endl;
	cout << "Nodes count (in edge entries)      : " << lstNodeIDs.size() << endl;

}

// -----------------------------------------------------------------------------------
//  op: --dijkstra
//   Load and visualize the graph
// -----------------------------------------------------------------------------------
IMPLEMENT_OP_FUNCTION(op_dijkstra)
{
	MRPT_UNUSED_PARAM(is3D); MRPT_UNUSED_PARAM(cmdline); MRPT_UNUSED_PARAM(verbose);
	const bool save_to_file = arg_output_file.isSet();	// Output to file??
	const bool display_3D   = arg_view.isSet();         // Output to 3D view??

	if (!save_to_file && !display_3D)
		std::cerr << "\n ** WARNING **: Neither --view nor --output specified. The result will not be saved anywhere.\n\n";

	// Load:
	GRAPHTYPE g;
	g.loadFromTextFile(in_file);

	// Find the first node ID and use it as root:
	VERBOSE_COUT << "Making a list with all node IDs...\n";

	set<TNodeID>  lstNodeIDs;
	g.getAllNodes(lstNodeIDs);

	ASSERT_(!lstNodeIDs.empty())

	const TNodeID id_root = *lstNodeIDs.begin();
	VERBOSE_COUT << "Using root node ID=" << id_root << endl;

	g.root = id_root;

	VERBOSE_COUT << "Executing Dijkstra...\n";
	CTicTac  tictac;
	tictac.Tic();

	g.dijkstra_nodes_estimate();

	VERBOSE_COUT << "Took: " << mrpt::system::formatTimeInterval(tictac.Tac()) << endl;

	if (save_to_file)
	{
		const string out_file = arg_output_file.getValue();
		VERBOSE_COUT  << "Saving resulting graph to: " << out_file << endl;
		g.saveToTextFile(out_file);
	}
	if (display_3D)
	{
		VERBOSE_COUT  << "Displaying resulting graph\n";
		display_graph(g);
	}
} // end op_dijkstra


// -----------------------------------------------------------------------------------
//  op: --levmarq
//   Load and optimize a graph with the sparse Lev-Marq algorithm.
// -----------------------------------------------------------------------------------
IMPLEMENT_OP_FUNCTION(op_levmarq)
{
	MRPT_UNUSED_PARAM(is3D); MRPT_UNUSED_PARAM(cmdline);
	const bool save_to_file  = arg_output_file.isSet();	// Output to file??
	const bool display_3D    = arg_view.isSet();         // Output to 3D view??
	const bool skip_dijkstra = arg_no_span.isSet();

	if (!save_to_file && !display_3D)
		std::cerr << "\n ** WARNING **: Neither --view nor --output specified. The result will not be saved anywhere.\n\n";

	// Load:
	GRAPHTYPE g;
	g.loadFromTextFile(in_file);

	// Find the first node ID and use it as root:
	VERBOSE_COUT << "Making a list with all node IDs...\n";

	set<TNodeID>  lstNodeIDs;
	g.getAllNodes(lstNodeIDs);

	ASSERT_(!lstNodeIDs.empty())

	const TNodeID id_root = *lstNodeIDs.begin();
	VERBOSE_COUT << "Using root node ID=" << id_root << endl;

	g.root = id_root;

	// Executes dijkstra (spanning tree) for initial guess?
	// --------------------------------
	if (!skip_dijkstra)
	{
		VERBOSE_COUT << "Executing Dijkstra...\n";
		CTicTac  tictac;
		tictac.Tic();

		g.dijkstra_nodes_estimate();

		VERBOSE_COUT << "Took: " << mrpt::system::formatTimeInterval(tictac.Tac()) << endl;
	}

	// Executes the optimization:
	// --------------------------------
	TParametersDouble  params;
	params["verbose"]  = verbose;
	params["profiler"] = verbose;
	params["max_iterations"] = arg_max_iters.getValue();
	params["initial_lambda"] = arg_initial_lambda.getValue();

	graphslam::TResultInfoSpaLevMarq  info;

	graphslam::optimize_graph_spa_levmarq(g,info, NULL, params);


	// Output:
	if (save_to_file)
	{
		const string out_file = arg_output_file.getValue();
		VERBOSE_COUT  << "Saving resulting graph to: " << out_file << endl;
		g.saveToTextFile(out_file);
	}
	if (display_3D)
	{
		VERBOSE_COUT  << "Displaying resulting graph\n";
		display_graph(g);
	}
} // end op_levmarq

